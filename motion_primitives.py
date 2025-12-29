#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Motion Planning Interface
将从视觉获取的抓取点和放置点转换为机械臂运动序列
"""

import numpy as np
import time
import sys
from reset_robot_safe import reset_arm, load_home_config
from scipy.spatial.transform import Rotation as R

def robust_input(prompt=""):
    """
    Robust input function that works even after OpenCV operations
    OpenCV can corrupt stdin, so we try to read from /dev/tty directly
    """
    print(prompt, end='', flush=True)
    
    # Try to read from /dev/tty directly (bypasses corrupted stdin)
    try:
        with open('/dev/tty', 'r') as tty:
            return tty.readline().rstrip('\n')
    except:
        # Fallback to regular input
        try:
            return input()
        except EOFError:
            raise EOFError("Cannot read input (EOF). Please run in interactive terminal.")

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (RPY) to quaternion [w, x, y, z]
    
    Args:
        roll, pitch, yaw: Rotation angles in radians
        
    Returns:
        quaternion [w, x, y, z]
    """
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # Returns [x, y, z, w]
    return np.array([quat[3], quat[0], quat[1], quat[2]])  # Convert to [w, x, y, z]

def quaternion_to_euler(quat):
    """
    Convert quaternion [w, x, y, z] to Euler angles (RPY)
    
    Args:
        quat: quaternion [w, x, y, z]
        
    Returns:
        [roll, pitch, yaw] in radians
    """
    # scipy expects [x, y, z, w]
    rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    return rot.as_euler('xyz')

def quaternion_slerp(q1, q2, t):
    """
    Spherical linear interpolation between two quaternions
    
    Args:
        q1: Starting quaternion [w, x, y, z]
        q2: Ending quaternion [w, x, y, z]
        t: Interpolation parameter (0 to 1)
        
    Returns:
        Interpolated quaternion [w, x, y, z]
    """
    from scipy.spatial.transform import Slerp
    
    # Convert to scipy format [x, y, z, w]
    q1_scipy = [q1[1], q1[2], q1[3], q1[0]]
    q2_scipy = [q2[1], q2[2], q2[3], q2[0]]
    
    # Create key rotations and times for Slerp
    key_rots = R.from_quat([q1_scipy, q2_scipy])
    key_times = [0, 1]
    
    # Perform spherical linear interpolation
    slerp_interp = Slerp(key_times, key_rots)
    result_rot = slerp_interp([t])[0]
    
    # Convert back to [w, x, y, z] format
    result_quat = result_rot.as_quat()  # [x, y, z, w]
    return np.array([result_quat[3], result_quat[0], result_quat[1], result_quat[2]])  # [w, x, y, z]

def rotation_matrix_from_euler(roll, pitch, yaw):
    """
    Create rotation matrix from Euler angles (RPY convention)
    
    Args:
        roll, pitch, yaw: Rotation angles in radians
        
    Returns:
        3x3 rotation matrix
    """
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    return rot.as_matrix()

def apply_hand_offset(position, orientation, offset_magnitude, direction='y'):
    """
    Apply hand offset in end-effector frame, transformed to base frame
    
    Args:
        position: [x, y, z] position in base frame
        orientation: [roll, pitch, yaw] orientation in radians
        offset_magnitude: Magnitude of offset (e.g., 0.12 meters)
        direction: Direction in end-effector frame ('x', 'y', or 'z')
        
    Returns:
        Adjusted position [x, y, z]
    """
    # Define offset vector in end-effector frame
    if direction == 'x':
        offset_ee = np.array([offset_magnitude, 0, 0])
    elif direction == 'y':
        offset_ee = np.array([0, offset_magnitude, 0])
    elif direction == 'z':
        offset_ee = np.array([0, 0, offset_magnitude])
    else:
        raise ValueError(f"Invalid direction: {direction}")
    
    # Get rotation matrix from orientation
    R = rotation_matrix_from_euler(orientation[0], orientation[1], orientation[2])
    
    # Transform offset to base frame
    offset_base = R @ offset_ee
    
    # Apply offset to position
    adjusted_position = np.array(position) + offset_base
    
    return adjusted_position

class MotionPlanner:
    def __init__(self, robot_controller):
        """
        Args:
            robot_controller: RealRobotController instance
        """
        self.robot = robot_controller
        
        # Configuration
        self.APPROACH_OFFSET = 0.092  # Approach from 9.2cm above target (meters)
        self.HAND_OFFSET = 0.12       # End-effector to palm center offset
        self.LIFT_HEIGHT = 0.1        # Lift 10cm after grasp
        self.MOVE_SPEED = 5          # Movement speed (0-100)
        self.STEP_SIZE = 0.05         # Step size for safety (meters)
        self.FINAL_DROP_HEIGHT = 0.02  # 10cm - height above target position to drop from
        # Custom orientation for left and right hands (roll, pitch, yaw) in radians
        self.LEFT_HAND_ORIENTATION = [-2.676000, -1.347000, 0.429000]   # Default orientation for left hand
        self.RIGHT_HAND_ORIENTATION = [-3.078000, -1.306000, -1.058000]  # Default orientation for right hand

    def execute_stepwise_motion(self, active_arm, target_pose, label="Motion"):
        """
        Execute motion in small steps with user confirmation
        Uses quaternion SLERP for smooth orientation interpolation
        """
        current_pose = active_arm.get_current_pose()
        start_pos = np.array(current_pose[:3])
        end_pos = np.array(target_pose[:3])
        
        # Convert Euler angles to quaternions for interpolation
        start_euler = np.array(current_pose[3:])
        target_euler = np.array(target_pose[3:])
        
        start_quat = euler_to_quaternion(start_euler[0], start_euler[1], start_euler[2])
        target_quat = euler_to_quaternion(target_euler[0], target_euler[1], target_euler[2])
        
        dist = np.linalg.norm(end_pos - start_pos)
        
        if dist < 0.001:
            print(f"  [INFO] Already at target ({label})")
            return True

        num_steps = int(np.ceil(dist / self.STEP_SIZE))
        if num_steps < 1: num_steps = 1
        
        print(f"\n  [STEPWISE] {label}")
        print(f"  Distance: {dist:.3f}m | Steps: {num_steps} | Step size: ~{self.STEP_SIZE*100:.1f}cm")
        print("  Controls: ENTER = Next step, 'q' = Quit")
        
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            
            # Interpolate position linearly
            interp_pos = start_pos + (end_pos - start_pos) * ratio
            
            # Interpolate orientation using quaternion SLERP
            interp_quat = quaternion_slerp(start_quat, target_quat, ratio)
            interp_euler = quaternion_to_euler(interp_quat)
            
            interp_pose = np.concatenate([interp_pos, interp_euler])
            
            # Always prompt for user confirmation
            try:
                user_input = robust_input(f"    Step {i}/{num_steps} [{interp_pos[0]:.3f}, {interp_pos[1]:.3f}, {interp_pos[2]:.3f}] > ").strip().lower()
                if user_input == 'q':
                    print("  [INFO] Motion cancelled by user")
                    return False
            except EOFError as e:
                print(f"\n  [ERROR] {e}")
                return False
                
            # Execute step
            active_arm.robot.Clear_System_Err()
            active_arm.move_pose_Cmd(interp_pose.tolist(), self.MOVE_SPEED)
            
            # Wait a bit for the move to complete
            time.sleep(0.5)
            
        print(f"  ✓ {label} Completed")
        return True

    def confirm_phase(self, phase_name):
        """Ask for user confirmation before starting a phase"""
        print(f"\n" + "-"*40)
        try:
            ans = robust_input(f"[CONFIRM] Press ENTER to execute {phase_name} (or 'q' to quit): ").strip().lower()
            if ans == 'q':
                print("[INFO] Quit requested.")
                return False
            return True
        except EOFError as e:
            print(f"\n[ERROR] {e}")
            return False

    def execute_fold(self, arm_select, grasp_pos, place_pos, lift_height=None, reset_joints=None):
        """
        Execute generic folding motion (Grasp -> Lift -> Move -> Place -> Reset)
        
        Args:
            arm_select: 'left' or 'right'
            grasp_pos: [x, y, z] target position for grasping
            place_pos: [x, y, z] target position for placing
            lift_height: Lift height (optional, defaults to config)
            reset_joints: [j1...j7] Joint angles to reset to (optional)
            
        Returns:
            bool: True if successful
        """
        if lift_height is None:
            lift_height = self.LIFT_HEIGHT

        try:
            print(f"\n[Motion] Executing Fold Sequence with {arm_select} arm")
            
            # Setup arm and hand functions
            if arm_select == 'left':
                active_arm = self.robot.left_arm
                close_hand_fn = self.robot.close_left_hand
                open_hand_fn = self.robot.open_left_hand
            else:
                active_arm = self.robot.right_arm
                close_hand_fn = self.robot.close_right_hand
                open_hand_fn = self.robot.open_right_hand

            # Get current pose to keep orientation
            current_pose = active_arm.get_current_pose()
            print(f"[INFO] Current Pose: {current_pose[:3]}")

            # ---------------------------------------------------------
            # 1. Approach & Move to Grasp
            # ---------------------------------------------------------
            if not self.confirm_phase("Phase 1: Move to Grasp"): return False
                
            target_grasp = np.array(grasp_pos)
            # 根据左右手分别处理高度(Z)和手掌偏移(Y)
            # Phase 1 uses simple Y-axis offset and keeps current orientation
            if arm_select == 'right':
                target_grasp[1] += self.HAND_OFFSET # 末端 -> 手心 (+Y)
                target_grasp[2] = -0.105+0.03  # 右手经验抓取高度
            else: # left
                target_grasp[1] -= self.HAND_OFFSET # 末端 -> 手心 (-Y)
                target_grasp[2] = -0.105+0.03  # 左手经验抓取高度
            
            grasp_pose = np.array([
                target_grasp[0], target_grasp[1], target_grasp[2],
                current_pose[3], current_pose[4], current_pose[5]  # Keep current orientation
            ])

            if not self.execute_stepwise_motion(active_arm, grasp_pose, label="Phase 1 (Move to Grasp)"): return False

            # ---------------------------------------------------------
            # 2. Grasp
            # ---------------------------------------------------------
            if not self.confirm_phase("Phase 2: Grasp"): return False
            close_hand_fn()
            time.sleep(0.5)

            # ---------------------------------------------------------
            # 3. Lift
            # ---------------------------------------------------------
            if not self.confirm_phase("Phase 3: Lift"): return False

            current_pose = active_arm.get_current_pose()
            lift_target = np.array(current_pose)
            lift_target[2] = self.FINAL_DROP_HEIGHT
            
            if not self.execute_stepwise_motion(active_arm, lift_target, label="Phase 3 (Lift)"): return False

            # ---------------------------------------------------------
            # 4. Move to Target (Place Position)
            # ---------------------------------------------------------
            if not self.confirm_phase("Phase 4: Move to Target"): return False

            # Use custom orientation based on left/right hand
            if arm_select == 'right':
                orientation = self.RIGHT_HAND_ORIENTATION
            else:
                orientation = self.LEFT_HAND_ORIENTATION
            
            target_place = np.array(place_pos)
            
            # Temporarily disabled hand offset for debugging
            # Apply hand offset using rotation-aware transformation
            # The offset represents end-effector to palm center distance
            # offset_sign = 1.0 if arm_select == 'right' else -1.0
            # target_place_adjusted = apply_hand_offset(
            #     target_place, 
            #     orientation, 
            #     offset_sign * self.HAND_OFFSET,
            #     direction='y'  # Offset is along Y-axis in end-effector frame
            # )
            # target_place_adjusted[2] = self.FINAL_DROP_HEIGHT
            
            # Use max height logic
            #target_z = max(lift_target[2], target_place[2] + lift_height)
            
            place_pose_high = np.array([
                target_place[0], target_place[1], self.FINAL_DROP_HEIGHT+0.05,
                orientation[0], orientation[1], orientation[2]
            ])

            if not self.execute_stepwise_motion(active_arm, place_pose_high, label="Phase 4 (Move to Target)"): return False

            # ---------------------------------------------------------
            # 5. Release (Place)
            # ---------------------------------------------------------
            if not self.confirm_phase("Phase 5: Release"): return False
            open_hand_fn()
            time.sleep(0.5)

            # ---------------------------------------------------------
            # 6. Reset (Joint Reset)
            # ---------------------------------------------------------
            if reset_joints is not None:
                if not self.confirm_phase("Phase 6: Reset (Joints)"): return False
                # Use reset_arm from reset_robot_safe.py
                success = reset_arm(active_arm, reset_joints, arm_name=f"{arm_select.capitalize()} Arm")
                if not success: return False
            
            print("\n[INFO] Fold Sequence Completed!")
            return True

        except Exception as e:
            print(f"\n✗ Motion failed: {e}")
            import traceback
            traceback.print_exc()
            return False

def mp_right_fold(robot_controller, grasp_pos, place_pos, lift_height=0.1, reset_joints=None):
    """
    右手折叠动作封装 (Right Arm Fold)
    Grasp -> Lift -> Move -> Place -> Reset
    """
    if reset_joints is None:
        # Load default safe home position
        _, right_home = load_home_config()
        reset_joints = right_home

    planner = MotionPlanner(robot_controller)
    return planner.execute_fold('right', grasp_pos, place_pos, lift_height, reset_joints)

def mp_left_fold(robot_controller, grasp_pos, place_pos, lift_height=0.1, reset_joints=None):
    """
    左手折叠动作封装 (Left Arm Fold)
    Grasp -> Lift -> Move -> Place -> Reset
    """
    if reset_joints is None:
        # Load default safe home position
        left_home, _ = load_home_config()
        reset_joints = left_home

    planner = MotionPlanner(robot_controller)
    return planner.execute_fold('left', grasp_pos, place_pos, lift_height, reset_joints)

def mp_bimanual_fold(robot_controller, left_grasp_pos, right_grasp_pos, 
                     left_place_pos, right_place_pos, lift_height=0.1):
    """
    双臂同时折叠动作 (Bimanual Fold)
    参考 Fold_Tops_HALO_mp.py 中的 mp_fold_bottom_up 逻辑
    
    Args:
        robot_controller: RealRobotController instance
        left_grasp_pos: [x, y, z] 左手抓取位置
        right_grasp_pos: [x, y, z] 右手抓取位置
        left_place_pos: [x, y, z] 左手放置位置
        right_place_pos: [x, y, z] 右手放置位置
        lift_height: 抬起高度 (meters)
        
    Returns:
        bool: True if successful
    """
    try:
        import threading
        
        print("\n[Motion] Executing Bimanual Fold Sequence")
        print("=" * 70)
        
        left_arm = robot_controller.left_arm
        right_arm = robot_controller.right_arm
        
        # Get current orientations to keep them
        left_current = left_arm.get_current_pose()
        right_current = right_arm.get_current_pose()
        
        # Configuration constants
        HAND_OFFSET = 0.12  # End-effector to palm center offset
        SMALL_LIFT_HEIGHT = 0.02  # 2cm - just enough to clear surface after grasp
        FINAL_DROP_HEIGHT = 0.07  # 10cm - height above target position to drop from
        
        # Custom orientations for different phases (roll, pitch, yaw in radians)
        # Phase 1: Grasp orientations
        LEFT_GRASP_ORIENTATION = [1.119000, -1.545000, 2.949000]   # 左手抓取姿态
        RIGHT_GRASP_ORIENTATION = [2.040000, -1.547000, 0.104000]  # 右手抓取姿态
        
        # Phase 4: Place orientations  
        LEFT_PLACE_ORIENTATION = [-0.291000, -1.554000, -1.568000]   # 左手放置姿态 (示例值,可后续调整)
        RIGHT_PLACE_ORIENTATION = [2.524000, -1.543000, -0.620000]  # 右手放置姿态 (示例值,可后续调整)
        
        # ---------------------------------------------------------
        # Phase 1: 双手移动到起始点 (Grasp positions)
        # ---------------------------------------------------------
        print("\n[Phase 1] Preparing to move both hands to grasp positions...")
        
        # Prepare grasp poses with offsets
        left_grasp = np.array(left_grasp_pos)
        left_grasp[1] -= 0.08
        left_grasp[0] -= 0.03  # Left hand offset
        left_grasp[2] = -0.105 + 0.03  # Left hand Z height
        
        right_grasp = np.array(right_grasp_pos)
        right_grasp[1] += 0.08  # Right hand offset
        right_grasp[0] -= 0.07  # Right hand X offset

        right_grasp[2] = -0.105 + 0.03  # Right hand Z height
        

        # Use custom grasp orientations instead of keeping current
        left_grasp_pose = np.concatenate([left_grasp, LEFT_GRASP_ORIENTATION])
        right_grasp_pose = np.concatenate([right_grasp, RIGHT_GRASP_ORIENTATION])
        
        print(f"  → Left target: [{left_grasp[0]:.3f}, {left_grasp[1]:.3f}, {left_grasp[2]:.3f}]")
        print(f"  → Left orientation: [{LEFT_GRASP_ORIENTATION[0]:.3f}, {LEFT_GRASP_ORIENTATION[1]:.3f}, {LEFT_GRASP_ORIENTATION[2]:.3f}]")
        print(f"  → Right target: [{right_grasp[0]:.3f}, {right_grasp[1]:.3f}, {right_grasp[2]:.3f}]")
        print(f"  → Right orientation: [{RIGHT_GRASP_ORIENTATION[0]:.3f}, {RIGHT_GRASP_ORIENTATION[1]:.3f}, {RIGHT_GRASP_ORIENTATION[2]:.3f}]")
        
        # Use stepwise motion for safety
        if not _execute_bimanual_stepwise_motion(
            left_arm, right_arm, 
            left_grasp_pose, right_grasp_pose,
            step_size=0.05, speed=5,
            label="Phase 1 (Move to Grasp)"
        ):
            return False
        
        # ---------------------------------------------------------
        # Phase 2: 双手同时闭合 (Grasp)
        # ---------------------------------------------------------
        if not _confirm_phase_simple("Phase 2: Close Both Hands Simultaneously"): 
            return False
        
        print("\n[Phase 2] Closing both hands simultaneously...")
        # Use threading to close both hands at the same time
        left_close_thread = threading.Thread(target=robot_controller.close_left_hand)
        right_close_thread = threading.Thread(target=robot_controller.close_right_hand)
        
        left_close_thread.start()
        right_close_thread.start()
        
        left_close_thread.join()
        right_close_thread.join()
        
        time.sleep(0.5)
        print("  ✓ Both hands closed simultaneously")
        
        # ---------------------------------------------------------
        # Phase 3: 双手原地抬起一小段距离 (Lift Slightly)
        # ---------------------------------------------------------
        # print("\n[Phase 3] Preparing to lift both hands slightly (just clear the surface)...")
        
        # # Get current poses to ensure we keep exact XY
        # left_current_grasp = left_arm.get_current_pose()
        # right_current_grasp = right_arm.get_current_pose()
        
        # # Create lift poses: keep XY the same, only increase Z by small amount
        # left_lift_pose = np.array(left_current_grasp).copy()
        # left_lift_pose[2] += SMALL_LIFT_HEIGHT  # Only change Z coordinate
        
        # right_lift_pose = np.array(right_current_grasp).copy()
        # right_lift_pose[2] += SMALL_LIFT_HEIGHT  # Only change Z coordinate
        
        # print(f"  → Small lift height: {SMALL_LIFT_HEIGHT*100:.1f}cm (just clear surface)")
        # print(f"  → Left target: [{left_lift_pose[0]:.3f}, {left_lift_pose[1]:.3f}, {left_lift_pose[2]:.3f}]")
        # print(f"  → Right target: [{right_lift_pose[0]:.3f}, {right_lift_pose[1]:.3f}, {right_lift_pose[2]:.3f}]")
        
        # # Use stepwise motion for safety (slow lift)
        # if not _execute_bimanual_stepwise_motion(
        #     left_arm, right_arm,
        #     left_lift_pose, right_lift_pose,
        #     step_size=0.01, speed=5,  # Very small steps for vertical lift (1cm steps)
        #     label="Phase 3 (Lift Slightly)"
        # ):
        #     return False
        
        # ---------------------------------------------------------
        # Phase 4: 移动到放置位置并抬高 (Move to Place and Lift)
        # ---------------------------------------------------------
        print("\n[Phase 4] Preparing to move to place positions while lifting...")
        
        # Prepare place poses with offsets
        left_place = np.array(left_place_pos)
        left_place[1] -= 0.12  # Slight overshoot for folding
        left_place[0] -= 0.02
        # Set final height: target Z + 10cm for dropping
        left_place[2] = FINAL_DROP_HEIGHT  # 10cm above target
        
        right_place = np.array(right_place_pos)
        right_place[1] += 0.12  # Slight overshoot for folding
        right_place[0] -= 0.02
        right_place[2] = FINAL_DROP_HEIGHT  # 10cm above target
        
        # Use custom place orientations instead of keeping current
        left_place_pose = np.concatenate([left_place, LEFT_PLACE_ORIENTATION])
        right_place_pose = np.concatenate([right_place, RIGHT_PLACE_ORIENTATION])
        
        print(f"  → Final drop height: {FINAL_DROP_HEIGHT*100:.0f}cm above target")
        print(f"  → Left target: [{left_place[0]:.3f}, {left_place[1]:.3f}, {left_place[2]:.3f}]")
        print(f"  → Left orientation: [{LEFT_PLACE_ORIENTATION[0]:.3f}, {LEFT_PLACE_ORIENTATION[1]:.3f}, {LEFT_PLACE_ORIENTATION[2]:.3f}]")
        print(f"  → Right target: [{right_place[0]:.3f}, {right_place[1]:.3f}, {right_place[2]:.3f}]")
        print(f"  → Right orientation: [{RIGHT_PLACE_ORIENTATION[0]:.3f}, {RIGHT_PLACE_ORIENTATION[1]:.3f}, {RIGHT_PLACE_ORIENTATION[2]:.3f}]")
        print(f"  → Will move horizontally while gradually lifting")
        
        # Use stepwise motion for safety (moving and lifting simultaneously)
        if not _execute_bimanual_stepwise_motion(
            left_arm, right_arm,
            left_place_pose, right_place_pose,
            step_size=0.05, speed=5,  # Slow and careful movement
            label="Phase 4 (Move & Lift to Drop Point)"
        ):
            return False
        
        # ---------------------------------------------------------
        # Phase 5: 同时松开双手让衣服下落 (Release and Drop)
        # ---------------------------------------------------------
        if not _confirm_phase_simple("Phase 5: Open Both Hands to Drop Garment"): 
            return False
        
        print("\n[Phase 5] Opening both hands to drop garment...")
        print(f"  → Garment will drop from {FINAL_DROP_HEIGHT*100:.0f}cm height to target position")
        
        # Use threading to open both hands at the same time
        left_open_thread = threading.Thread(target=robot_controller.open_left_hand)
        right_open_thread = threading.Thread(target=robot_controller.open_right_hand)
        
        left_open_thread.start()
        right_open_thread.start()
        
        left_open_thread.join()
        right_open_thread.join()
        
        time.sleep(0.5)
        print("  ✓ Both hands opened - garment dropped to target position")
        
        # ---------------------------------------------------------
        # Phase 6: 重置双手 (Reset Both Arms)
        # ---------------------------------------------------------
        if not _confirm_phase_simple("Phase 6: Reset Both Arms to Home"): 
            return False
        
        print("\n[Phase 6] Resetting both arms to home positions...")
        left_home, right_home = load_home_config()
        
        if left_home is not None:
            reset_arm(left_arm, left_home, arm_name="Left Arm")
        
        if right_home is not None:
            reset_arm(right_arm, right_home, arm_name="Right Arm")
        
        print("\n[SUCCESS] Bimanual Fold Sequence Completed!")
        print("=" * 70)
        return True
        
    except Exception as e:
        print(f"\n✗ Bimanual motion failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def _confirm_phase_simple(phase_name):
    """Helper function for phase confirmation"""
    print(f"\n" + "-"*40)
    try:
        ans = robust_input(f"[CONFIRM] Press ENTER to execute {phase_name} (or 'q' to quit): ").strip().lower()
        if ans == 'q':
            print("[INFO] Quit requested.")
            return False
        return True
    except EOFError as e:
        print(f"\n[ERROR] {e}")
        return False

def _wait_for_position_reached(arm, target_pose, tolerance=0.01, timeout=30.0, check_interval=0.1):
    """
    Wait until arm reaches target position
    
    Args:
        arm: ArmControl instance
        target_pose: Target pose [x, y, z, rx, ry, rz]
        tolerance: Position tolerance in meters
        timeout: Maximum wait time in seconds
        check_interval: Time between position checks
        
    Returns:
        bool: True if reached, False if timeout
    """
    import time
    start_time = time.time()
    target_pos = np.array(target_pose[:3])
    
    while (time.time() - start_time) < timeout:
        current_pose = arm.get_current_pose()
        current_pos = np.array(current_pose[:3])
        distance = np.linalg.norm(current_pos - target_pos)
        
        if distance < tolerance:
            return True
        
        time.sleep(check_interval)
    
    # Timeout
    current_pose = arm.get_current_pose()
    current_pos = np.array(current_pose[:3])
    distance = np.linalg.norm(current_pos - target_pos)
    print(f"  [WARNING] Timeout waiting for position. Distance: {distance:.3f}m")
    return False

def _execute_bimanual_stepwise_motion(left_arm, right_arm, left_target_pose, right_target_pose, 
                                       step_size=0.05, speed=5, label="Motion"):
    """
    Execute bimanual stepwise motion with user confirmation at each step
    Both arms move simultaneously in small steps
    Uses quaternion SLERP for smooth orientation interpolation
    
    Args:
        left_arm: Left ArmControl instance
        right_arm: Right ArmControl instance
        left_target_pose: Target pose for left arm [x, y, z, rx, ry, rz]
        right_target_pose: Target pose for right arm [x, y, z, rx, ry, rz]
        step_size: Step size in meters (default: 0.05m = 5cm)
        speed: Movement speed (0-100, default: 10 for safety)
        label: Label for this motion phase
        
    Returns:
        bool: True if successful, False if cancelled
    """
    # Get current poses
    left_current = left_arm.get_current_pose()
    right_current = right_arm.get_current_pose()
    
    left_start_pos = np.array(left_current[:3])
    right_start_pos = np.array(right_current[:3])
    
    left_end_pos = np.array(left_target_pose[:3])
    right_end_pos = np.array(right_target_pose[:3])
    
    # Convert orientations to quaternions
    left_start_euler = np.array(left_current[3:])
    right_start_euler = np.array(right_current[3:])
    left_target_euler = np.array(left_target_pose[3:])
    right_target_euler = np.array(right_target_pose[3:])
    
    left_start_quat = euler_to_quaternion(left_start_euler[0], left_start_euler[1], left_start_euler[2])
    right_start_quat = euler_to_quaternion(right_start_euler[0], right_start_euler[1], right_start_euler[2])
    left_target_quat = euler_to_quaternion(left_target_euler[0], left_target_euler[1], left_target_euler[2])
    right_target_quat = euler_to_quaternion(right_target_euler[0], right_target_euler[1], right_target_euler[2])
    
    # Calculate distances
    left_dist = np.linalg.norm(left_end_pos - left_start_pos)
    right_dist = np.linalg.norm(right_end_pos - right_start_pos)
    max_dist = max(left_dist, right_dist)
    
    if max_dist < 0.001:
        print(f"  [INFO] Already at target ({label})")
        return True
    
    # Calculate number of steps based on max distance
    num_steps = int(np.ceil(max_dist / step_size))
    if num_steps < 1: num_steps = 1
    
    print(f"\n  [BIMANUAL STEPWISE] {label}")
    print(f"  Left distance: {left_dist:.3f}m | Right distance: {right_dist:.3f}m")
    print(f"  Steps: {num_steps} | Step size: ~{step_size*100:.1f}cm | Speed: {speed}")
    print("  Controls: ENTER = Next step, 'q' = Quit")
    
    for i in range(1, num_steps + 1):
        ratio = i / num_steps
        
        # Interpolate positions linearly
        left_interp_pos = left_start_pos + (left_end_pos - left_start_pos) * ratio
        right_interp_pos = right_start_pos + (right_end_pos - right_start_pos) * ratio
        
        # Interpolate orientations using quaternion SLERP
        left_interp_quat = quaternion_slerp(left_start_quat, left_target_quat, ratio)
        right_interp_quat = quaternion_slerp(right_start_quat, right_target_quat, ratio)
        
        left_interp_euler = quaternion_to_euler(left_interp_quat)
        right_interp_euler = quaternion_to_euler(right_interp_quat)
        
        left_interp_pose = np.concatenate([left_interp_pos, left_interp_euler])
        right_interp_pose = np.concatenate([right_interp_pos, right_interp_euler])
        
        # User confirmation
        try:
            user_input = robust_input(
                f"    Step {i}/{num_steps} | "
                f"L:[{left_interp_pos[0]:.3f}, {left_interp_pos[1]:.3f}, {left_interp_pos[2]:.3f}] "
                f"R:[{right_interp_pos[0]:.3f}, {right_interp_pos[1]:.3f}, {right_interp_pos[2]:.3f}] > "
            ).strip().lower()
            
            if user_input == 'q':
                print("  [INFO] Motion cancelled by user")
                return False
        except EOFError as e:
            print(f"\n  [ERROR] {e}")
            return False
        
        # Execute step for both arms simultaneously
        left_arm.robot.Clear_System_Err()
        right_arm.robot.Clear_System_Err()
        
        left_arm.move_pose_Cmd(left_interp_pose.tolist(), speed, block=False)
        right_arm.move_pose_Cmd(right_interp_pose.tolist(), speed, block=False)
        
        # Wait for both arms to reach positions
        # _wait_for_position_reached(left_arm, left_interp_pose, tolerance=0.015, timeout=20.0)
        # _wait_for_position_reached(right_arm, right_interp_pose, tolerance=0.015, timeout=20.0)
    
    print(f"  ✓ {label} Completed")
    return True
