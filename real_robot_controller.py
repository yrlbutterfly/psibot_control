#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real Robot Controller
真机机器人控制器 - 封装双臂、双手、相机
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
from robot_libs.ruiyan_hand_module import Hand
from robot_libs.realsense_image_module import RealSenseImage
import time
import os


class RealRobotController:
    """Real robot controller - integrates dual arms, dual hands, and camera"""
    
    def __init__(self, camera_sn="046322250624"):
        """
        Initialize all hardware
        初始化所有硬件
        
        Args:
            camera_sn: RealSense camera serial number
        """
        print("\n" + "="*60)
        print("  Initializing Real Robot Controller")
        print("  初始化真机控制器")
        print("="*60 + "\n")
        
        # Connect to arms (连接机械臂)
        print("[1/5] Connecting to arms...")
        self.left_arm = ArmControl(ip='192.168.100.100')
        print("  ✓ Left arm connected")
        
        self.right_arm = ArmControl(ip='192.168.100.101')
        print("  ✓ Right arm connected")
        
        # Connect to hands (连接灵巧手) - Use correct ports!
        print("\n[2/5] Connecting to hands...")
        self.left_hand = Hand(port='/dev/ttyACM2')  # Left hand
        print("  ✓ Left hand connected")
        time.sleep(0.5)  # Wait before connecting second hand
        
        self.right_hand = Hand(port='/dev/ttyACM3')  # Right hand
        print("  ✓ Right hand connected")
        time.sleep(0.5)
        
        # Connect to camera (连接相机)
        print("\n[3/5] Connecting to camera...")
        self.camera = RealSenseImage(SN_number=camera_sn)
        self.camera_intrinsics = self.camera.o3d_intrinsics.intrinsic_matrix
        print("  ✓ Camera connected")
        
        # Load calibration matrices (加载标定矩阵)
        print("\n[4/5] Loading calibration matrices...")
        calib_left_path = 'calibration_results/camera_calibration_left_arm_20251222-224450.npz'
        calib_right_path = 'calibration_results/camera_calibration_right_arm_20251222-222131.npz'
        
        if not os.path.exists(calib_left_path) or not os.path.exists(calib_right_path):
            raise FileNotFoundError("Calibration files not found!")
        
        calib_left = np.load(calib_left_path)
        calib_right = np.load(calib_right_path)
        
        self.T_cam2base_left = calib_left['T_cam2base']
        self.T_cam2base_right = calib_right['T_cam2base']
        
        # Use right arm calibration as default
        self.T_cam2base = self.T_cam2base_right
        
        print("  ✓ Calibration loaded")
        
        # Clear errors (清除错误)
        print("\n[5/5] Clearing system errors...")
        self.left_arm.robot.Clear_System_Err()
        self.right_arm.robot.Clear_System_Err()
        print("  ✓ Errors cleared")
        
        print("\n" + "="*60)
        print("  ✓ Robot Controller Ready!")
        print("="*60 + "\n")
    
    # ==================== Hand Control ====================
    
    def open_left_hand(self, duration=3.0):
        """Open left hand (张开左手)"""
        self.left_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(duration)
    
    def close_left_hand(self, duration=3.0):
        """Close left hand (闭合左手)"""
        self.left_hand.set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(duration)
    
    def open_right_hand(self, duration=3.0):
        """Open right hand (张开右手)"""
        self.right_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(duration)
    
    def close_right_hand(self, duration=3.0):
        """Close right hand (闭合右手)"""
        self.right_hand.set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(duration)
    
    def set_both_hands(self, left_state, right_state, duration=3.0):
        """
        Set both hands state
        设置双手状态
        
        Args:
            left_state: "open", "close", or None (keep current)
            right_state: "open", "close", or None (keep current)
            duration: Movement duration in seconds
        """
        # Send left hand command
        if left_state == "open":
            self.left_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        elif left_state == "close":
            self.left_hand.set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        time.sleep(0.1)  # Small delay between hands to avoid serial conflict
        
        # Send right hand command
        if right_state == "open":
            self.right_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        elif right_state == "close":
            self.right_hand.set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Wait for both hands to reach target
        time.sleep(duration)
    
    # ==================== Arm Control ====================
    
    def get_left_arm_pose(self):
        """Get left arm current pose (获取左臂当前位姿)"""
        return self.left_arm.get_current_pose()
    
    def get_right_arm_pose(self):
        """Get right arm current pose (获取右臂当前位姿)"""
        return self.right_arm.get_current_pose()
    
    def move_left_arm_to_pose(self, target_pose, speed=20, block=True):
        """
        Move left arm to target pose
        移动左臂到目标位姿
        
        Args:
            target_pose: [x, y, z, rx, ry, rz] in meters and radians
            speed: Movement speed (0-100)
            block: Wait until movement completes
        """
        return self.left_arm.move_pose(target_pose, speed=speed, block=block)
    
    def move_right_arm_to_pose(self, target_pose, speed=20, block=True):
        """Move right arm to target pose (移动右臂到目标位姿)"""
        return self.right_arm.move_pose(target_pose, speed=speed, block=block)
    
    def move_left_arm_to_position(self, target_pos, speed=20, block=True):
        """
        Move left arm to target position (keep current orientation)
        移动左臂到目标位置（保持当前姿态）
        
        Args:
            target_pos: [x, y, z] in meters
        """
        current_pose = self.left_arm.get_current_pose()
        target_pose = np.array([
            target_pos[0], target_pos[1], target_pos[2],
            current_pose[3], current_pose[4], current_pose[5]
        ])
        return self.left_arm.move_pose(target_pose, speed=speed, block=block)
    
    def move_right_arm_to_position(self, target_pos, speed=20, block=True):
        """Move right arm to target position (移动右臂到目标位置)"""
        current_pose = self.right_arm.get_current_pose()
        target_pose = np.array([
            target_pos[0], target_pos[1], target_pos[2],
            current_pose[3], current_pose[4], current_pose[5]
        ])
        return self.right_arm.move_pose(target_pose, speed=speed, block=block)
    
    # ==================== Camera Control ====================
    
    def capture_scene(self):
        """
        Capture RGB and depth images
        拍摄场景（RGB + 深度）
        
        Returns:
            color_img: RGB image (H x W x 3)
            depth_img: Depth image (H x W) in mm
        """
        color, depth = self.camera.capture_frame()
        return color, depth
    
    def capture_rgb(self):
        """Capture RGB image only (只拍摄RGB)"""
        return self.camera.capture_rgb_frame()
    
    # ==================== Cleanup ====================
    
    def close(self):
        """Close all connections (关闭所有连接)"""
        print("\n[Cleanup] Closing connections...")
        
        try:
            self.left_arm.close()
            print("  ✓ Left arm closed")
        except:
            pass
        
        try:
            self.right_arm.close()
            print("  ✓ Right arm closed")
        except:
            pass
        
        try:
            self.left_hand.close()
            print("  ✓ Left hand closed")
        except:
            pass
        
        try:
            self.right_hand.close()
            print("  ✓ Right hand closed")
        except:
            pass
        
        try:
            self.camera.close()
            print("  ✓ Camera closed")
        except:
            pass
        
        print("✓ All connections closed\n")


if __name__ == "__main__":
    # Simple test
    print("\n" + "="*60)
    print("  Real Robot Controller Test")
    print("="*60)
    print("\n⚠️  Safety Reminder:")
    print("  - Hands will move slowly (5 seconds per action)")
    print("  - Press Ctrl+C anytime to stop")
    print("  - Ensure hands are clear of obstacles")
    print("="*60 + "\n")
    
    confirm = input("Ready to start test? (yes/no): ").strip().lower()
    if confirm != 'yes':
        print("Test cancelled.")
        exit(0)
    
    try:
        robot = RealRobotController()
        
        print("\n[Test 1] Testing hands...")
        print("  Opening both hands (5 seconds)...")
        robot.set_both_hands("open", "open", duration=5.0)
        
        print("  Closing both hands (5 seconds)...")
        robot.set_both_hands("close", "close", duration=5.0)
        
        print("  Opening both hands again (5 seconds)...")
        robot.set_both_hands("open", "open", duration=5.0)
        print("  ✓ Hand test passed")
        
        print("\n[Test 2] Testing camera...")
        color, depth = robot.capture_scene()
        print(f"  ✓ Image captured: {color.shape}")
        
        print("\n[Test 3] Reading arm poses...")
        left_pose = robot.get_left_arm_pose()
        right_pose = robot.get_right_arm_pose()
        print(f"  Left arm:  X={left_pose[0]:.3f}, Y={left_pose[1]:.3f}, Z={left_pose[2]:.3f}")
        print(f"  Right arm: X={right_pose[0]:.3f}, Y={right_pose[1]:.3f}, Z={right_pose[2]:.3f}")
        
        print("\n✓ All tests passed!")
        
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'robot' in locals():
            robot.close()
