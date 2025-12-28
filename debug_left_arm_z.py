#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug script for Left Arm Z-axis drift
测试左臂在保持 Z 值不变的情况下移动时的 Z 轴漂移问题
"""

import numpy as np
import time
import sys
from real_robot_controller import RealRobotController

def wait_for_user(msg="Press ENTER to continue..."):
    """Wait for user confirmation - AUTO MODE for Agent execution"""
    print(f"\n[AUTO] {msg} (Resuming in 1s...)")
    time.sleep(1)
    # input(f"\n[USER] {msg}")

def debug_z_drift():
    print("\n" + "="*60)
    print("  Left Arm Z-Axis Drift Debug (Stepwise)")
    print("  左臂 Z 轴漂移测试 - 分步执行")
    print("="*60)

    # 1. Initialize Robot
    print("\n[INFO] Initializing robot controller...")
    try:
        robot = RealRobotController()
    except Exception as e:
        print(f"[ERROR] Failed to initialize robot: {e}")
        return

    try:
        # Select Left Arm
        active_arm = robot.left_arm
        print("\n[INFO] Testing LEFT ARM")

        # Get current pose
        current_pose = active_arm.get_current_pose()
        print(f"Current Pose (Left Arm Base Frame): X={current_pose[0]:.4f}, Y={current_pose[1]:.4f}, Z={current_pose[2]:.4f}")

        # Define Target Z
        TARGET_Z = 0.105
        
        # Step 1: Move to Initial Z
        print(f"\n[Step 1] Moving to Initial Z={TARGET_Z} at current X, Y...")
        print(f"Target: [{current_pose[0]:.4f}, {current_pose[1]:.4f}, {TARGET_Z:.4f}]")
        wait_for_user("Press ENTER to move to initial height")
        
        start_pose = np.array(current_pose)
        start_pose[2] = TARGET_Z
        
        robot.left_arm.move_pose(start_pose, speed=3)
        time.sleep(1)
        
        actual_pose_1 = active_arm.get_current_pose()
        print(f"  Result 1: X={actual_pose_1[0]:.4f}, Y={actual_pose_1[1]:.4f}, Z={actual_pose_1[2]:.4f}")
        print(f"  Z Error: {actual_pose_1[2] - TARGET_Z:.4f}")

        # Step 2: Move Y+10cm
        print("\n[Step 2] Moving to Point B (Y + 0.1m) with same Z...")
        target_pose = np.array(actual_pose_1)
        target_pose[1] += 0.10  # Move Y by 10cm
        target_pose[2] = TARGET_Z # Ensure Z is explicitly set
        
        print(f"  Final Target: X={target_pose[0]:.4f}, Y={target_pose[1]:.4f}, Z={target_pose[2]:.4f}")
        
        wait_for_user("Press ENTER to start stepwise movement to Point B")

        # Stepwise movement logic similar to motion_primitives
        start_p = actual_pose_1
        end_p = target_pose
        steps = 10
        
        for i in range(1, steps + 1):
            ratio = i / steps
            interp_pos = start_p[:3] + (end_p[:3] - start_p[:3]) * ratio
            # Force Z to be exactly TARGET_Z regardless of start point error
            interp_pos[2] = TARGET_Z
            interp_pose = np.concatenate([interp_pos, start_p[3:]])
            
            print(f"\n  Step {i}/{steps}")
            print(f"    Target: X={interp_pose[0]:.4f}, Y={interp_pose[1]:.4f}, Z={interp_pose[2]:.4f}")
            
            wait_for_user(f"Press ENTER to execute Step {i}")
            
            robot.left_arm.move_pose(interp_pose, speed=3)
            time.sleep(0.5)
            
            curr = active_arm.get_current_pose()
            print(f"    Actual: X={curr[0]:.4f}, Y={curr[1]:.4f}, Z={curr[2]:.4f}")
            print(f"    Z Diff (Actual - Target): {curr[2] - interp_pose[2]:.4f}")
            print(f"    Z Drift (Actual - Start): {curr[2] - start_p[2]:.4f}")

    except Exception as e:
        print(f"[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[INFO] Closing robot...")
        robot.close()

if __name__ == "__main__":
    debug_z_drift()
