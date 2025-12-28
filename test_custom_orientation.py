#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Custom Orientation
测试自定义姿态 - 验证motion_primitives.py中的姿态设置是否正确
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
import time

# Custom orientations from motion_primitives.py
LEFT_HAND_ORIENTATION = [-2.676000, -1.347000, 0.429000]
RIGHT_HAND_ORIENTATION = [-3.078000, -1.306000, -1.058000]

def test_orientation(arm, arm_name, target_orientation, test_position=None):
    """
    Test moving to a specific orientation
    
    Args:
        arm: ArmControl instance
        arm_name: 'Left' or 'Right'
        target_orientation: [roll, pitch, yaw] in radians
        test_position: [x, y, z] optional test position, if None use current position
    """
    print("\n" + "="*70)
    print(f"  Testing {arm_name} Arm Orientation")
    print("="*70)
    
    # Get current pose
    current_pose = arm.get_current_pose()
    if current_pose is None or len(current_pose) < 6:
        print(f"  ✗ Failed to get current pose")
        return False
    
    print(f"\nCurrent Pose:")
    print(f"  Position:    [{current_pose[0]:.4f}, {current_pose[1]:.4f}, {current_pose[2]:.4f}]")
    print(f"  Orientation: [{current_pose[3]:.4f}, {current_pose[4]:.4f}, {current_pose[5]:.4f}]")
    
    # Prepare target pose
    if test_position is None:
        # Keep current position, only change orientation
        target_pose = [
            current_pose[0], current_pose[1], current_pose[2],
            target_orientation[0], target_orientation[1], target_orientation[2]
        ]
    else:
        # Use specified position
        target_pose = [
            test_position[0], test_position[1], test_position[2],
            target_orientation[0], target_orientation[1], target_orientation[2]
        ]
    
    print(f"\nTarget Pose:")
    print(f"  Position:    [{target_pose[0]:.4f}, {target_pose[1]:.4f}, {target_pose[2]:.4f}]")
    print(f"  Orientation: [{target_pose[3]:.4f}, {target_pose[4]:.4f}, {target_pose[5]:.4f}]")
    
    # Calculate differences
    pos_diff = np.linalg.norm(np.array(current_pose[:3]) - np.array(target_pose[:3]))
    ori_diff = np.linalg.norm(np.array(current_pose[3:]) - np.array(target_pose[3:]))
    ori_diff_deg = np.rad2deg(ori_diff)
    
    print(f"\nDifferences:")
    print(f"  Position:    {pos_diff:.4f} m")
    print(f"  Orientation: {ori_diff:.4f} rad ({ori_diff_deg:.2f}°)")
    
    # Ask for confirmation
    print("\n" + "-"*70)
    response = input(f"Execute move to target pose? (y/n) [n]: ").strip().lower()
    
    if response != 'y':
        print("  [INFO] Move cancelled")
        return False
    
    # Execute move
    print("\n[INFO] Executing move...")
    try:
        arm.robot.Clear_System_Err()
        arm.move_pose_Cmd(target_pose, speed=5)
        
        # Wait for move to complete
        print("[INFO] Waiting for move to complete (5 seconds)...")
        time.sleep(5)
        
        # Check final pose
        final_pose = arm.get_current_pose()
        if final_pose is not None:
            print(f"\nFinal Pose:")
            print(f"  Position:    [{final_pose[0]:.4f}, {final_pose[1]:.4f}, {final_pose[2]:.4f}]")
            print(f"  Orientation: [{final_pose[3]:.4f}, {final_pose[4]:.4f}, {final_pose[5]:.4f}]")
            
            # Calculate error
            pos_error = np.linalg.norm(np.array(final_pose[:3]) - np.array(target_pose[:3]))
            ori_error = np.linalg.norm(np.array(final_pose[3:]) - np.array(target_pose[3:]))
            ori_error_deg = np.rad2deg(ori_error)
            
            print(f"\nReached Target with Error:")
            print(f"  Position:    {pos_error:.6f} m")
            print(f"  Orientation: {ori_error:.6f} rad ({ori_error_deg:.4f}°)")
            
            if ori_error_deg > 5.0:
                print(f"\n  ⚠️  WARNING: Orientation error is large ({ori_error_deg:.2f}°)")
                print(f"      The target orientation might be unreachable or singular!")
            elif ori_error_deg < 0.5:
                print(f"\n  ✓ Orientation reached successfully!")
            else:
                print(f"\n  ✓ Orientation reached with acceptable error")
        
        return True
        
    except Exception as e:
        print(f"\n  ✗ Move failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("\n" + "="*70)
    print("  CUSTOM ORIENTATION TEST TOOL")
    print("  自定义姿态测试工具")
    print("="*70 + "\n")
    
    print("This tool tests the orientations defined in motion_primitives.py")
    print("by commanding the arm to move to that orientation.\n")
    
    # Ask which arm
    arm_choice = input("Which arm to test? (left/right) [right]: ").strip().lower()
    if not arm_choice:
        arm_choice = 'right'
    
    # Connect to arm
    print(f"\n[INFO] Connecting to {arm_choice} arm...")
    try:
        if arm_choice == 'right':
            arm = ArmControl(ip='192.168.100.101')
            target_orientation = RIGHT_HAND_ORIENTATION
        else:
            arm = ArmControl(ip='192.168.100.100')
            target_orientation = LEFT_HAND_ORIENTATION
        
        print(f"  ✓ {arm_choice.capitalize()} arm connected")
        
        # Test orientation
        test_orientation(arm, arm_choice.capitalize(), target_orientation)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n[INFO] Closing connection...")
        if 'arm' in locals():
            arm.close()
        print("  ✓ Done\n")

if __name__ == "__main__":
    main()

