#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Get Right Arm Current Pose
获取右手机械臂当前姿态 - 用于找到自定义姿态值
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
import time

def main():
    print("\n" + "="*60)
    print("  Right Arm Pose Reader")
    print("  右手姿态读取工具")
    print("="*60 + "\n")
    
    # Connect to right arm
    print("[INFO] Connecting to right arm (192.168.100.101)...")
    try:
        right_arm = ArmControl(ip='192.168.100.101')
        print("  ✓ Right arm connected\n")
    except Exception as e:
        print(f"  ✗ Connection failed: {e}")
        return
    
    print("Instructions:")
    print("  1. Move the right arm to your desired position manually")
    print("  2. Press ENTER to read current pose")
    print("  3. Type 'save' to save the orientation values")
    print("  4. Type 'q' to quit")
    print("-"*60 + "\n")
    
    saved_orientations = []
    
    try:
        while True:
            user_input = input("Press ENTER to read pose (or 'save'/'q'): ").strip().lower()
            
            if user_input == 'q':
                print("\n[INFO] Exiting...")
                break
            
            # Get current pose
            current_pose = right_arm.get_current_pose()
            
            if current_pose is None or len(current_pose) < 6:
                print("  ✗ Failed to get pose")
                continue
            
            # Extract position and orientation
            position = current_pose[:3]  # [x, y, z]
            orientation = current_pose[3:]  # [roll, pitch, yaw]
            
            print("\n" + "="*60)
            print("Current Right Arm Pose:")
            print("-"*60)
            print(f"  Position (XYZ):  [{position[0]:+.6f}, {position[1]:+.6f}, {position[2]:+.6f}]")
            print(f"  Orientation (RPY): [{orientation[0]:+.6f}, {orientation[1]:+.6f}, {orientation[2]:+.6f}]")
            print("-"*60)
            print("\nFor motion_primitives.py configuration:")
            print(f"  self.RIGHT_HAND_ORIENTATION = [{orientation[0]:.6f}, {orientation[1]:.6f}, {orientation[2]:.6f}]")
            print("="*60 + "\n")
            
            # If user wants to save
            if user_input == 'save':
                saved_orientations.append(orientation.copy())
                print(f"✓ Orientation saved! (Total saved: {len(saved_orientations)})")
                
    except KeyboardInterrupt:
        print("\n\n[INFO] Interrupted by user")
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Show saved orientations
        if saved_orientations:
            print("\n" + "="*60)
            print("Saved Orientations:")
            print("-"*60)
            for i, ori in enumerate(saved_orientations, 1):
                print(f"  {i}. [{ori[0]:.6f}, {ori[1]:.6f}, {ori[2]:.6f}]")
            print("="*60 + "\n")
        
        # Close connection
        print("[INFO] Closing connection...")
        right_arm.close()
        print("  ✓ Done\n")

if __name__ == "__main__":
    main()

