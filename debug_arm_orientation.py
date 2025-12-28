#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug Arm Orientation
调试机械臂姿态获取 - 增强版
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
import time
import sys

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles to rotation matrix"""
    # Rotation around X-axis (Roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation around Y-axis (Pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation around Z-axis (Yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: R = R_z * R_y * R_x
    R = R_z @ R_y @ R_x
    return R

def print_orientation_info(arm, arm_name, target_orientation=None):
    """Print detailed orientation information"""
    
    # Get current pose
    current_pose = arm.get_current_pose()
    if current_pose is None or len(current_pose) < 6:
        print(f"  ✗ Failed to get pose for {arm_name}")
        return None
    
    # Get current joint angles
    current_joint = arm.get_current_joint()
    
    # Extract position and orientation
    position = np.array(current_pose[:3])
    orientation = np.array(current_pose[3:])  # [roll, pitch, yaw] in radians
    
    # Convert to degrees for easier reading
    orientation_deg = np.rad2deg(orientation)
    
    # Calculate rotation matrix
    R = euler_to_rotation_matrix(orientation[0], orientation[1], orientation[2])
    
    # Get direction vectors from rotation matrix
    x_axis = R[:, 0]  # X-axis direction in base frame
    y_axis = R[:, 1]  # Y-axis direction in base frame
    z_axis = R[:, 2]  # Z-axis direction in base frame (pointing direction)
    
    print("\n" + "="*70)
    print(f"  {arm_name} - Detailed Orientation Analysis")
    print("="*70)
    
    # Position
    print("\n1. Position (Base Frame):")
    print(f"   X: {position[0]:+.6f} m")
    print(f"   Y: {position[1]:+.6f} m")
    print(f"   Z: {position[2]:+.6f} m")
    
    # Orientation in radians
    print("\n2. Orientation (Euler Angles - Radians):")
    print(f"   Roll  (Rx): {orientation[0]:+.6f} rad")
    print(f"   Pitch (Ry): {orientation[1]:+.6f} rad")
    print(f"   Yaw   (Rz): {orientation[2]:+.6f} rad")
    
    # Orientation in degrees
    print("\n3. Orientation (Euler Angles - Degrees):")
    print(f"   Roll  (Rx): {orientation_deg[0]:+.2f}°")
    print(f"   Pitch (Ry): {orientation_deg[1]:+.2f}°")
    print(f"   Yaw   (Rz): {orientation_deg[2]:+.2f}°")
    
    # Rotation matrix
    print("\n4. Rotation Matrix (End-Effector Frame in Base Frame):")
    print(f"   [{R[0,0]:+.4f}  {R[0,1]:+.4f}  {R[0,2]:+.4f}]")
    print(f"   [{R[1,0]:+.4f}  {R[1,1]:+.4f}  {R[1,2]:+.4f}]")
    print(f"   [{R[2,0]:+.4f}  {R[2,1]:+.4f}  {R[2,2]:+.4f}]")
    
    # Direction vectors
    print("\n5. End-Effector Axis Directions (in Base Frame):")
    print(f"   X-axis (Red):   [{x_axis[0]:+.4f}, {x_axis[1]:+.4f}, {x_axis[2]:+.4f}]")
    print(f"   Y-axis (Green): [{y_axis[0]:+.4f}, {y_axis[1]:+.4f}, {y_axis[2]:+.4f}]")
    print(f"   Z-axis (Blue):  [{z_axis[0]:+.4f}, {z_axis[1]:+.4f}, {z_axis[2]:+.4f}] (Approach)")
    
    # Joint angles
    if current_joint is not None:
        print("\n6. Joint Angles (Degrees):")
        joint_deg = np.rad2deg(current_joint)
        for i, angle in enumerate(joint_deg, 1):
            print(f"   Joint {i}: {angle:+.2f}°")
    
    # Configuration for motion_primitives.py
    print("\n7. Configuration for motion_primitives.py:")
    print(f"   self.{arm_name.upper()}_HAND_ORIENTATION = [{orientation[0]:.6f}, {orientation[1]:.6f}, {orientation[2]:.6f}]")
    
    # If target orientation is provided, show difference
    if target_orientation is not None:
        print("\n8. Comparison with Target Orientation:")
        target_ori = np.array(target_orientation)
        diff = orientation - target_ori
        diff_deg = np.rad2deg(diff)
        
        print(f"   Target:  [{target_ori[0]:+.6f}, {target_ori[1]:+.6f}, {target_ori[2]:+.6f}]")
        print(f"   Current: [{orientation[0]:+.6f}, {orientation[1]:+.6f}, {orientation[2]:+.6f}]")
        print(f"   Diff:    [{diff[0]:+.6f}, {diff[1]:+.6f}, {diff[2]:+.6f}] rad")
        print(f"   Diff:    [{diff_deg[0]:+.2f}°, {diff_deg[1]:+.2f}°, {diff_deg[2]:+.2f}°]")
    
    print("="*70 + "\n")
    
    return orientation

def main():
    print("\n" + "="*70)
    print("  ARM ORIENTATION DEBUG TOOL")
    print("  机械臂姿态调试工具（增强版）")
    print("="*70 + "\n")
    
    # Ask which arm to debug
    arm_choice = input("Which arm to debug? (left/right/both) [right]: ").strip().lower()
    if not arm_choice:
        arm_choice = 'right'
    
    arms = {}
    
    try:
        # Connect to arms
        if arm_choice in ['right', 'both']:
            print("[INFO] Connecting to right arm (192.168.100.101)...")
            arms['right'] = ArmControl(ip='192.168.100.101')
            print("  ✓ Right arm connected")
        
        if arm_choice in ['left', 'both']:
            print("[INFO] Connecting to left arm (192.168.100.100)...")
            arms['left'] = ArmControl(ip='192.168.100.100')
            print("  ✓ Left arm connected")
        
        if not arms:
            print("  ✗ No arms connected")
            return
        
        # Ask for comparison mode
        compare_mode = input("\nCompare with target orientation? (y/n) [n]: ").strip().lower()
        target_orientations = {}
        
        if compare_mode == 'y':
            for arm_name in arms.keys():
                print(f"\nEnter target orientation for {arm_name} arm (roll, pitch, yaw in radians):")
                print("  (Press ENTER to skip)")
                target_input = input(f"  {arm_name.capitalize()}: ").strip()
                if target_input:
                    try:
                        values = [float(x.strip()) for x in target_input.replace('[', '').replace(']', '').split(',')]
                        if len(values) == 3:
                            target_orientations[arm_name] = values
                    except:
                        print(f"  Invalid input, skipping {arm_name}")
        
        # Default targets from motion_primitives.py
        default_targets = {
            'left': [-2.676000, -1.347000, 0.429000],
            'right': [-3.078000, -1.306000, -1.058000]
        }
        
        for arm_name in arms.keys():
            if arm_name not in target_orientations and arm_name in default_targets:
                target_orientations[arm_name] = default_targets[arm_name]
        
        print("\n" + "-"*70)
        print("Instructions:")
        print("  - Press ENTER to refresh/read current pose")
        print("  - Type 'continuous' to enable continuous monitoring (1Hz)")
        print("  - Type 'q' to quit")
        print("-"*70)
        
        continuous = False
        
        while True:
            if not continuous:
                user_input = input("\nPress ENTER to read (or 'continuous'/'q'): ").strip().lower()
                
                if user_input == 'q':
                    print("\n[INFO] Exiting...")
                    break
                elif user_input == 'continuous':
                    continuous = True
                    print("\n[INFO] Continuous monitoring enabled. Press Ctrl+C to stop.")
                    time.sleep(1)
            
            # Read and display for each arm
            for arm_name, arm in arms.items():
                target = target_orientations.get(arm_name, None)
                print_orientation_info(arm, arm_name.capitalize(), target)
            
            if continuous:
                time.sleep(1)  # 1Hz update rate
            
    except KeyboardInterrupt:
        print("\n\n[INFO] Interrupted by user")
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Close connections
        print("\n[INFO] Closing connections...")
        for arm in arms.values():
            arm.close()
        print("  ✓ Done\n")

if __name__ == "__main__":
    main()

