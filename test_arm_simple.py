#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple test script for RealMan arm
Test basic connection and joint reading
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
import time

def test_single_arm(ip_address, arm_name="Arm"):
    """
    Test single arm connection and basic operations
    
    Args:
        ip_address: IP address of the arm controller
        arm_name: Name identifier for logging
    """
    print(f"\n{'='*50}")
    print(f"Testing {arm_name} at IP: {ip_address}")
    print(f"{'='*50}\n")
    
    try:
        # Step 1: Initialize connection
        print(f"[1/4] Initializing connection to {arm_name}...")
        arm = ArmControl(ip=ip_address)
        print(f"✓ Connection established successfully!")
        time.sleep(0.5)
        
        # Step 2: Clear any errors
        print(f"\n[2/4] Clearing system errors...")
        arm.robot.Clear_System_Err()
        print(f"✓ System errors cleared!")
        time.sleep(0.5)
        
        # Step 3: Read current joint angles
        print(f"\n[3/4] Reading current joint angles...")
        current_joint = arm.get_current_joint()
        print(f"✓ Current joint angles (radians):")
        print(f"  [{current_joint[0]:.6f}, {current_joint[1]:.6f}, {current_joint[2]:.6f}, "
              f"{current_joint[3]:.6f}, {current_joint[4]:.6f}, {current_joint[5]:.6f}, {current_joint[6]:.6f}]")
        
        # Convert to degrees for easier reading
        current_joint_deg = np.rad2deg(current_joint)
        print(f"\n  Current joint angles (degrees):")
        print(f"  [{current_joint_deg[0]:.2f}°, {current_joint_deg[1]:.2f}°, {current_joint_deg[2]:.2f}°, "
              f"{current_joint_deg[3]:.2f}°, {current_joint_deg[4]:.2f}°, {current_joint_deg[5]:.2f}°, {current_joint_deg[6]:.2f}°]")
        
        # Step 4: Read current pose
        print(f"\n[4/4] Reading current end-effector pose...")
        current_pose = arm.get_current_pose()
        print(f"✓ Current pose [x, y, z, rx, ry, rz]:")
        print(f"  {current_pose}")
        
        print(f"\n{'='*50}")
        print(f"✓ {arm_name} Test PASSED!")
        print(f"{'='*50}\n")
        
        # Close connection
        arm.close()
        return True
        
    except Exception as e:
        print(f"\n{'='*50}")
        print(f"✗ {arm_name} Test FAILED!")
        print(f"Error: {str(e)}")
        print(f"{'='*50}\n")
        return False


def test_both_arms():
    """Test both left and right arms"""
    print("\n" + "="*60)
    print("  RealMan RM75 Dual-Arm System Test")
    print("="*60)
    
    # Test configuration
    left_ip = '192.168.100.100'
    right_ip = '192.168.100.101'
    
    # Test left arm
    left_success = test_single_arm(left_ip, "Left Arm")
    
    # Wait a bit between tests
    time.sleep(1)
    
    # Test right arm
    right_success = test_single_arm(right_ip, "Right Arm")
    
    # Summary
    print("\n" + "="*60)
    print("  Test Summary")
    print("="*60)
    print(f"Left Arm:  {'✓ PASS' if left_success else '✗ FAIL'}")
    print(f"Right Arm: {'✓ PASS' if right_success else '✗ FAIL'}")
    print("="*60 + "\n")
    

def test_single_arm_only():
    """Test only one arm - useful for initial setup"""
    print("\n" + "="*60)
    print("  Single Arm Test Mode")
    print("="*60)
    
    print("\nWhich arm do you want to test?")
    print("1. Left Arm  (192.168.100.100)")
    print("2. Right Arm (192.168.100.101)")
    print("3. Custom IP")
    
    choice = input("\nEnter your choice (1/2/3): ").strip()
    
    if choice == '1':
        test_single_arm('192.168.100.100', "Left Arm")
    elif choice == '2':
        test_single_arm('192.168.100.101', "Right Arm")
    elif choice == '3':
        custom_ip = input("Enter custom IP address: ").strip()
        test_single_arm(custom_ip, "Custom Arm")
    else:
        print("Invalid choice!")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("  RealMan Arm Test Utility")
    print("="*60)
    print("\nSelect test mode:")
    print("1. Test both arms (default)")
    print("2. Test single arm only")
    
    mode = input("\nEnter your choice (1/2, press Enter for default): ").strip()
    
    if mode == '2':
        test_single_arm_only()
    else:
        test_both_arms()
    
    print("\n✓ Test completed!\n")

