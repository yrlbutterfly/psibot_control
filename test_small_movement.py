#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Safe Small Movement Test Script
安全的小幅度运动测试脚本

This script is designed for first-time users to safely test arm movement
本脚本专为首次使用者设计，用于安全测试机械臂运动

Features:
- Small movements only (5-10 degrees)
- Slow speed
- Single joint testing
- Clear prompts and confirmations
- Emergency stop instructions
"""

import numpy as np
from robot_libs.realman_arm_module import ArmControl
import time

class SafeArmTest:
    """
    Safe arm movement test class
    安全机械臂运动测试类
    """
    
    def __init__(self, ip_address, arm_name="Arm"):
        """
        Initialize arm connection
        初始化机械臂连接
        
        Args:
            ip_address: IP address of the arm controller (机械臂控制器IP地址)
            arm_name: Name for display (显示名称)
        """
        print(f"\n{'='*60}")
        print(f"  Initializing {arm_name}")
        print(f"{'='*60}\n")
        
        # Connect to arm (连接机械臂)
        print(f"[1/3] Connecting to {arm_name} at {ip_address}...")
        self.arm = ArmControl(ip=ip_address)
        self.arm_name = arm_name
        print(f"✓ Connected successfully!")
        
        # Clear any errors (清除错误)
        print(f"\n[2/3] Clearing system errors...")
        self.arm.robot.Clear_System_Err()
        print(f"✓ System errors cleared!")
        
        # Read and save current position (读取并保存当前位置)
        print(f"\n[3/3] Reading current position...")
        self.initial_joint = self.arm.get_current_joint()
        self.initial_joint_deg = np.rad2deg(self.initial_joint)
        
        print(f"✓ Current joint angles (degrees):")
        for i, angle in enumerate(self.initial_joint_deg, 1):
            print(f"    Joint {i}: {angle:7.2f}°")
        
        # Joint limits in radians (关节限位，弧度制)
        self.dof_lower_limits = np.array([-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28])
        self.dof_upper_limits = np.array([3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28])
        
        print(f"\n{'='*60}")
        print(f"  {arm_name} Ready!")
        print(f"{'='*60}\n")
    
    def move_single_joint(self, joint_index, offset_degrees, speed=5):
        """
        Move a single joint by a small offset
        移动单个关节一个小的偏移量
        
        Args:
            joint_index: Joint number (1-7) (关节编号 1-7)
            offset_degrees: Movement in degrees, positive or negative (移动角度，正负)
            speed: Movement speed (0-100), lower is slower (运动速度，越低越慢)
        """
        # Validate joint index (验证关节编号)
        if joint_index < 1 or joint_index > 7:
            print(f"✗ Error: Joint index must be 1-7, got {joint_index}")
            return False
        
        # Convert to 0-indexed (转换为0索引)
        idx = joint_index - 1
        
        print(f"\n{'='*60}")
        print(f"  Testing Joint {joint_index} Movement")
        print(f"{'='*60}\n")
        
        # Calculate target position (计算目标位置)
        target_joint = self.initial_joint.copy()
        offset_radians = np.deg2rad(offset_degrees)
        target_joint[idx] += offset_radians
        
        # Safety check: ensure within limits (安全检查：确保在限位内)
        if target_joint[idx] < self.dof_lower_limits[idx]:
            print(f"✗ Error: Target position {np.rad2deg(target_joint[idx]):.2f}° "
                  f"below lower limit {np.rad2deg(self.dof_lower_limits[idx]):.2f}°")
            return False
        
        if target_joint[idx] > self.dof_upper_limits[idx]:
            print(f"✗ Error: Target position {np.rad2deg(target_joint[idx]):.2f}° "
                  f"above upper limit {np.rad2deg(self.dof_upper_limits[idx]):.2f}°")
            return False
        
        # Display movement plan (显示运动计划)
        print(f"Movement Plan (运动计划):")
        print(f"  Joint {joint_index}:")
        print(f"    Current position : {self.initial_joint_deg[idx]:7.2f}°")
        print(f"    Target position  : {np.rad2deg(target_joint[idx]):7.2f}°")
        print(f"    Movement         : {offset_degrees:+7.2f}°")
        print(f"    Speed            : {speed}/100")
        
        # Confirmation prompt (确认提示)
        print(f"\n⚠️  IMPORTANT SAFETY CHECKS:")
        print(f"  1. Ensure workspace is clear (确保工作空间无障碍物)")
        print(f"  2. Be ready to press Emergency Stop button (准备好按急停按钮)")
        print(f"  3. This is a small, slow movement (这是一个小幅度、慢速运动)")
        
        confirm = input(f"\nType 'yes' to proceed, anything else to cancel: ").strip().lower()
        
        if confirm != 'yes':
            print(f"✗ Movement cancelled by user")
            return False
        
        # Clear errors before movement (运动前清除错误)
        self.arm.robot.Clear_System_Err()
        
        # Execute movement (执行运动)
        print(f"\n🤖 Starting movement...")
        print(f"   (Movement will take approximately {2 + (10-speed)}s)")
        
        try:
            # Move to target position (移动到目标位置)
            # block=True means wait until movement completes (阻塞模式，等待运动完成)
            ret = self.arm.move_joint(target_joint, speed=speed, block=True)
            
            if ret == 0:  # Success (成功)
                print(f"✓ Movement completed successfully!")
                
                # Read actual final position (读取实际最终位置)
                time.sleep(0.5)
                actual_joint = self.arm.get_current_joint()
                actual_joint_deg = np.rad2deg(actual_joint)
                
                print(f"\nActual final position (实际最终位置):")
                print(f"  Joint {joint_index}: {actual_joint_deg[idx]:7.2f}°")
                print(f"  Error: {abs(actual_joint_deg[idx] - np.rad2deg(target_joint[idx])):5.2f}°")
                
                return True
            else:
                print(f"✗ Movement failed with error code: {ret}")
                return False
                
        except Exception as e:
            print(f"✗ Movement failed with exception: {e}")
            return False
    
    def return_to_initial(self, speed=5):
        """
        Return to initial position
        返回初始位置
        
        Args:
            speed: Movement speed (0-100) (运动速度)
        """
        print(f"\n{'='*60}")
        print(f"  Returning to Initial Position")
        print(f"{'='*60}\n")
        
        # Get current position (获取当前位置)
        current_joint = self.arm.get_current_joint()
        current_joint_deg = np.rad2deg(current_joint)
        
        print(f"Current position (当前位置):")
        for i, angle in enumerate(current_joint_deg, 1):
            print(f"  Joint {i}: {angle:7.2f}°")
        
        print(f"\nInitial position (初始位置):")
        for i, angle in enumerate(self.initial_joint_deg, 1):
            print(f"  Joint {i}: {angle:7.2f}°")
        
        confirm = input(f"\nType 'yes' to return to initial position: ").strip().lower()
        
        if confirm != 'yes':
            print(f"✗ Return cancelled by user")
            return False
        
        # Clear errors (清除错误)
        self.arm.robot.Clear_System_Err()
        
        print(f"\n🤖 Returning to initial position...")
        
        try:
            ret = self.arm.move_joint(self.initial_joint, speed=speed, block=True)
            
            if ret == 0:
                print(f"✓ Returned to initial position successfully!")
                return True
            else:
                print(f"✗ Return failed with error code: {ret}")
                return False
                
        except Exception as e:
            print(f"✗ Return failed with exception: {e}")
            return False
    
    def close(self):
        """
        Close connection and cleanup
        关闭连接和清理
        """
        print(f"\n[Cleanup] Closing connection to {self.arm_name}...")
        self.arm.close()
        print(f"✓ Connection closed")


def main():
    """
    Main interactive testing function
    主交互测试函数
    """
    print("\n" + "="*60)
    print("  Safe Small Movement Test - First Time User Mode")
    print("  安全小幅度运动测试 - 首次使用者模式")
    print("="*60)
    
    # Choose which arm to test (选择测试哪个臂)
    print("\nWhich arm do you want to test?")
    print("你想测试哪个机械臂？")
    print("\n1. Left Arm  (192.168.100.100)")
    print("2. Right Arm (192.168.100.101)")
    print("3. Custom IP")
    
    choice = input("\nEnter your choice (1/2/3): ").strip()
    
    if choice == '1':
        ip_address = '192.168.100.100'
        arm_name = "Left Arm"
    elif choice == '2':
        ip_address = '192.168.100.101'
        arm_name = "Right Arm"
    elif choice == '3':
        ip_address = input("Enter IP address: ").strip()
        arm_name = "Custom Arm"
    else:
        print("Invalid choice!")
        return
    
    # Initialize arm (初始化机械臂)
    try:
        arm_test = SafeArmTest(ip_address, arm_name)
    except Exception as e:
        print(f"\n✗ Failed to initialize arm: {e}")
        return
    
    # Interactive testing loop (交互测试循环)
    print("\n" + "="*60)
    print("  Ready for Testing!")
    print("  准备开始测试！")
    print("="*60)
    
    while True:
        print("\n" + "-"*60)
        print("What would you like to do?")
        print("你想做什么？")
        print("-"*60)
        print("\n1. Test a single joint movement (测试单个关节运动)")
        print("2. Return to initial position (返回初始位置)")
        print("3. Exit (退出)")
        
        action = input("\nEnter your choice (1/2/3): ").strip()
        
        if action == '1':
            # Single joint test (单关节测试)
            print("\n" + "-"*60)
            print("Which joint do you want to move? (1-7)")
            print("你想移动哪个关节？(1-7)")
            print("-"*60)
            print("\nJoint functions (关节功能):")
            print("  Joint 1: Base rotation (基座旋转)")
            print("  Joint 2: Shoulder pitch (肩部俯仰)")
            print("  Joint 3: Shoulder roll (肩部侧摆)")
            print("  Joint 4: Elbow (肘部)")
            print("  Joint 5: Wrist pitch (腕部俯仰)")
            print("  Joint 6: Wrist roll (腕部侧摆)")
            print("  Joint 7: Wrist rotate (腕部旋转)")
            
            try:
                joint_num = int(input("\nEnter joint number (1-7): ").strip())
                
                print("\nHow many degrees to move? (建议 ±5 到 ±10 度)")
                print("  Positive = clockwise/up (正数 = 顺时针/向上)")
                print("  Negative = counter-clockwise/down (负数 = 逆时针/向下)")
                
                offset = float(input("Enter offset in degrees (±5 to ±10 recommended): ").strip())
                
                # Safety limit check (安全限制检查)
                if abs(offset) > 20:
                    print(f"\n⚠️  Warning: {offset}° is quite large for first test!")
                    print(f"   建议首次测试使用 ±5° 到 ±10°")
                    confirm = input("Continue anyway? (yes/no): ").strip().lower()
                    if confirm != 'yes':
                        print("Movement cancelled")
                        continue
                
                # Execute movement (执行运动)
                arm_test.move_single_joint(joint_num, offset, speed=5)
                
            except ValueError:
                print("✗ Invalid input! Please enter numbers only.")
                continue
        
        elif action == '2':
            # Return to initial position (返回初始位置)
            arm_test.return_to_initial(speed=5)
        
        elif action == '3':
            # Exit (退出)
            print("\nExiting test mode...")
            break
        
        else:
            print("Invalid choice!")
    
    # Cleanup (清理)
    arm_test.close()
    
    print("\n" + "="*60)
    print("  Test Session Completed")
    print("  测试会话完成")
    print("="*60 + "\n")


if __name__ == "__main__":
    # Safety reminder (安全提醒)
    print("\n" + "!"*60)
    print("  ⚠️  SAFETY REMINDER / 安全提醒")
    print("!"*60)
    print("""
BEFORE STARTING (开始前):
  1. Clear the workspace - remove all obstacles
     清空工作空间 - 移除所有障碍物
     
  2. Know where the Emergency Stop button is
     知道急停按钮的位置
     
  3. Keep a safe distance during movement
     运动时保持安全距离
     
  4. This script uses SLOW speeds and SMALL movements
     本脚本使用慢速和小幅度运动
     
  5. First movement is always the most important to monitor
     首次运动是最需要监控的
""")
    
    confirm = input("I have read and understand the safety warnings. Continue? (yes/no): ").strip().lower()
    
    if confirm == 'yes':
        try:
            main()
        except KeyboardInterrupt:
            print("\n\n⚠️  Program interrupted by user (Ctrl+C)")
            print("   Emergency stop activated!")
        except Exception as e:
            print(f"\n\n✗ Unexpected error: {e}")
            print("   Please check the arm and restart if needed")
    else:
        print("\nTest cancelled. Safety first! 安全第一！")

