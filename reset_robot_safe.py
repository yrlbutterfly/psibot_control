#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safe Robot Reset Tool (Dual Arm)
Safely returns the robot arms to their initial positions step-by-step.
Following the safety protocol from test_bbox_to_motion.py.
"""

import numpy as np
import time
import sys
import os
import json
from real_robot_controller import RealRobotController

# Default initial joint positions (from control.py - UPDATED based on codebase analysis)
# [J1, J2, J3, J4, J5, J6, J7] in radians
# Old/Original: [0.262044, -1.214400, -0.514802, -1.596121, -1.199373, -1.059380, 2.984618]
# Alternative found in control.py (Line 92): [-1.505765, -1.655183, 0.270020, -0.903487, -1.742903, -0.668775, 2.574570]
# Using the one from line 81 as default, but providing options below.
DEFAULT_HOME_QPOS_LEFT = [0.262044, -1.214400, -0.514802, -1.596121, -1.199373, -1.059380, 2.984618]
DEFAULT_HOME_QPOS_RIGHT = [-0.010699, -1.099767, 0.276198, -1.602980, -1.640941, 1.259412, -0.131388]

HOME_CONFIG_FILE = "robot_home_config.json"

def load_home_config():
    """Load home configuration from file if exists, otherwise use defaults"""
    if os.path.exists(HOME_CONFIG_FILE):
        try:
            with open(HOME_CONFIG_FILE, 'r') as f:
                config = json.load(f)
            print(f"\n[INFO] Loaded custom home configuration from {HOME_CONFIG_FILE}")
            return config.get('left', DEFAULT_HOME_QPOS_LEFT), config.get('right', DEFAULT_HOME_QPOS_RIGHT)
        except Exception as e:
            print(f"\n[WARNING] Failed to load config file: {e}")
    
    print("\n[INFO] Using default home configuration (from control.py)")
    return DEFAULT_HOME_QPOS_LEFT, DEFAULT_HOME_QPOS_RIGHT

def save_home_config(left_qpos, right_qpos):
    """Save current configuration as home"""
    config = {
        'left': list(left_qpos),
        'right': list(right_qpos),
        'timestamp': time.time()
    }
    try:
        with open(HOME_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"\n[SUCCESS] Saved current pose as new Home to {HOME_CONFIG_FILE}")
    except Exception as e:
        print(f"\n[ERROR] Failed to save config: {e}")

MOVE_SPEED = 6 # Low speed for safety
STEP_SIZE_RAD = 0.1  # ~5.7 degrees per step max change

def reset_arm(arm_controller, target_qpos, arm_name="Arm"):
    """
    Reset a single arm to target position step-by-step
    """
    print("\n" + "-"*50)
    print(f"  Resetting {arm_name}...")
    print("-"*50)

    # Get current state
    current_qpos = arm_controller.get_current_joint()
    
    # Calculate distance and steps
    current_qpos = np.array(current_qpos)
    target_qpos = np.array(target_qpos)
    
    # Check if we are already close
    max_diff = np.max(np.abs(current_qpos - target_qpos))
    if max_diff < 0.05:
        print(f"[INFO] {arm_name} is already at home position (diff < 0.05 rad).")
        return True
    
    # Preview target
    print(f"  Current Joints (deg): {np.rad2deg(current_qpos).astype(int)}")
    print(f"  Target Joints (deg):  {np.rad2deg(target_qpos).astype(int)}")

    # Calculate number of steps
    num_steps = int(np.ceil(max_diff / STEP_SIZE_RAD))
    if num_steps < 1: num_steps = 1
    
    print(f"  Plan: {num_steps} steps to home position")
    print(f"  Max joint change per step: {STEP_SIZE_RAD:.3f} rad")
    
    # Execution loop
    
    # Check if we should execute directly or step-by-step
    # For this modified version, we will execute the full trajectory but still with speed control
    
    # user_input = input(f"  Execute full reset for {arm_name}? (yes/step/q) > ").strip().lower()
    user_input = "yes" # Default to yes as requested
    
    if user_input == 'q':
         return False
    
    if user_input == 'step':
        # Legacy step-by-step mode
        for i in range(1, num_steps + 1):
            ratio = i / num_steps
            interp_qpos = current_qpos + (target_qpos - current_qpos) * ratio
            print(f"\n[{arm_name} Step {i}/{num_steps}] Target J1-J3: [{interp_qpos[0]:.2f}, {interp_qpos[1]:.2f}, {interp_qpos[2]:.2f}]...")
            if input(f"  Execute {arm_name} Step {i}? (Enter/q) > ").lower() == 'q': return False
            arm_controller.robot.Clear_System_Err()
            arm_controller.move_joint(interp_qpos, speed=MOVE_SPEED, block=True)
            time.sleep(0.1)
    else:
        # Continuous mode (but safely)
        print(f"\n[INFO] Executing continuous reset for {arm_name}...")
        arm_controller.robot.Clear_System_Err()
        # Direct move to target with safe speed
        ret = arm_controller.move_joint(target_qpos, speed=MOVE_SPEED, block=True)
        if ret != 0:
            print(f"[WARNING] Move command returned code {ret}")

    print(f"  ✓ {arm_name} reset complete")
    return True

def reset_hand(robot, hand_side="left", action="open"):
    """
    Reset hand state
    """
    print(f"\n[INFO] Resetting {hand_side} hand to {action} state...")
    try:
        if hand_side == "left":
            if action == "open":
                robot.open_left_hand()
            elif action == "close":
                robot.close_left_hand()
        elif hand_side == "right":
            if action == "open":
                robot.open_right_hand()
            elif action == "close":
                robot.close_right_hand()
        print(f"  ✓ {hand_side} hand {action} command sent")
    except Exception as e:
        print(f"  ✗ Failed to reset {hand_side} hand: {e}")

def main():
    print("\n" + "="*70)
    print("  Safe Robot Reset Tool (Dual Arm & Hands)")
    print("  双臂/双手安全复位工具")
    print("="*70)

    # Initialize controller
    print("\n[INFO] Initializing robot controller...")
    try:
        robot = RealRobotController()
    except Exception as e:
        print(f"[ERROR] Failed to initialize robot: {e}")
        return

    # Load home config
    home_left, home_right = load_home_config()

    try:
        while True:
            print("\n" + "="*50)
            print("Select Operation:")
            print("  --- Arms (机械臂) ---")
            print("  1. Reset Left Arm (左臂复位)")
            print("  2. Reset Right Arm (右臂复位)")
            print("  3. Reset Both Arms (双臂复位)")
            print("  --- Hands (灵巧手) ---")
            print("  4. Open Left Hand (张开左手)")
            print("  5. Open Right Hand (张开右手)")
            print("  6. Open Both Hands (张开双手)")
            print("  --- Configuration (配置) ---")
            print("  8. SET CURRENT POSE AS HOME (设当前位置为初始点 - 慎用)")
            print("  9. Reload Home Config (重新加载初始点配置)")
            print("  q. Quit (退出)")
            print("="*50)
            
            choice = input("Enter choice: ").strip().lower()
            
            if choice == 'q':
                break
                
            if choice == '1':
                reset_arm(robot.left_arm, home_left, "Left Arm")
            
            elif choice == '2':
                reset_arm(robot.right_arm, home_right, "Right Arm")
                
            elif choice == '3':
                print("\n[INFO] Starting sequential reset for both arms...")
                if reset_arm(robot.left_arm, home_left, "Left Arm"):
                    print("\n[INFO] Left arm done. Proceeding to right arm...")
                    reset_arm(robot.right_arm, home_right, "Right Arm")
            
            elif choice == '4':
                reset_hand(robot, "left", "open")
                
            elif choice == '5':
                reset_hand(robot, "right", "open")
                
            elif choice == '6':
                reset_hand(robot, "left", "open")
                time.sleep(0.5)
                reset_hand(robot, "right", "open")
                
            elif choice == '8':
                print("\n[WARNING] You are about to overwrite the default HOME position.")
                print("Make sure both arms are in a SAFE, collision-free initial state.")
                confirm = input("Are you sure you want to save CURRENT POSE as HOME? (yes/no): ").strip().lower()
                if confirm == 'yes':
                    cur_left = robot.left_arm.get_current_joint()
                    cur_right = robot.right_arm.get_current_joint()
                    save_home_config(cur_left, cur_right)
                    # Update current session
                    home_left, home_right = cur_left, cur_right
            
            elif choice == '9':
                home_left, home_right = load_home_config()
            
            else:
                print("[ERROR] Invalid choice")

        print("\n[INFO] Exiting...")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during reset: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'robot' in locals():
            robot.close()

if __name__ == "__main__":
    main()
