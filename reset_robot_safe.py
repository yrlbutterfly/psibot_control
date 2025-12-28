#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safe Robot Reset Tool (Dual Arm)
Safely returns the robot arms to their initial positions step-by-step.
Following the safety protocol from test_bbox_to_motion.py.

Updates:
- Added support for multiple preset positions (Home, Photo)
"""

import numpy as np
import time
import sys
import os
import json
from real_robot_controller import RealRobotController

# Default initial joint positions (from control.py)
# [J1, J2, J3, J4, J5, J6, J7] in radians
# Using these as fallbacks if config files don't exist
DEFAULT_HOME_QPOS_LEFT = [0.262044, -1.214400, -0.514802, -1.596121, -1.199373, -1.059380, 2.984618]
DEFAULT_HOME_QPOS_RIGHT = [-0.010699, -1.099767, 0.276198, -1.602980, -1.640941, 1.259412, -0.131388]

HOME_CONFIG_FILE = "robot_home_config.json"
PHOTO_CONFIG_FILE = "robot_photo_config.json"

def load_pose_config(filename, config_name="Home"):
    """
    Generic function to load a pose configuration from a file.
    Returns (left_qpos, right_qpos).
    """
    if os.path.exists(filename):
        try:
            with open(filename, 'r') as f:
                config = json.load(f)
            print(f"\n[INFO] Loaded {config_name} configuration from {filename}")
            return config.get('left', DEFAULT_HOME_QPOS_LEFT), config.get('right', DEFAULT_HOME_QPOS_RIGHT)
        except Exception as e:
            print(f"\n[WARNING] Failed to load {filename}: {e}")
    
    print(f"\n[INFO] {filename} not found. Using default internal values for {config_name}.")
    return DEFAULT_HOME_QPOS_LEFT, DEFAULT_HOME_QPOS_RIGHT

def load_home_config():
    """
    Wrapper for backward compatibility to load Home config specifically.
    Returns (left_qpos, right_qpos)
    """
    return load_pose_config(HOME_CONFIG_FILE, "Home")

def save_pose_config(filename, left_qpos, right_qpos, config_name="Config"):
    """
    Generic function to save current configuration to a file.
    """
    config = {
        'left': list(left_qpos),
        'right': list(right_qpos),
        'timestamp': time.time(),
        'name': config_name
    }
    try:
        with open(filename, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"\n[SUCCESS] Saved current pose as {config_name} to {filename}")
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
        print(f"[INFO] {arm_name} is already at target position (diff < 0.05 rad).")
        return True
    
    # Preview target
    print(f"  Current Joints (deg): {np.rad2deg(current_qpos).astype(int)}")
    print(f"  Target Joints (deg):  {np.rad2deg(target_qpos).astype(int)}")

    # Calculate number of steps
    num_steps = int(np.ceil(max_diff / STEP_SIZE_RAD))
    if num_steps < 1: num_steps = 1
    
    print(f"  Plan: {num_steps} steps to target")
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

    print(f"  ✓ {arm_name} movement complete")
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
    print("  双臂/双手安全复位工具 (Home & Photo Mode)")
    print("="*70)

    # Initialize controller
    print("\n[INFO] Initializing robot controller...")
    try:
        robot = RealRobotController()
    except Exception as e:
        print(f"[ERROR] Failed to initialize robot: {e}")
        return

    # Load configurations
    home_left, home_right = load_pose_config(HOME_CONFIG_FILE, "Home")
    photo_left, photo_right = load_pose_config(PHOTO_CONFIG_FILE, "Photo")

    try:
        while True:
            print("\n" + "="*60)
            print("Select Operation:")
            print("  --- Move to HOME (复位到初始点) ---")
            print("  1. Left Arm  -> Home")
            print("  2. Right Arm -> Home")
            print("  3. Both Arms -> Home")
            print("  --- Move to PHOTO (移动到拍照点) ---")
            print("  4. Left Arm  -> Photo")
            print("  5. Right Arm -> Photo")
            print("  6. Both Arms -> Photo")
            print("  --- Hands (灵巧手) ---")
            print("  7. Open Left Hand")
            print("  8. Open Right Hand")
            print("  9. Open Both Hands")
            print("  --- Configuration (配置管理) ---")
            print("  s_home.  Save Current as HOME (设为Home点)")
            print("  s_photo. Save Current as PHOTO (设为Photo点)")
            print("  r.       Reload All Configs (重载配置)")
            print("  q.       Quit (退出)")
            print("="*60)
            
            choice = input("Enter choice: ").strip().lower()
            
            if choice == 'q':
                break
                
            # --- HOME Operations ---
            elif choice == '1':
                reset_arm(robot.left_arm, home_left, "Left Arm (Home)")
            
            elif choice == '2':
                reset_arm(robot.right_arm, home_right, "Right Arm (Home)")
                
            elif choice == '3':
                print("\n[INFO] Starting sequential reset for both arms to HOME...")
                if reset_arm(robot.left_arm, home_left, "Left Arm (Home)"):
                    print("\n[INFO] Left arm done. Proceeding to right arm...")
                    reset_arm(robot.right_arm, home_right, "Right Arm (Home)")

            # --- PHOTO Operations ---
            elif choice == '4':
                reset_arm(robot.left_arm, photo_left, "Left Arm (Photo)")

            elif choice == '5':
                reset_arm(robot.right_arm, photo_right, "Right Arm (Photo)")

            elif choice == '6':
                print("\n[INFO] Starting sequential move for both arms to PHOTO...")
                if reset_arm(robot.left_arm, photo_left, "Left Arm (Photo)"):
                    print("\n[INFO] Left arm done. Proceeding to right arm...")
                    reset_arm(robot.right_arm, photo_right, "Right Arm (Photo)")
            
            # --- HAND Operations ---
            elif choice == '7':
                reset_hand(robot, "left", "open")
                
            elif choice == '8':
                reset_hand(robot, "right", "open")
                
            elif choice == '9':
                reset_hand(robot, "left", "open")
                time.sleep(0.5)
                reset_hand(robot, "right", "open")
            
            # --- SAVE/CONFIG Operations ---
            elif choice == 's_home':
                print("\n[WARNING] You are about to overwrite the default HOME position.")
                if input("Are you sure? (yes/no): ").strip().lower() == 'yes':
                    cur_left = robot.left_arm.get_current_joint()
                    cur_right = robot.right_arm.get_current_joint()
                    save_pose_config(HOME_CONFIG_FILE, cur_left, cur_right, "Home")
                    home_left, home_right = cur_left, cur_right

            elif choice == 's_photo':
                print("\n[WARNING] You are about to overwrite the PHOTO position.")
                if input("Are you sure? (yes/no): ").strip().lower() == 'yes':
                    cur_left = robot.left_arm.get_current_joint()
                    cur_right = robot.right_arm.get_current_joint()
                    save_pose_config(PHOTO_CONFIG_FILE, cur_left, cur_right, "Photo")
                    photo_left, photo_right = cur_left, cur_right
            
            elif choice == 'r':
                home_left, home_right = load_pose_config(HOME_CONFIG_FILE, "Home")
                photo_left, photo_right = load_pose_config(PHOTO_CONFIG_FILE, "Photo")
            
            else:
                print("[ERROR] Invalid choice")

        print("\n[INFO] Exiting...")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during operation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'robot' in locals():
            robot.close()

if __name__ == "__main__":
    main()