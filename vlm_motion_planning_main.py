#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM + Motion Planning 主程序
整合所有模块实现完整的视觉-语言-运动规划流程
参考 Isaac Sim 的 Fold_Tops_HALO_mp.py
"""

import os
import sys
import time
import cv2
import numpy as np
import open3d as o3d
import json
from datetime import datetime

# Import custom modules
from robot_libs.realsense_image_module import RealSenseImage, generate_pcd
from robot_libs.realman_arm_module import ArmControl
from robot_libs.aoyi_hand_module_modbus import Hand

from vlm_interface import VLMInterface
from bbox_to_3d import BboxTo3DProjector
from motion_primitives import MotionPrimitives


# ========================================
# Configuration
# ========================================
CONFIG = {
    # Hardware
    "camera_sn": "046322250624",  # 你的头部相机序列号
    "left_arm_ip": "192.168.100.100",
    "right_arm_ip": "192.168.100.101",
    "left_hand_port": "/dev/ttyUSB0",
    "right_hand_port": "/dev/ttyUSB1",
    
    # Calibration (根据你要用哪个机械臂选择对应的标定文件)
    "calib_file_right": "calibration_results/camera_calibration_right_arm_20251226-234855.npz",  # 右臂标定
    "calib_file_left": "calibration_results/camera_calibration_left_arm_20251226-234053.npz",   # 左臂标定
    "calib_file": "calibration_results/camera_calibration_right_arm_20251226-234855.npz",  # 默认使用右臂
    "intrinsics_file": "camera_intrinsics.json",
    
    # VLM
    "vlm_base_url": "http://localhost:8000/v1",  # TODO: Update to your VLM endpoint
    "vlm_model_name": "qwen-vl-plus",  # TODO: Update to your model name
    
    # SAM 2.1 (optional, for initial segmentation)
    "sam_checkpoint": "sam2.1_hiera_large.pt",
    
    # Task parameters
    "max_subtasks": 6,  # Maximum folding steps
    "debug": True,  # Enable debug mode (save images, logs)
    "debug_dir": "debug_output"
}


class VLMMotionPlanningSystem:
    """Complete VLM + Motion Planning system"""
    
    def __init__(self, config):
        self.config = config
        print("\n" + "="*60)
        print("Initializing VLM + Motion Planning System")
        print("="*60)
        
        # Initialize hardware
        print("[1/6] Initializing hardware...")
        self.camera = RealSenseImage(SN_number=config["camera_sn"])
        self.left_arm = ArmControl(ip=config["left_arm_ip"])
        self.right_arm = ArmControl(ip=config["right_arm_ip"])
        self.left_hand = Hand(port=config["left_hand_port"])
        self.right_hand = Hand(port=config["right_hand_port"])
        
        # Load calibration
        print("[2/6] Loading calibration...")
        calib_data = np.load(config["calib_file"])
        self.T_cam2base = calib_data['T_cam2base']
        print(f"  Loaded cam2base transform:\n{self.T_cam2base}")
        
        # Load camera intrinsics
        print("[3/6] Loading camera intrinsics...")
        self.intrinsics = o3d.io.read_pinhole_camera_intrinsic(config["intrinsics_file"])
        
        # Initialize VLM
        print("[4/6] Initializing VLM...")
        self.vlm = VLMInterface(
            base_url=config["vlm_base_url"],
            model_name=config["vlm_model_name"]
        )
        
        # Initialize projector (2D bbox -> 3D)
        print("[5/6] Initializing bbox projector...")
        self.projector = BboxTo3DProjector(self.intrinsics, self.T_cam2base)
        
        # Initialize motion primitives
        print("[6/6] Initializing motion primitives...")
        self.motion = MotionPrimitives(
            self.left_arm, self.right_arm,
            self.left_hand, self.right_hand
        )
        
        # Setup debug directory
        if config["debug"]:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.debug_dir = os.path.join(config["debug_dir"], timestamp)
            os.makedirs(self.debug_dir, exist_ok=True)
            print(f"[DEBUG] Output directory: {self.debug_dir}")
        
        print("\n✓ Initialization complete!\n")
    
    def capture_observation(self):
        """Capture RGB-D observation and generate point cloud"""
        print("[OBS] Capturing observation...")
        
        # Capture frames
        color_img, depth_img = self.camera.capture_frame()
        
        # Generate full point cloud
        full_pcd = generate_pcd(color_img, depth_img, self.intrinsics, visualize_flag=False)
        pcd_points_cam = np.asarray(full_pcd.points)
        
        # TODO: Here you should segment the garment from the scene
        # For now, we use the full point cloud
        # In production, use SAM or other segmentation methods (see get_garment_pointcloud.py)
        garment_pcd_cam = pcd_points_cam
        
        print(f"  Captured RGB: {color_img.shape}, Depth: {depth_img.shape}")
        print(f"  Point cloud: {len(garment_pcd_cam)} points")
        
        return color_img, depth_img, garment_pcd_cam
    
    def save_debug_image(self, step_idx, img, vlm_points, suffix=""):
        """Save debug images with VLM bboxes visualized"""
        if not self.config["debug"]:
            return
        
        # Save raw image
        raw_path = os.path.join(self.debug_dir, f"step_{step_idx:02d}_raw{suffix}.png")
        cv2.imwrite(raw_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        
        # Save with bboxes
        if vlm_points:
            vis_img = self.projector.visualize_bbox_on_image(img, vlm_points)
            bbox_path = os.path.join(self.debug_dir, f"step_{step_idx:02d}_bbox{suffix}.png")
            cv2.imwrite(bbox_path, cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR))
    
    def save_debug_vlm_output(self, step_idx, vlm_result):
        """Save VLM output to JSON"""
        if not self.config["debug"]:
            return
        
        log_path = os.path.join(self.debug_dir, f"step_{step_idx:02d}_vlm_output.json")
        with open(log_path, 'w', encoding='utf-8') as f:
            json.dump(vlm_result, f, indent=2, ensure_ascii=False)
    
    def run_folding_task(self):
        """Main task loop: VLM planning + Motion execution"""
        print("\n" + "="*60)
        print("Starting Garment Folding Task")
        print("="*60 + "\n")
        
        # Move to home position
        print("[INIT] Moving to home position...")
        self.motion.open_both_hands()
        self.motion.move_to_home(settle_time=2.0)
        
        # Main loop
        max_subtasks = self.config["max_subtasks"]
        finished = False
        
        for step_idx in range(max_subtasks):
            print("\n" + "-"*60)
            print(f"Step {step_idx + 1}/{max_subtasks}")
            print("-"*60)
            
            # Step 1: Capture observation
            color_img, depth_img, garment_pcd_cam = self.capture_observation()
            
            # Save debug images
            self.save_debug_image(step_idx, color_img, None, suffix="_before")
            
            # Step 2: Query VLM for next action
            print("[VLM] Querying VLM for next action...")
            vlm_result = self.vlm.ask_vlm_plan(color_img)
            
            plan = vlm_result.get("plan")
            vlm_points = vlm_result.get("points", [])
            
            # Save VLM output
            self.save_debug_vlm_output(step_idx, vlm_result)
            self.save_debug_image(step_idx, color_img, vlm_points, suffix="_vlm")
            
            # Check if finished
            if isinstance(plan, str) and "finish" in plan.lower():
                print("[VLM] Task complete: already finish folding")
                finished = True
                break
            
            if not isinstance(plan, list) or len(plan) == 0:
                print(f"[WARNING] Invalid plan format: {plan}")
                break
            
            # Extract the first (and should be only) action
            action = plan[0]
            print(f"[PLAN] Action: {action}")
            
            # Step 3: Convert VLM 2D bboxes to 3D points (camera frame)
            print("[3D] Converting bboxes to 3D points...")
            label_to_3d_cam = self.projector.vlm_points_to_3d_map(
                vlm_points, garment_pcd_cam
            )
            
            if not label_to_3d_cam:
                print("[ERROR] Failed to convert any keypoints to 3D")
                break
            
            # Step 4: Transform to base frame
            print("[3D] Transforming to base frame...")
            label_to_3d_base = {}
            for label, point_cam in label_to_3d_cam.items():
                point_base = self.projector.cam_to_base(point_cam)
                label_to_3d_base[label] = point_base
                print(f"  {label}: {point_base}")
            
            # Step 5: Execute motion
            print("[MP] Executing motion primitive...")
            success = self.motion.execute_vlm_action(action, label_to_3d_base)
            
            if not success:
                print("[ERROR] Motion execution failed")
                # Continue to next step or break
            
            # Step 6: Return to home
            print("[MP] Returning to home position...")
            self.motion.move_to_home(settle_time=2.0)
            
            print(f"✓ Step {step_idx + 1} complete\n")
        
        # Final observation
        print("\n" + "="*60)
        print("Task Loop Complete")
        print("="*60)
        
        if finished:
            print("✓ Folding task finished successfully!")
        else:
            print("⚠ Task ended without completion signal")
        
        # Capture final state
        final_color, _, _ = self.capture_observation()
        self.save_debug_image(999, final_color, None, suffix="_final")
        
        # Return to home
        self.motion.move_to_home()
        print("\n✓ All operations complete!\n")
    
    def cleanup(self):
        """Cleanup resources"""
        print("[CLEANUP] Closing connections...")
        try:
            self.left_arm.close()
            self.right_arm.close()
            print("✓ Cleanup complete")
        except Exception as e:
            print(f"⚠ Cleanup warning: {e}")


def main():
    """Main entry point"""
    print("\n" + "="*60)
    print("VLM + Motion Planning for Garment Folding")
    print("Real Robot Implementation")
    print("="*60 + "\n")
    
    # Check if calibration file exists
    if not os.path.exists(CONFIG["calib_file"]):
        print(f"[ERROR] Calibration file not found: {CONFIG['calib_file']}")
        print("Please run camera_calibreate.py first to perform hand-eye calibration.")
        return
    
    # Initialize system
    try:
        system = VLMMotionPlanningSystem(CONFIG)
    except Exception as e:
        print(f"[ERROR] Failed to initialize system: {e}")
        return
    
    try:
        # Run task
        system.run_folding_task()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Task failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        system.cleanup()
    
    print("\n" + "="*60)
    print("Program Exit")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()

