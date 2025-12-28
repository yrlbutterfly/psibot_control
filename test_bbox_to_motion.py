#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试从bbox到motion planning的完整流程 - 抓取并放置
Test complete pipeline: bbox annotation (Grasp & Place) -> 3D mapping -> Motion Sequence
"""

import cv2
import numpy as np
import open3d as o3d
import os
import sys
import time
import json

# Import custom modules
from get_garment_pointcloud import GarmentPointCloudExtractor
from bbox_to_3d import project_pcd_to_image, get_3d_center_from_bbox
from real_robot_controller import RealRobotController
# from motion_primitives import MotionPlanner # Removed as we use the functional interface now

class BboxAnnotator:
    """Interactive bbox annotation tool"""

    def __init__(self, image):
        """
        Args:
            image: RGB image (H x W x 3) in RGB format
        """
        self.image = image.copy()
        self.image_display = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert to BGR for display
        self.bbox = None
        self.drawing = False
        self.start_point = None
        self.current_point = None

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for bbox drawing"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Start drawing
            self.drawing = True
            self.start_point = (x, y)
            self.current_point = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE:
            # Update current point while drawing
            if self.drawing:
                self.current_point = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            # Finish drawing
            self.drawing = False
            self.current_point = (x, y)

            # Calculate bbox [x_min, y_min, x_max, y_max]
            x_min = min(self.start_point[0], self.current_point[0])
            x_max = max(self.start_point[0], self.current_point[0])
            y_min = min(self.start_point[1], self.current_point[1])
            y_max = max(self.start_point[1], self.current_point[1])

            self.bbox = [x_min, y_min, x_max, y_max]
            print(f"[INFO] Bbox drawn: [{x_min}, {y_min}, {x_max}, {y_max}]")

    def annotate(self, title="Draw Bbox"):
        """
        Interactive bbox annotation

        Args:
            title: Window title and prompt

        Returns:
            bbox: [x_min, y_min, x_max, y_max] or None if cancelled
        """
        # Reset state for new annotation
        self.bbox = None
        self.drawing = False
        self.start_point = None
        self.current_point = None
        
        print("\n" + "="*50)
        print(f"Interactive Bbox Annotation: {title}")
        print("  - Click and drag to draw bbox")
        print("  - Press SPACE to confirm")
        print("  - Press 'R' to reset")
        print("  - Press ESC to cancel")
        print("="*50 + "\n")

        window_name = title
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1000, 750)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while True:
            # Create display image
            display = self.image_display.copy()

            # Draw current bbox
            if self.drawing and self.start_point and self.current_point:
                cv2.rectangle(display, self.start_point, self.current_point, (0, 255, 0), 2)
            elif self.bbox is not None:
                x_min, y_min, x_max, y_max = self.bbox
                cv2.rectangle(display, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                # Add label
                cv2.putText(display, title, (int(x_min), int(y_min)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show instructions
            cv2.putText(display, f"{title} | SPACE: Confirm | R: Reset | ESC: Cancel",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(10) & 0xFF

            if key == 27:  # ESC: cancel
                cv2.destroyAllWindows()
                return None
            elif key == 32:  # SPACE: confirm
                if self.bbox is not None:
                    cv2.destroyAllWindows()
                    return self.bbox
                else:
                    print("[WARNING] Please draw a bbox first!")
            elif key == ord('r') or key == ord('R'):  # Reset
                self.bbox = None
                self.drawing = False
                self.start_point = None
                self.current_point = None
                print("[INFO] Bbox reset")

        cv2.destroyAllWindows()
        return self.bbox


def visualize_projection_with_bboxes(color_img, pcd_points_cam, camera_intrinsics, bboxes, targets_3d_cam, labels=None):
    """
    Visualize point cloud projection on image with multiple bboxes and target points

    Args:
        color_img: RGB image
        pcd_points_cam: Point cloud in camera frame
        camera_intrinsics: Camera intrinsic matrix
        bboxes: List of [x_min, y_min, x_max, y_max]
        targets_3d_cam: List of Target 3D points in camera frame (numpy arrays)
        labels: List of label strings
    """
    if labels is None:
        labels = [f"Target {i+1}" for i in range(len(bboxes))]

    # Project point cloud to image
    us, vs, valid = project_pcd_to_image(pcd_points_cam, camera_intrinsics)

    # Create visualization image (BGR format)
    vis_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR).copy()

    # Draw point cloud projection (green dots)
    for u, v, is_valid in zip(us, vs, valid):
        if is_valid and 0 <= u < color_img.shape[1] and 0 <= v < color_img.shape[0]:
            cv2.circle(vis_img, (int(u), int(v)), 1, (0, 255, 0), -1)

    colors = [(0, 255, 255), (255, 0, 255), (255, 255, 0), (0, 255, 0)] # Yellow, Magenta, Cyan, Green

    for i, (bbox, target_3d, label) in enumerate(zip(bboxes, targets_3d_cam, labels)):
        color = colors[i % len(colors)]
        
        # Draw bbox
        x_min, y_min, x_max, y_max = bbox
        cv2.rectangle(vis_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
        cv2.putText(vis_img, label, (int(x_min), int(y_min)-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Project target 3D point to image (cross)
        target_us, target_vs, target_valid = project_pcd_to_image(
            target_3d.reshape(1, 3), camera_intrinsics
        )
        target_u, target_v = int(target_us[0]), int(target_vs[0])

        if 0 <= target_u < color_img.shape[1] and 0 <= target_v < color_img.shape[0]:
            # Draw cross marker
            cv2.drawMarker(vis_img, (target_u, target_v), (0, 0, 255), 
                          cv2.MARKER_CROSS, 20, 3)
            cv2.putText(vis_img, f"{label} 3D", (target_u+15, target_v-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Save and display
    cv2.imwrite("bbox_projection_overlay.png", vis_img)
    print(f"[INFO] Saved visualization to bbox_projection_overlay.png")

    cv2.namedWindow("Bbox to 3D Projection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Bbox to 3D Projection", 1000, 750)
    
    # Add instruction text to image
    cv2.putText(vis_img, "Press SPACE/ENTER to Confirm Motion", (20, 40), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(vis_img, "Press ESC to Cancel", (20, 80), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    cv2.imshow("Bbox to 3D Projection", vis_img)
    print("\n[INFO] Waiting for user confirmation in GUI window...")
    print("       Press SPACE/ENTER to confirm motion, ESC to cancel.")
    
    proceed = False
    
    # Loop to wait for key and handle window events better
    while True:
        k = cv2.waitKey(100)
        
        if k == 27: # ESC
            print("[INFO] User cancelled via GUI")
            proceed = False
            break
        elif k == 32 or k == 13: # SPACE or ENTER
            print("[INFO] User confirmed via GUI")
            proceed = True
            break
            
        # Check if window was closed by X button (robustness check)
        try:
            if cv2.getWindowProperty("Bbox to 3D Projection", cv2.WND_PROP_VISIBLE) < 1:
                print("[INFO] Window closed, treating as cancel")
                proceed = False
                break
        except:
            proceed = False
            break

    cv2.destroyAllWindows()
    for _ in range(10): cv2.waitKey(1) # Pump events multiple times to clear buffer
    time.sleep(0.5) # Give system time to restore focus to terminal
    
    return proceed


def robust_input_main(prompt=""):
    """
    Robust input function for main script
    Works even after OpenCV operations by reading from /dev/tty directly
    """
    print(prompt, end='', flush=True)
    try:
        with open('/dev/tty', 'r') as tty:
            return tty.readline().rstrip('\n')
    except:
        try:
            return input()
        except EOFError:
            return ""  # Return empty string as default

def main():
    """Main test pipeline"""

    # ==================== Configuration ====================
    CAMERA_SN = "046322250624"
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251226-234053.npz"
    CALIB_FILE_RIGHT = "calibration_results/camera_calibration_right_arm_20251226-234855.npz"

    # Arm selection: 'left', 'right', or 'bimanual'
    ARM_SELECT = 'right'  # Default to right arm
    CONTROL_MODE = 'single'  # 'single' or 'bimanual'

    # Safety parameters
    SAFE_HEIGHT = -0.5  # Modified: Allow going lower than table for now
    LIFT_HEIGHT = 0.08 # Lift 10cm after grasp
    
    # ==================== Manual Offset Correction (Debug) ====================
    MANUAL_OFFSET_X = 0.0   # meters
    MANUAL_OFFSET_Y = 0.0   # meters
    MANUAL_OFFSET_Z = 0.0   # meters
    # ==========================================================================

    print("\n" + "="*70)
    print(f"  Grasp & Place Motion Test")
    print(f"  测试: 抓取点 -> 抬起 -> 放置点 -> 放下")
    print("="*70)

    # Select control mode
    mode_choice = robust_input_main("Select control mode (1=single arm, 2=bimanual) [default=1]: ").strip()
    if mode_choice == '2':
        CONTROL_MODE = 'bimanual'
        print("[INFO] Selected mode: BIMANUAL (双臂)")
    else:
        CONTROL_MODE = 'single'
        # Select arm for single mode
        arm_choice = robust_input_main(f"Select arm to control (left/right) [default={ARM_SELECT}]: ").strip().lower()
        if arm_choice in ['left', 'right']:
            ARM_SELECT = arm_choice
        print(f"[INFO] Selected mode: SINGLE ARM - {ARM_SELECT.upper()}")

    # ==================== Step 0: Move to Photo Position ====================
    PHOTO_CONFIG_FILE = "robot_photo_config.json"
    HOME_CONFIG_FILE = "robot_home_config.json"

    def move_to_preset(config_file, pose_name):
        print(f"\n[Pre-Check] Moving robot to {pose_name} position...")
        if not os.path.exists(config_file):
            print(f"  [SKIP] Config file {config_file} not found.")
            return
        
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            
            # Initialize temp controller
            tmp_robot = RealRobotController()
            
            if 'left' in config:
                print(f"  Moving Left Arm to {pose_name}...")
                tmp_robot.left_arm.move_joint(config['left'], speed=6, block=True)
            
            if 'right' in config:
                print(f"  Moving Right Arm to {pose_name}...")
                tmp_robot.right_arm.move_joint(config['right'], speed=6, block=True)
                
            tmp_robot.close()
            print(f"  ✓ Robot at {pose_name}")
            
        except Exception as e:
            print(f"  [WARNING] Failed to move to {pose_name}: {e}")

    # Move to Photo position before capture
    move_to_preset(PHOTO_CONFIG_FILE, "Photo")

    # ==================== Step 1: Get Garment Point Cloud ====================
    print("\n[Step 1/6] Acquiring garment point cloud...")
    print("           获取衣物点云...")

    extractor = GarmentPointCloudExtractor(
        camera_sn=CAMERA_SN,
        sam_checkpoint=SAM_CHECKPOINT
    )

    # Capture frames
    color_img, depth_img = extractor.capture_frames()
    img_height, img_width = color_img.shape[:2]
    print(f"  ✓ Captured image: {img_width}x{img_height}")

    # Interactive segmentation
    mask = extractor.interactive_segment(color_img)
    if mask is None:
        print("[ERROR] Segmentation cancelled")
        return

    # Generate point cloud
    garment_pcd, garment_points_cam = extractor.mask_to_pointcloud(color_img, depth_img, mask)
    print(f"  ✓ Generated point cloud: {len(garment_points_cam)} points")

    # Save point cloud
    extractor.save_pointcloud(garment_pcd, "garment_pointcloud.ply")

    # Get camera intrinsics before closing
    camera_intrinsics = extractor.camera.o3d_intrinsics.intrinsic_matrix

    # Close camera to free the device for later use
    extractor.camera.close()
    print("  ✓ Camera closed (freed for motion planning stage)")

    # Move back to Home position after capture
    move_to_preset(HOME_CONFIG_FILE, "Home")

    # ==================== Step 2: Annotate Points ====================
    print("\n[Step 2/6] Annotating Grasp and Place points...")
    print("           标注抓取点和放置点...")

    annotator = BboxAnnotator(color_img)
    
    if CONTROL_MODE == 'single':
        # Single arm mode: annotate 2 points (grasp, place)
        # 2.1 Annotate Grasp Point
        print("\n  >> Please draw GRASP area")
        bbox_grasp = annotator.annotate(title="1. Draw GRASP Area")
        if bbox_grasp is None:
            print("[ERROR] Grasp annotation cancelled")
            return
        print(f"  ✓ Grasp bbox: {bbox_grasp}")

        # 2.2 Annotate Place Point
        print("\n  >> Please draw PLACE area")
        bbox_place = annotator.annotate(title="2. Draw PLACE Area")
        if bbox_place is None:
            print("[ERROR] Place annotation cancelled")
            return
        print(f"  ✓ Place bbox: {bbox_place}")
        
    else:  # CONTROL_MODE == 'bimanual'
        # Bimanual mode: annotate 4 points (left grasp, left place, right grasp, right place)
        print("\n  >> Please draw LEFT hand GRASP area")
        bbox_left_grasp = annotator.annotate(title="1. Left GRASP Area")
        if bbox_left_grasp is None:
            print("[ERROR] Left grasp annotation cancelled")
            return
        print(f"  ✓ Left Grasp bbox: {bbox_left_grasp}")
        
        print("\n  >> Please draw LEFT hand PLACE area")
        bbox_left_place = annotator.annotate(title="2. Left PLACE Area")
        if bbox_left_place is None:
            print("[ERROR] Left place annotation cancelled")
            return
        print(f"  ✓ Left Place bbox: {bbox_left_place}")
        
        print("\n  >> Please draw RIGHT hand GRASP area")
        bbox_right_grasp = annotator.annotate(title="3. Right GRASP Area")
        if bbox_right_grasp is None:
            print("[ERROR] Right grasp annotation cancelled")
            return
        print(f"  ✓ Right Grasp bbox: {bbox_right_grasp}")
        
        print("\n  >> Please draw RIGHT hand PLACE area")
        bbox_right_place = annotator.annotate(title="4. Right PLACE Area")
        if bbox_right_place is None:
            print("[ERROR] Right place annotation cancelled")
            return
        print(f"  ✓ Right Place bbox: {bbox_right_place}")

    # ==================== Step 3: Map to 3D ====================
    print("\n[Step 3/6] Mapping to 3D positions...")
    print("           映射到3D坐标...")

    # Project point cloud to image
    us, vs, valid_mask = project_pcd_to_image(garment_points_cam, camera_intrinsics)

    # Helper to get 3D point
    def get_point_3d(bbox, label, T_cam2base):
        result = get_3d_center_from_bbox(
            bbox, garment_points_cam, us, vs, valid_mask,
            img_width, img_height, T_cam2base, sample_stride=3
        )
        if result is None:
            print(f"[ERROR] Failed to map {label} bbox to 3D")
            return None, None
        return result

    if CONTROL_MODE == 'single':
        # Single arm mode
        # Load calibration based on selected arm
        calib_file = CALIB_FILE_LEFT if ARM_SELECT == 'left' else CALIB_FILE_RIGHT

        if not os.path.exists(calib_file):
            print(f"[ERROR] Calibration file not found: {calib_file}")
            return

        calib_data = np.load(calib_file)
        T_cam2base = calib_data['T_cam2base']

        # Get Grasp 3D
        grasp_3d_base, grasp_3d_cam = get_point_3d(bbox_grasp, "Grasp", T_cam2base)
        if grasp_3d_base is None: return

        # Get Place 3D
        place_3d_base, place_3d_cam = get_point_3d(bbox_place, "Place", T_cam2base)
        if place_3d_base is None: return

        # Apply offsets
        if MANUAL_OFFSET_X != 0 or MANUAL_OFFSET_Y != 0 or MANUAL_OFFSET_Z != 0:
            offset = np.array([MANUAL_OFFSET_X, MANUAL_OFFSET_Y, MANUAL_OFFSET_Z])
            grasp_3d_base += offset
            place_3d_base += offset
            print(f"  [DEBUG] Applied manual offset: {offset}")

        print(f"  ✓ Grasp Position: [{grasp_3d_base[0]:.3f}, {grasp_3d_base[1]:.3f}, {grasp_3d_base[2]:.3f}] m")
        print(f"  ✓ Place Position: [{place_3d_base[0]:.3f}, {place_3d_base[1]:.3f}, {place_3d_base[2]:.3f}] m")

        # Visualize and Confirm
        print("\n[INFO] Visualizing points...")
        proceed = visualize_projection_with_bboxes(
            color_img, garment_points_cam, camera_intrinsics,
            [bbox_grasp, bbox_place],
            [grasp_3d_cam, place_3d_cam],
            ["Grasp", "Place"]
        )

    else:  # CONTROL_MODE == 'bimanual'
        # Bimanual mode: need both calibrations
        if not os.path.exists(CALIB_FILE_LEFT) or not os.path.exists(CALIB_FILE_RIGHT):
            print(f"[ERROR] Calibration files not found")
            print(f"  Left: {CALIB_FILE_LEFT}")
            print(f"  Right: {CALIB_FILE_RIGHT}")
            return

        calib_data_left = np.load(CALIB_FILE_LEFT)
        calib_data_right = np.load(CALIB_FILE_RIGHT)
        T_cam2base_left = calib_data_left['T_cam2base']
        T_cam2base_right = calib_data_right['T_cam2base']

        # Get Left Grasp & Place 3D
        left_grasp_3d_base, left_grasp_3d_cam = get_point_3d(bbox_left_grasp, "Left Grasp", T_cam2base_left)
        if left_grasp_3d_base is None: return
        
        left_place_3d_base, left_place_3d_cam = get_point_3d(bbox_left_place, "Left Place", T_cam2base_left)
        if left_place_3d_base is None: return

        # Get Right Grasp & Place 3D
        right_grasp_3d_base, right_grasp_3d_cam = get_point_3d(bbox_right_grasp, "Right Grasp", T_cam2base_right)
        if right_grasp_3d_base is None: return
        
        right_place_3d_base, right_place_3d_cam = get_point_3d(bbox_right_place, "Right Place", T_cam2base_right)
        if right_place_3d_base is None: return

        # Apply offsets
        if MANUAL_OFFSET_X != 0 or MANUAL_OFFSET_Y != 0 or MANUAL_OFFSET_Z != 0:
            offset = np.array([MANUAL_OFFSET_X, MANUAL_OFFSET_Y, MANUAL_OFFSET_Z])
            left_grasp_3d_base += offset
            left_place_3d_base += offset
            right_grasp_3d_base += offset
            right_place_3d_base += offset
            print(f"  [DEBUG] Applied manual offset: {offset}")

        print(f"  ✓ Left Grasp:  [{left_grasp_3d_base[0]:.3f}, {left_grasp_3d_base[1]:.3f}, {left_grasp_3d_base[2]:.3f}] m")
        print(f"  ✓ Left Place:  [{left_place_3d_base[0]:.3f}, {left_place_3d_base[1]:.3f}, {left_place_3d_base[2]:.3f}] m")
        print(f"  ✓ Right Grasp: [{right_grasp_3d_base[0]:.3f}, {right_grasp_3d_base[1]:.3f}, {right_grasp_3d_base[2]:.3f}] m")
        print(f"  ✓ Right Place: [{right_place_3d_base[0]:.3f}, {right_place_3d_base[1]:.3f}, {right_place_3d_base[2]:.3f}] m")

        # Visualize and Confirm (show all 4 points)
        print("\n[INFO] Visualizing points...")
        proceed = visualize_projection_with_bboxes(
            color_img, garment_points_cam, camera_intrinsics,
            [bbox_left_grasp, bbox_left_place, bbox_right_grasp, bbox_right_place],
            [left_grasp_3d_cam, left_place_3d_cam, right_grasp_3d_cam, right_place_3d_cam],
            ["L_Grasp", "L_Place", "R_Grasp", "R_Place"]
        )

    if not proceed:
        print("\n[INFO] Motion cancelled by user")
        return

    # ==================== Step 4: Safety Check ====================
    print("\n[Step 4/6] Safety check...")
    
    if CONTROL_MODE == 'single':
        for label, pos in [("Grasp", grasp_3d_base), ("Place", place_3d_base)]:
            if pos[2] < SAFE_HEIGHT:
                print(f"  ⚠️  WARNING: {label} height ({pos[2]:.3f}m) is below safe threshold ({SAFE_HEIGHT}m)")
                print(f"  ⚠️  Please be careful!")
    else:  # bimanual
        for label, pos in [("Left Grasp", left_grasp_3d_base), ("Left Place", left_place_3d_base),
                           ("Right Grasp", right_grasp_3d_base), ("Right Place", right_place_3d_base)]:
            if pos[2] < SAFE_HEIGHT:
                print(f"  ⚠️  WARNING: {label} height ({pos[2]:.3f}m) is below safe threshold ({SAFE_HEIGHT}m)")
                print(f"  ⚠️  Please be careful!")

    # ==================== Step 5: Motion Planning ====================
    if CONTROL_MODE == 'single':
        print(f"\n[Step 5/6] Executing {ARM_SELECT} arm motion planning...")
        print(f"           执行{ARM_SELECT}手运动规划...")
    else:
        print(f"\n[Step 5/6] Executing bimanual motion planning...")
        print(f"           执行双臂运动规划...")

    print("\n" + "="*70)
    print("  ⚠️  SAFETY WARNING")
    if CONTROL_MODE == 'single':
        print("  The robot will move: Grasp -> Lift -> Place -> Release")
    else:
        print("  Both robot arms will move: Grasp -> Lift -> Place -> Release")
    print("  Please ensure workspace is clear!")
    print("="*70)

    try:
        # Initialize robot controller
        print("\n[INFO] Initializing robot controller...")
        robot = RealRobotController(camera_sn=CAMERA_SN)

        # Execute Pick and Place sequence
        from motion_primitives import mp_right_fold, mp_left_fold, mp_bimanual_fold
        
        if CONTROL_MODE == 'single':
            if ARM_SELECT == 'right':
                success = mp_right_fold(
                    robot_controller=robot,
                    grasp_pos=grasp_3d_base,
                    place_pos=place_3d_base,
                    lift_height=LIFT_HEIGHT,
                    reset_joints=None  # Will use default from load_home_config
                )
            else:
                success = mp_left_fold(
                    robot_controller=robot,
                    grasp_pos=grasp_3d_base,
                    place_pos=place_3d_base,
                    lift_height=LIFT_HEIGHT,
                    reset_joints=None
                )
        else:  # bimanual
            success = mp_bimanual_fold(
                robot_controller=robot,
                left_grasp_pos=left_grasp_3d_base,
                right_grasp_pos=right_grasp_3d_base,
                left_place_pos=left_place_3d_base,
                right_place_pos=right_place_3d_base,
                lift_height=LIFT_HEIGHT
            )
        
        if success:
            print("\n[SUCCESS] Pipeline completed successfully!")
        else:
            print("\n[FAILED] Pipeline stopped or failed.")

        # Cleanup
        robot.close()

    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        if 'robot' in locals():
            robot.close()
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        if 'robot' in locals():
            robot.close()


if __name__ == "__main__":
    main()
