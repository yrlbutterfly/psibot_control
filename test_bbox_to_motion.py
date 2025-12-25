#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试从bbox到motion planning的完整流程
Test complete pipeline: bbox annotation -> 3D mapping -> left arm motion planning
"""

import cv2
import numpy as np
import open3d as o3d
import os
import sys

# Import custom modules
from get_garment_pointcloud import GarmentPointCloudExtractor
from bbox_to_3d import project_pcd_to_image, get_3d_center_from_bbox
from real_robot_controller import RealRobotController


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
    
    def annotate(self):
        """
        Interactive bbox annotation
        
        Returns:
            bbox: [x_min, y_min, x_max, y_max] or None if cancelled
        """
        print("\n" + "="*50)
        print("Interactive Bbox Annotation:")
        print("  - Click and drag to draw bbox")
        print("  - Press SPACE to confirm")
        print("  - Press 'R' to reset")
        print("  - Press ESC to cancel")
        print("="*50 + "\n")
        
        window_name = "Draw Bbox"
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
                cv2.putText(display, "Target Area", (int(x_min), int(y_min)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show instructions
            cv2.putText(display, "Draw bbox | SPACE: Confirm | R: Reset | ESC: Cancel",
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


def visualize_projection_with_bbox(color_img, pcd_points_cam, camera_intrinsics, bbox, target_3d_cam):
    """
    Visualize point cloud projection on image with bbox and target point
    
    Args:
        color_img: RGB image
        pcd_points_cam: Point cloud in camera frame
        camera_intrinsics: Camera intrinsic matrix
        bbox: [x_min, y_min, x_max, y_max]
        target_3d_cam: Target 3D point in camera frame
    """
    # Project point cloud to image
    us, vs, valid = project_pcd_to_image(pcd_points_cam, camera_intrinsics)
    
    # Create visualization image (BGR format)
    vis_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR).copy()
    
    # Draw point cloud projection (green dots)
    for u, v, is_valid in zip(us, vs, valid):
        if is_valid and 0 <= u < color_img.shape[1] and 0 <= v < color_img.shape[0]:
            cv2.circle(vis_img, (int(u), int(v)), 1, (0, 255, 0), -1)
    
    # Draw bbox (yellow)
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(vis_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 255), 2)
    cv2.putText(vis_img, "Target Area", (int(x_min), int(y_min)-10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    # Project target 3D point to image (red cross)
    # target_3d_cam is already in the same coordinate system as pcd_points_cam
    # Use the same projection method as project_pcd_to_image
    target_us, target_vs, target_valid = project_pcd_to_image(
        target_3d_cam.reshape(1, 3), camera_intrinsics
    )
    target_u, target_v = int(target_us[0]), int(target_vs[0])
    
    if 0 <= target_u < color_img.shape[1] and 0 <= target_v < color_img.shape[0]:
        # Draw red cross marker
        cv2.drawMarker(vis_img, (target_u, target_v), (0, 0, 255), 
                      cv2.MARKER_CROSS, 20, 3)
        cv2.putText(vis_img, "Target 3D", (target_u+15, target_v-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Save and display
    cv2.imwrite("bbox_projection_overlay.png", vis_img)
    print(f"[INFO] Saved visualization to bbox_projection_overlay.png")
    
    cv2.namedWindow("Bbox to 3D Projection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Bbox to 3D Projection", 1000, 750)
    cv2.imshow("Bbox to 3D Projection", vis_img)
    print("\n[INFO] Press any key to close visualization...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    """Main test pipeline"""
    
    # ==================== Configuration ====================
    CAMERA_SN = "046322250624"
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251222-224450.npz"
    
    # Safety parameters
    SAFE_HEIGHT = 0.3  # Minimum safe height above table (meters)
    APPROACH_OFFSET = 0.10  # Approach from 10cm above target (meters)
    MOVE_SPEED = 15  # Movement speed (0-100)
    
    print("\n" + "="*70)
    print("  Bbox to Motion Planning Test")
    print("  测试从bbox标注到左手运动规划的完整流程")
    print("="*70)
    
    # ==================== Step 1: Get Garment Point Cloud ====================
    print("\n[Step 1/5] Acquiring garment point cloud...")
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
    
    # ==================== Step 2: Manual Bbox Annotation ====================
    print("\n[Step 2/5] Annotating target bbox...")
    print("           手动标注目标区域...")
    
    annotator = BboxAnnotator(color_img)
    bbox = annotator.annotate()
    
    if bbox is None:
        print("[ERROR] Bbox annotation cancelled")
        return
    
    print(f"  ✓ Bbox annotated: {bbox}")
    
    # ==================== Step 3: Map Bbox to 3D ====================
    print("\n[Step 3/5] Mapping bbox to 3D position...")
    print("           将bbox映射到3D坐标...")
    
    # Load calibration (left arm)
    if not os.path.exists(CALIB_FILE_LEFT):
        print(f"[ERROR] Calibration file not found: {CALIB_FILE_LEFT}")
        return
    
    calib_left = np.load(CALIB_FILE_LEFT)
    T_cam2base = calib_left['T_cam2base']
    # camera_intrinsics already obtained before closing camera
    
    # Project point cloud to image
    us, vs, valid_mask = project_pcd_to_image(garment_points_cam, camera_intrinsics)
    
    # Get 3D center from bbox
    result = get_3d_center_from_bbox(
        bbox, garment_points_cam, us, vs, valid_mask,
        img_width, img_height, T_cam2base, sample_stride=3
    )
    
    if result is None:
        print("[ERROR] Failed to map bbox to 3D")
        return
    
    target_3d_base, target_3d_cam = result
    print(f"  ✓ Target 3D position (base frame): [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    print(f"  ✓ Target 3D position (camera frame): [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    
    # Visualize projection
    print("\n[INFO] Visualizing projection...")
    visualize_projection_with_bbox(color_img, garment_points_cam, camera_intrinsics, bbox, target_3d_cam)
    
    # ==================== Step 4: Safety Check ====================
    print("\n[Step 4/5] Safety check...")
    print("           安全检查...")
    
    # Check if target height is safe
    if target_3d_base[2] < SAFE_HEIGHT:
        print(f"  ⚠️  WARNING: Target height ({target_3d_base[2]:.3f}m) is below safe threshold ({SAFE_HEIGHT}m)")
        print(f"  ⚠️  Adjusting target height to {SAFE_HEIGHT}m for safety")
        target_3d_base[2] = SAFE_HEIGHT
    
    print(f"  ✓ Safe target position: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    # ==================== Step 5: Motion Planning ====================
    print("\n[Step 5/5] Executing left arm motion planning...")
    print("           执行左手运动规划...")
    
    print("\n" + "="*70)
    print("  ⚠️  SAFETY WARNING")
    print("  The robot will now move to the target position!")
    print("  Please ensure:")
    print("    - Workspace is clear")
    print("    - Emergency stop is ready")
    print("    - You are ready to intervene if needed")
    print("="*70)
    
    confirm = input("\nProceed with motion? (yes/no): ").strip().lower()
    if confirm != 'yes':
        print("\n[INFO] Motion cancelled by user")
        return
    
    try:
        # Initialize robot controller
        print("\n[INFO] Initializing robot controller...")
        robot = RealRobotController(camera_sn=CAMERA_SN)
        
        # Get current left arm pose
        current_pose = robot.get_left_arm_pose()
        print(f"\n[INFO] Current left arm pose:")
        print(f"  Position: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}] m")
        print(f"  Orientation: [{current_pose[3]:.3f}, {current_pose[4]:.3f}, {current_pose[5]:.3f}] rad")
        
        # Phase 1: Move to approach position (above target)
        approach_pos = target_3d_base.copy()
        approach_pos[2] += APPROACH_OFFSET
        approach_pose = np.array([
            approach_pos[0], approach_pos[1], approach_pos[2],
            current_pose[3], current_pose[4], current_pose[5]  # Keep current orientation
        ])
        
        print(f"\n[Phase 1] Moving to approach position...")
        print(f"  Target: [{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, {approach_pos[2]:.3f}] m")
        
        # Use move_pose_Cmd from ArmControl
        robot.left_arm.robot.Clear_System_Err()
        ret = robot.left_arm.robot.Movej_P_Cmd(approach_pose.tolist(), MOVE_SPEED)
        
        if ret == 0:
            print(f"  ✓ Reached approach position")
        else:
            print(f"  ✗ Move failed with error code: {ret}")
            robot.close()
            return
        
        import time
        time.sleep(1.0)  # Settle time
        
        # Phase 2: Move down to target position
        target_pose = np.array([
            target_3d_base[0], target_3d_base[1], target_3d_base[2],
            current_pose[3], current_pose[4], current_pose[5]
        ])
        
        print(f"\n[Phase 2] Moving to target position...")
        print(f"  Target: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
        
        robot.left_arm.robot.Clear_System_Err()
        ret = robot.left_arm.robot.Movej_P_Cmd(target_pose.tolist(), MOVE_SPEED // 2)  # Slower for final approach
        
        if ret == 0:
            print(f"  ✓ Reached target position!")
        else:
            print(f"  ✗ Move failed with error code: {ret}")
            robot.close()
            return
        
        time.sleep(1.0)
        
        # Get final pose
        final_pose = robot.get_left_arm_pose()
        print(f"\n[INFO] Final left arm pose:")
        print(f"  Position: [{final_pose[0]:.3f}, {final_pose[1]:.3f}, {final_pose[2]:.3f}] m")
        print(f"  Error: [{final_pose[0]-target_3d_base[0]:.3f}, "
              f"{final_pose[1]-target_3d_base[1]:.3f}, "
              f"{final_pose[2]-target_3d_base[2]:.3f}] m")
        
        print("\n" + "="*70)
        print("  ✓ Motion planning test completed successfully!")
        print("  ✓ 运动规划测试成功完成!")
        print("="*70)
        
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

