#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试从bbox到motion planning的完整流程（修复坐标系问题）
Test complete pipeline with coordinate fix
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
        self.image_display = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.bbox = None
        self.drawing = False
        self.start_point = None
        self.current_point = None
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for bbox drawing"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            self.current_point = (x, y)
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.current_point = (x, y)
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.current_point = (x, y)
            
            x_min = min(self.start_point[0], self.current_point[0])
            x_max = max(self.start_point[0], self.current_point[0])
            y_min = min(self.start_point[1], self.current_point[1])
            y_max = max(self.start_point[1], self.current_point[1])
            
            self.bbox = [x_min, y_min, x_max, y_max]
            print(f"[INFO] Bbox drawn: [{x_min}, {y_min}, {x_max}, {y_max}]")
    
    def annotate(self):
        """Interactive bbox annotation"""
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
            display = self.image_display.copy()
            
            if self.drawing and self.start_point and self.current_point:
                cv2.rectangle(display, self.start_point, self.current_point, (0, 255, 0), 2)
            elif self.bbox is not None:
                x_min, y_min, x_max, y_max = self.bbox
                cv2.rectangle(display, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                cv2.putText(display, "Target Area", (int(x_min), int(y_min)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(display, "Draw bbox | SPACE: Confirm | R: Reset | ESC: Cancel",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow(window_name, display)
            key = cv2.waitKey(10) & 0xFF
            
            if key == 27:  # ESC
                cv2.destroyAllWindows()
                return None
            elif key == 32:  # SPACE
                if self.bbox is not None:
                    cv2.destroyAllWindows()
                    return self.bbox
                else:
                    print("[WARNING] Please draw a bbox first!")
            elif key == ord('r') or key == ord('R'):
                self.bbox = None
                self.drawing = False
                self.start_point = None
                self.current_point = None
                print("[INFO] Bbox reset")
        
        cv2.destroyAllWindows()
        return self.bbox


def fix_coordinate_transform(target_3d_base_wrong, T_cam2base):
    """
    Fix coordinate transformation issue
    
    The current pipeline applies a flip transform to point cloud that inverts Z axis,
    causing points to be transformed incorrectly. This function compensates for that.
    
    Args:
        target_3d_base_wrong: Target position with wrong Z coordinate
        T_cam2base: Calibration transformation matrix
    
    Returns:
        target_3d_base_corrected: Corrected target position
    """
    # The issue: point cloud has Z-axis flipped (backward instead of forward)
    # When a point is at depth 0.9m (in front of camera), it becomes Z=-0.9m
    # After transformation to base frame, it ends up above the camera
    
    # To fix: We need to reverse the effect of the Z-flip
    # Method: Transform back to camera frame, flip Z, then transform to base again
    
    # Inverse transform: base -> camera
    T_base2cam = np.linalg.inv(T_cam2base)
    
    # Transform to camera frame (homogeneous coordinates)
    target_homo = np.array([target_3d_base_wrong[0], target_3d_base_wrong[1], 
                           target_3d_base_wrong[2], 1.0])
    target_cam_homo = T_base2cam @ target_homo
    target_cam = target_cam_homo[:3]
    
    print(f"\n[Coordinate Fix Debug]")
    print(f"  Original base coord (wrong): ({target_3d_base_wrong[0]:.3f}, {target_3d_base_wrong[1]:.3f}, {target_3d_base_wrong[2]:.3f}) m")
    print(f"  Back to camera frame: ({target_cam[0]:.3f}, {target_cam[1]:.3f}, {target_cam[2]:.3f}) m")
    
    # Flip Z axis to correct the coordinate system
    # Current: Z is negative for points in front
    # Correct: Z should be positive for points in front
    target_cam_corrected = np.array([target_cam[0], target_cam[1], -target_cam[2]])
    print(f"  Camera frame (Z flipped): ({target_cam_corrected[0]:.3f}, {target_cam_corrected[1]:.3f}, {target_cam_corrected[2]:.3f}) m")
    
    # Transform back to base frame with corrected coordinates
    target_cam_corrected_homo = np.array([target_cam_corrected[0], target_cam_corrected[1], 
                                         target_cam_corrected[2], 1.0])
    target_base_corrected_homo = T_cam2base @ target_cam_corrected_homo
    target_base_corrected = target_base_corrected_homo[:3]
    
    print(f"  Corrected base coord: ({target_base_corrected[0]:.3f}, {target_base_corrected[1]:.3f}, {target_base_corrected[2]:.3f}) m")
    
    # Sanity check
    camera_height = T_cam2base[2, 3]
    print(f"\n[Sanity Check]")
    print(f"  Camera height: {camera_height:.3f} m")
    print(f"  Target height (wrong): {target_3d_base_wrong[2]:.3f} m")
    print(f"  Target height (corrected): {target_base_corrected[2]:.3f} m")
    
    if target_base_corrected[2] < camera_height and target_base_corrected[2] > 0:
        print(f"  ✓ Corrected height looks reasonable (below camera, above base)")
    else:
        print(f"  ⚠️  Corrected height may still need adjustment")
    
    return target_base_corrected


def visualize_projection_with_bbox(color_img, pcd_points_cam, camera_intrinsics, bbox, target_3d_cam):
    """Visualize point cloud projection on image with bbox and target point"""
    us, vs, valid = project_pcd_to_image(pcd_points_cam, camera_intrinsics)
    
    vis_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR).copy()
    
    for u, v, is_valid in zip(us, vs, valid):
        if is_valid and 0 <= u < color_img.shape[1] and 0 <= v < color_img.shape[0]:
            cv2.circle(vis_img, (int(u), int(v)), 1, (0, 255, 0), -1)
    
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(vis_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 255), 2)
    cv2.putText(vis_img, "Target Area", (int(x_min), int(y_min)-10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    
    target_us, target_vs, target_valid = project_pcd_to_image(
        target_3d_cam.reshape(1, 3), camera_intrinsics
    )
    target_u, target_v = int(target_us[0]), int(target_vs[0])
    
    if 0 <= target_u < color_img.shape[1] and 0 <= target_v < color_img.shape[0]:
        cv2.drawMarker(vis_img, (target_u, target_v), (0, 0, 255), 
                      cv2.MARKER_CROSS, 20, 3)
        cv2.putText(vis_img, "Target 3D", (target_u+15, target_v-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
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
    
    # Configuration
    CAMERA_SN = "046322250624"
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251222-224450.npz"
    
    # Safety parameters
    SAFE_HEIGHT = 0.15  # Minimum safe height above table (meters) - adjusted
    APPROACH_OFFSET = 0.10  # Approach from 10cm above target (meters)
    MOVE_SPEED = 15  # Movement speed (0-100)
    
    print("\n" + "="*70)
    print("  Bbox to Motion Planning Test (COORDINATE FIXED)")
    print("  测试从bbox标注到左手运动规划的完整流程（坐标修复版）")
    print("="*70)
    
    # Step 1: Get Garment Point Cloud
    print("\n[Step 1/5] Acquiring garment point cloud...")
    print("           获取衣物点云...")
    
    extractor = GarmentPointCloudExtractor(
        camera_sn=CAMERA_SN,
        sam_checkpoint=SAM_CHECKPOINT
    )
    
    color_img, depth_img = extractor.capture_frames()
    img_height, img_width = color_img.shape[:2]
    print(f"  ✓ Captured image: {img_width}x{img_height}")
    
    mask = extractor.interactive_segment(color_img)
    if mask is None:
        print("[ERROR] Segmentation cancelled")
        return
    
    garment_pcd, garment_points_cam = extractor.mask_to_pointcloud(color_img, depth_img, mask)
    print(f"  ✓ Generated point cloud: {len(garment_points_cam)} points")
    
    extractor.save_pointcloud(garment_pcd, "garment_pointcloud.ply")
    
    camera_intrinsics = extractor.camera.o3d_intrinsics.intrinsic_matrix
    
    extractor.camera.close()
    print("  ✓ Camera closed")
    
    # Step 2: Manual Bbox Annotation
    print("\n[Step 2/5] Annotating target bbox...")
    print("           手动标注目标区域...")
    
    annotator = BboxAnnotator(color_img)
    bbox = annotator.annotate()
    
    if bbox is None:
        print("[ERROR] Bbox annotation cancelled")
        return
    
    print(f"  ✓ Bbox annotated: {bbox}")
    
    # Step 3: Map Bbox to 3D
    print("\n[Step 3/5] Mapping bbox to 3D position...")
    print("           将bbox映射到3D坐标...")
    
    if not os.path.exists(CALIB_FILE_LEFT):
        print(f"[ERROR] Calibration file not found: {CALIB_FILE_LEFT}")
        return
    
    calib_left = np.load(CALIB_FILE_LEFT)
    T_cam2base = calib_left['T_cam2base']
    
    us, vs, valid_mask = project_pcd_to_image(garment_points_cam, camera_intrinsics)
    
    result = get_3d_center_from_bbox(
        bbox, garment_points_cam, us, vs, valid_mask,
        img_width, img_height, T_cam2base, sample_stride=3
    )
    
    if result is None:
        print("[ERROR] Failed to map bbox to 3D")
        return
    
    target_3d_base_wrong, target_3d_cam = result
    print(f"  ✓ Target 3D position (camera frame): [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    print(f"  ✓ Target 3D position (base frame, uncorrected): [{target_3d_base_wrong[0]:.3f}, {target_3d_base_wrong[1]:.3f}, {target_3d_base_wrong[2]:.3f}] m")
    
    # Apply coordinate fix
    print("\n[Coordinate Fix] Applying correction for Z-axis flip issue...")
    target_3d_base = fix_coordinate_transform(target_3d_base_wrong, T_cam2base)
    
    print(f"\n  ✓ Final target position (base frame): [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    # Visualize projection
    print("\n[INFO] Visualizing projection...")
    visualize_projection_with_bbox(color_img, garment_points_cam, camera_intrinsics, bbox, target_3d_cam)
    
    # Step 4: Safety Check
    print("\n[Step 4/5] Safety check...")
    print("           安全检查...")
    
    if target_3d_base[2] < SAFE_HEIGHT:
        print(f"  ⚠️  WARNING: Target height ({target_3d_base[2]:.3f}m) is below safe threshold ({SAFE_HEIGHT}m)")
        print(f"  ⚠️  Adjusting target height to {SAFE_HEIGHT}m for safety")
        target_3d_base[2] = SAFE_HEIGHT
    
    if target_3d_base[2] > 0.6:
        print(f"  ⚠️  WARNING: Target height ({target_3d_base[2]:.3f}m) seems too high")
        print(f"  ⚠️  Please verify this is correct before proceeding")
    
    print(f"  ✓ Safe target position: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    # Step 5: Motion Planning
    print("\n[Step 5/5] Motion planning preparation...")
    print("           运动规划准备...")
    
    print("\n" + "="*70)
    print("  ⚠️  SAFETY WARNING")
    print("  The robot will move to the target position!")
    print("  Please ensure:")
    print("    - Workspace is clear")
    print("    - Emergency stop is ready")
    print("    - Target position looks reasonable")
    print("="*70)
    print(f"\n  Target position: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    confirm = input("\nProceed with motion? (yes/no): ").strip().lower()
    if confirm != 'yes':
        print("\n[INFO] Motion cancelled by user")
        print("[INFO] You can review the coordinates above and re-run if needed")
        return
    
    try:
        print("\n[INFO] Initializing robot controller...")
        robot = RealRobotController(camera_sn=CAMERA_SN)
        
        current_pose = robot.get_left_arm_pose()
        print(f"\n[INFO] Current left arm pose:")
        print(f"  Position: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}] m")
        print(f"  Orientation: [{current_pose[3]:.3f}, {current_pose[4]:.3f}, {current_pose[5]:.3f}] rad")
        
        # Phase 1: Move to approach position
        approach_pos = target_3d_base.copy()
        approach_pos[2] += APPROACH_OFFSET
        approach_pose = np.array([
            approach_pos[0], approach_pos[1], approach_pos[2],
            current_pose[3], current_pose[4], current_pose[5]
        ])
        
        print(f"\n[Phase 1] Moving to approach position...")
        print(f"  Target: [{approach_pos[0]:.3f}, {approach_pos[1]:.3f}, {approach_pos[2]:.3f}] m")
        
        robot.left_arm.robot.Clear_System_Err()
        ret = robot.left_arm.robot.Movej_P_Cmd(approach_pose.tolist(), MOVE_SPEED)
        
        if ret == 0:
            print(f"  ✓ Reached approach position")
        else:
            print(f"  ✗ Move failed with error code: {ret}")
            robot.close()
            return
        
        import time
        time.sleep(1.0)
        
        # Phase 2: Move down to target
        target_pose = np.array([
            target_3d_base[0], target_3d_base[1], target_3d_base[2],
            current_pose[3], current_pose[4], current_pose[5]
        ])
        
        print(f"\n[Phase 2] Moving to target position...")
        print(f"  Target: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
        
        robot.left_arm.robot.Clear_System_Err()
        ret = robot.left_arm.robot.Movej_P_Cmd(target_pose.tolist(), MOVE_SPEED // 2)
        
        if ret == 0:
            print(f"  ✓ Reached target position!")
        else:
            print(f"  ✗ Move failed with error code: {ret}")
            robot.close()
            return
        
        time.sleep(1.0)
        
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

