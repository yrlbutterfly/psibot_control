#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试注释掉transform后的坐标是否正确
Test if coordinates are correct after commenting out the transform
"""

import cv2
import numpy as np
import open3d as o3d
import os

# Import custom modules
from get_garment_pointcloud import GarmentPointCloudExtractor
from bbox_to_3d import project_pcd_to_image, get_3d_center_from_bbox


class BboxAnnotator:
    """Interactive bbox annotation tool"""
    
    def __init__(self, image):
        self.image = image.copy()
        self.image_display = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.bbox = None
        self.drawing = False
        self.start_point = None
        self.current_point = None
    
    def mouse_callback(self, event, x, y, flags, param):
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


def main():
    """Main test function"""
    
    CAMERA_SN = "046322250624"
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251222-224450.npz"
    
    print("\n" + "="*70)
    print("  Coordinate Test After Removing Transform")
    print("  测试移除transform后的坐标")
    print("="*70)
    
    # Step 1: Get point cloud
    print("\n[Step 1/3] Acquiring garment point cloud...")
    
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
    
    camera_intrinsics = extractor.camera.o3d_intrinsics.intrinsic_matrix
    extractor.camera.close()
    
    # Step 2: Annotate bbox
    print("\n[Step 2/3] Annotating target bbox...")
    
    annotator = BboxAnnotator(color_img)
    bbox = annotator.annotate()
    
    if bbox is None:
        print("[ERROR] Bbox annotation cancelled")
        return
    
    print(f"  ✓ Bbox annotated: {bbox}")
    
    # Step 3: Get 3D coordinates
    print("\n[Step 3/3] Mapping bbox to 3D...")
    
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
    
    target_3d_base, target_3d_cam = result
    
    # Analysis
    print("\n" + "="*70)
    print("  Coordinate Analysis")
    print("="*70)
    
    camera_height = T_cam2base[2, 3]
    
    print(f"\n[Camera Frame Coordinates]")
    print(f"  Target position: [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    print(f"  Point cloud Z range: [{garment_points_cam[:, 2].min():.3f}, {garment_points_cam[:, 2].max():.3f}] m")
    
    if target_3d_cam[2] > 0:
        print(f"  ✓ Z is positive (point is in front of camera)")
    else:
        print(f"  ⚠️  Z is negative (point is behind camera - this seems wrong)")
    
    print(f"\n[Base Frame Coordinates]")
    print(f"  Camera height: {camera_height:.3f} m")
    print(f"  Target height: {target_3d_base[2]:.3f} m")
    print(f"  Target position: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    # Comprehensive checks
    print(f"\n[Sanity Checks]")
    checks_passed = 0
    checks_total = 0
    
    # Check 1: Target should be below camera
    checks_total += 1
    if target_3d_base[2] < camera_height:
        print(f"  ✓ Target is below camera ({target_3d_base[2]:.3f}m < {camera_height:.3f}m)")
        checks_passed += 1
    else:
        print(f"  ✗ Target is above camera ({target_3d_base[2]:.3f}m > {camera_height:.3f}m)")
    
    # Check 2: Target should be above base (positive Z)
    checks_total += 1
    if target_3d_base[2] > 0:
        print(f"  ✓ Target is above base plane (Z > 0)")
        checks_passed += 1
    else:
        print(f"  ✗ Target is below base plane (Z < 0)")
    
    # Check 3: Target should be at reasonable table height (0.05-0.4m)
    checks_total += 1
    if 0.05 <= target_3d_base[2] <= 0.4:
        print(f"  ✓ Target height is reasonable for table surface (0.05-0.4m)")
        checks_passed += 1
    else:
        print(f"  ⚠️  Target height {target_3d_base[2]:.3f}m is outside typical table range (0.05-0.4m)")
    
    # Check 4: Horizontal distance should be reasonable (0.2-1.0m)
    checks_total += 1
    horizontal_dist = np.sqrt(target_3d_base[0]**2 + target_3d_base[1]**2)
    if 0.2 <= horizontal_dist <= 1.0:
        print(f"  ✓ Horizontal distance from base is reasonable ({horizontal_dist:.3f}m)")
        checks_passed += 1
    else:
        print(f"  ⚠️  Horizontal distance {horizontal_dist:.3f}m may be out of reach")
    
    # Check 5: Point cloud should be in front of camera (positive Z)
    checks_total += 1
    if garment_points_cam[:, 2].mean() > 0:
        print(f"  ✓ Point cloud is in front of camera (average Z > 0)")
        checks_passed += 1
    else:
        print(f"  ✗ Point cloud is behind camera (average Z < 0)")
    
    # Overall assessment
    print(f"\n" + "="*70)
    print(f"  Overall Assessment: {checks_passed}/{checks_total} checks passed")
    print("="*70)
    
    if checks_passed == checks_total:
        print("\n  ✓✓✓ All checks passed! Coordinates look correct!")
        print("  ✓ The transform removal fixed the coordinate issue.")
        print("  ✓ You can now safely use test_bbox_to_motion.py")
    elif checks_passed >= 4:
        print("\n  ✓ Most checks passed. Coordinates look reasonable.")
        print("  ⚠️  Minor issues detected - review warnings above")
    else:
        print("\n  ✗ Multiple checks failed. Coordinates may still be incorrect.")
        print("  ⚠️  Further investigation needed")
    
    print(f"\n[Summary]")
    print(f"  Camera height: {camera_height:.3f} m")
    print(f"  Target height: {target_3d_base[2]:.3f} m")
    print(f"  Target position: ({target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}) m")
    
    if checks_passed >= 4:
        print(f"\n  💡 This target position should be safe for robot motion.")
    else:
        print(f"\n  ⚠️  Review the coordinates before moving the robot!")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    main()

