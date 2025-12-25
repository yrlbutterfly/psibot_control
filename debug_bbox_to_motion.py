#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试bbox到motion planning的坐标转换
Debug coordinate transformation without moving the robot
"""

import cv2
import numpy as np
import open3d as o3d
import os
import sys

# Import custom modules
from get_garment_pointcloud import GarmentPointCloudExtractor
from bbox_to_3d import project_pcd_to_image, get_3d_center_from_bbox


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


def visualize_3d_coordinates(garment_points_cam, target_3d_cam, garment_points_base, target_3d_base, T_cam2base):
    """
    Visualize point cloud and target in both camera and base frames
    
    Args:
        garment_points_cam: Point cloud in camera frame (N x 3)
        target_3d_cam: Target point in camera frame (3,)
        garment_points_base: Point cloud in base frame (N x 3)
        target_3d_base: Target point in base frame (3,)
        T_cam2base: Transformation matrix from camera to base (4 x 4)
    """
    print("\n" + "="*70)
    print("3D Coordinate Visualization")
    print("="*70)
    
    # Create two point clouds for visualization
    # Camera frame point cloud (blue)
    pcd_cam = o3d.geometry.PointCloud()
    pcd_cam.points = o3d.utility.Vector3dVector(garment_points_cam)
    pcd_cam.paint_uniform_color([0.0, 0.5, 1.0])  # Blue
    
    # Base frame point cloud (green)
    pcd_base = o3d.geometry.PointCloud()
    pcd_base.points = o3d.utility.Vector3dVector(garment_points_base)
    pcd_base.paint_uniform_color([0.0, 1.0, 0.0])  # Green
    
    # Create spheres for target points
    # Target in camera frame (red)
    target_sphere_cam = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
    target_sphere_cam.translate(target_3d_cam)
    target_sphere_cam.paint_uniform_color([1.0, 0.0, 0.0])  # Red
    
    # Target in base frame (yellow)
    target_sphere_base = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
    target_sphere_base.translate(target_3d_base)
    target_sphere_base.paint_uniform_color([1.0, 1.0, 0.0])  # Yellow
    
    # Create coordinate frames
    # Camera frame origin
    camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    # Base frame origin (at transformed position)
    base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    # Transform base frame to show relationship
    # We want to show where camera is in base frame
    T_base2cam = np.linalg.inv(T_cam2base)
    camera_in_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    camera_in_base.transform(T_base2cam)
    
    print("\n[INFO] Visualization Instructions:")
    print("  - Blue points: Point cloud in camera frame")
    print("  - Green points: Point cloud in base frame")
    print("  - Red sphere: Target in camera frame")
    print("  - Yellow sphere: Target in base frame")
    print("  - Small RGB axis: Camera frame")
    print("  - Large RGB axis: Base frame")
    print("\n  Press 'Q' to close each window")
    
    # Visualize camera frame view
    print("\n[Visualization 1] Camera Frame View")
    print("  Showing: Point cloud (blue) + Target (red) in camera coordinates")
    o3d.visualization.draw_geometries(
        [pcd_cam, target_sphere_cam, camera_frame],
        window_name="Camera Frame View",
        width=1024, height=768
    )
    
    # Visualize base frame view
    print("\n[Visualization 2] Base Frame View")
    print("  Showing: Point cloud (green) + Target (yellow) in base coordinates")
    o3d.visualization.draw_geometries(
        [pcd_base, target_sphere_base, base_frame, camera_in_base],
        window_name="Base Frame View",
        width=1024, height=768
    )


def check_coordinate_sanity(target_3d_cam, target_3d_base, garment_points_cam, garment_points_base):
    """
    Check if coordinates are in reasonable ranges
    
    Args:
        target_3d_cam: Target point in camera frame
        target_3d_base: Target point in base frame
        garment_points_cam: Point cloud in camera frame
        garment_points_base: Point cloud in base frame
    """
    print("\n" + "="*70)
    print("Coordinate Sanity Check")
    print("="*70)
    
    # Camera frame analysis
    print("\n[Camera Frame]")
    print(f"  Target position: [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    print(f"  Point cloud range:")
    print(f"    X: [{garment_points_cam[:, 0].min():.3f}, {garment_points_cam[:, 0].max():.3f}] m")
    print(f"    Y: [{garment_points_cam[:, 1].min():.3f}, {garment_points_cam[:, 1].max():.3f}] m")
    print(f"    Z: [{garment_points_cam[:, 2].min():.3f}, {garment_points_cam[:, 2].max():.3f}] m")
    
    # Check if target is within point cloud bounds
    in_bounds_x = garment_points_cam[:, 0].min() <= target_3d_cam[0] <= garment_points_cam[:, 0].max()
    in_bounds_y = garment_points_cam[:, 1].min() <= target_3d_cam[1] <= garment_points_cam[:, 1].max()
    in_bounds_z = garment_points_cam[:, 2].min() <= target_3d_cam[2] <= garment_points_cam[:, 2].max()
    
    if in_bounds_x and in_bounds_y and in_bounds_z:
        print(f"  ✓ Target is within point cloud bounds")
    else:
        print(f"  ⚠️  Target is outside point cloud bounds!")
        if not in_bounds_x:
            print(f"     - X is out of bounds")
        if not in_bounds_y:
            print(f"     - Y is out of bounds")
        if not in_bounds_z:
            print(f"     - Z is out of bounds")
    
    # Base frame analysis
    print("\n[Base Frame]")
    print(f"  Target position: [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    print(f"  Point cloud range:")
    print(f"    X: [{garment_points_base[:, 0].min():.3f}, {garment_points_base[:, 0].max():.3f}] m")
    print(f"    Y: [{garment_points_base[:, 1].min():.3f}, {garment_points_base[:, 1].max():.3f}] m")
    print(f"    Z: [{garment_points_base[:, 2].min():.3f}, {garment_points_base[:, 2].max():.3f}] m")
    
    # Typical robot workspace check
    print("\n[Robot Workspace Check]")
    workspace_warnings = []
    
    # Typical RM arm reach: ~0.5-1.0m from base
    reach = np.sqrt(target_3d_base[0]**2 + target_3d_base[1]**2)
    print(f"  Horizontal reach: {reach:.3f} m")
    if reach < 0.2:
        workspace_warnings.append("Target too close to base (< 0.2m)")
    if reach > 1.0:
        workspace_warnings.append("Target may be too far from base (> 1.0m)")
    
    # Height check (Z should be positive and reasonable)
    if target_3d_base[2] < 0:
        workspace_warnings.append("Target is below base plane (Z < 0)")
    if target_3d_base[2] < 0.1:
        workspace_warnings.append("Target is very close to table (Z < 0.1m)")
    if target_3d_base[2] > 0.8:
        workspace_warnings.append("Target is very high (Z > 0.8m)")
    
    # X-Y range check
    if abs(target_3d_base[0]) > 1.0:
        workspace_warnings.append(f"X coordinate very large ({target_3d_base[0]:.3f}m)")
    if abs(target_3d_base[1]) > 1.0:
        workspace_warnings.append(f"Y coordinate very large ({target_3d_base[1]:.3f}m)")
    
    if not workspace_warnings:
        print(f"  ✓ Target position looks reasonable")
    else:
        print(f"  ⚠️  Potential issues detected:")
        for warning in workspace_warnings:
            print(f"     - {warning}")
    
    # Camera position analysis
    print("\n[Camera Position in Base Frame]")
    # Camera is at origin in camera frame, which is -T_cam2base[:3, 3] in base frame
    # Actually, camera position in base frame is T_cam2base applied to [0,0,0,1]
    # Or simply T_cam2base[:3, 3]
    
    return len(workspace_warnings) == 0


def analyze_transformation_matrix(T_cam2base):
    """
    Analyze the transformation matrix
    
    Args:
        T_cam2base: 4x4 transformation matrix from camera to base
    """
    print("\n" + "="*70)
    print("Transformation Matrix Analysis")
    print("="*70)
    
    # Extract rotation and translation
    R = T_cam2base[:3, :3]
    t = T_cam2base[:3, 3]
    
    print("\n[Translation (Camera position in base frame)]")
    print(f"  t = [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m")
    
    # Check if camera is in reasonable position
    camera_height = t[2]
    camera_distance = np.sqrt(t[0]**2 + t[1]**2)
    print(f"  Camera height above base: {camera_height:.3f} m")
    print(f"  Camera horizontal distance from base: {camera_distance:.3f} m")
    
    if camera_height < 0.2:
        print(f"  ⚠️  Camera seems very low (< 0.2m)")
    elif camera_height > 1.5:
        print(f"  ⚠️  Camera seems very high (> 1.5m)")
    else:
        print(f"  ✓ Camera height looks reasonable")
    
    # Rotation matrix analysis
    print("\n[Rotation Matrix]")
    print("  R = ")
    for i in range(3):
        print(f"    [{R[i, 0]:7.3f}, {R[i, 1]:7.3f}, {R[i, 2]:7.3f}]")
    
    # Check if rotation matrix is valid (det(R) should be 1)
    det_R = np.linalg.det(R)
    print(f"\n  det(R) = {det_R:.6f}")
    if abs(det_R - 1.0) < 0.01:
        print(f"  ✓ Determinant is close to 1 (valid rotation)")
    else:
        print(f"  ⚠️  Determinant is not close to 1 (may include scaling)")
    
    # Check orthogonality (R @ R.T should be identity)
    RTR = R @ R.T
    is_orthogonal = np.allclose(RTR, np.eye(3), atol=0.01)
    if is_orthogonal:
        print(f"  ✓ Matrix is orthogonal (valid rotation)")
    else:
        print(f"  ⚠️  Matrix is not orthogonal")
        print(f"  R @ R.T =")
        for i in range(3):
            print(f"    [{RTR[i, 0]:7.3f}, {RTR[i, 1]:7.3f}, {RTR[i, 2]:7.3f}]")
    
    # Extract Euler angles (for reference)
    # Using ZYX convention (roll, pitch, yaw)
    try:
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        print(f"\n[Euler Angles (Roll, Pitch, Yaw)]")
        print(f"  Roll:  {np.degrees(roll):7.2f}° ({roll:.3f} rad)")
        print(f"  Pitch: {np.degrees(pitch):7.2f}° ({pitch:.3f} rad)")
        print(f"  Yaw:   {np.degrees(yaw):7.2f}° ({yaw:.3f} rad)")
        
    except Exception as e:
        print(f"\n[Euler Angles] Could not extract: {e}")


def main():
    """Main debug pipeline"""
    
    # Configuration
    CAMERA_SN = "046322250624"
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251222-224450.npz"
    
    print("\n" + "="*70)
    print("  Bbox to Motion Planning Coordinate Debug")
    print("  调试bbox到运动规划的坐标转换")
    print("  (不会移动机械臂)")
    print("="*70)
    
    # Step 1: Get Garment Point Cloud
    print("\n[Step 1/4] Acquiring garment point cloud...")
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
    
    # Get camera intrinsics
    camera_intrinsics = extractor.camera.o3d_intrinsics.intrinsic_matrix
    
    # Close camera
    extractor.camera.close()
    print("  ✓ Camera closed")
    
    # Step 2: Manual Bbox Annotation
    print("\n[Step 2/4] Annotating target bbox...")
    print("           手动标注目标区域...")
    
    annotator = BboxAnnotator(color_img)
    bbox = annotator.annotate()
    
    if bbox is None:
        print("[ERROR] Bbox annotation cancelled")
        return
    
    print(f"  ✓ Bbox annotated: {bbox}")
    
    # Step 3: Map Bbox to 3D
    print("\n[Step 3/4] Mapping bbox to 3D position...")
    print("           将bbox映射到3D坐标...")
    
    # Load calibration
    if not os.path.exists(CALIB_FILE_LEFT):
        print(f"[ERROR] Calibration file not found: {CALIB_FILE_LEFT}")
        return
    
    calib_left = np.load(CALIB_FILE_LEFT)
    T_cam2base = calib_left['T_cam2base']
    
    print("\n[Calibration Data Loaded]")
    print(f"  File: {CALIB_FILE_LEFT}")
    
    # Analyze transformation matrix
    analyze_transformation_matrix(T_cam2base)
    
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
    print(f"\n  ✓ Target 3D position (camera frame): [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    print(f"  ✓ Target 3D position (base frame):   [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    # Transform all points to base frame for comparison
    garment_points_cam_homo = np.hstack([garment_points_cam, np.ones((len(garment_points_cam), 1))])
    garment_points_base_homo = (T_cam2base @ garment_points_cam_homo.T).T
    garment_points_base = garment_points_base_homo[:, :3]
    
    # Step 4: Coordinate Analysis
    print("\n[Step 4/4] Coordinate analysis...")
    print("           坐标分析...")
    
    # Sanity check
    is_sane = check_coordinate_sanity(target_3d_cam, target_3d_base, garment_points_cam, garment_points_base)
    
    # 3D visualization
    print("\n[INFO] Launching 3D visualization...")
    visualize_3d_coordinates(garment_points_cam, target_3d_cam, garment_points_base, target_3d_base, T_cam2base)
    
    # Summary
    print("\n" + "="*70)
    print("  Debug Summary")
    print("="*70)
    print(f"\n  Target in camera frame: [{target_3d_cam[0]:.3f}, {target_3d_cam[1]:.3f}, {target_3d_cam[2]:.3f}] m")
    print(f"  Target in base frame:   [{target_3d_base[0]:.3f}, {target_3d_base[1]:.3f}, {target_3d_base[2]:.3f}] m")
    
    if is_sane:
        print(f"\n  ✓ Coordinates look reasonable for robot motion")
    else:
        print(f"\n  ⚠️  Coordinates may need adjustment - see warnings above")
    
    print("\n" + "="*70)
    print("  Debug completed (no robot motion performed)")
    print("  调试完成（未移动机械臂）")
    print("="*70)


if __name__ == "__main__":
    main()

