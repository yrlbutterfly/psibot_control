#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize point cloud projection overlay on RGB image
在RGB图像上叠加显示点云投影
"""

import cv2
import numpy as np
import open3d as o3d
from bbox_to_3d import project_pcd_to_image
from robot_libs.realsense_image_module import RealSenseImage


def visualize_projection_overlay():
    """Overlay projected points on RGB image"""
    print("\n" + "="*60)
    print("  Point Cloud Projection Overlay")
    print("="*60 + "\n")
    
    # Load RGB image
    print("[1/4] Loading RGB image...")
    rgb_image = cv2.imread('captured_color.png')
    print(f"✓ Image shape: {rgb_image.shape}")
    
    # Load point cloud (camera frame)
    print("\n[2/4] Loading point cloud...")
    pcd_cam = o3d.io.read_point_cloud('garment_pointcloud.ply')
    points_cam = np.asarray(pcd_cam.points)
    print(f"✓ Loaded {len(points_cam)} points")
    
    # Get camera intrinsics
    print("\n[3/4] Getting camera intrinsics...")
    camera = RealSenseImage(SN_number="046322250624")
    K = camera.o3d_intrinsics.intrinsic_matrix
    camera.close()
    print("✓ Camera intrinsics obtained")
    
    # Project to image
    print("\n[4/4] Projecting points...")
    us, vs, valid = project_pcd_to_image(points_cam, K)
    
    print(f"✓ Valid points: {np.sum(valid)}/{len(points_cam)}")
    
    if np.sum(valid) == 0:
        print("❌ No valid points to project!")
        return
    
    # Filter to image bounds
    img_h, img_w = rgb_image.shape[:2]
    in_bounds = (
        valid &
        (us >= 0) & (us < img_w) &
        (vs >= 0) & (vs < img_h)
    )
    
    print(f"✓ Points in image bounds: {np.sum(in_bounds)}/{np.sum(valid)}")
    
    # Create overlay
    overlay = rgb_image.copy()
    
    # Draw all projected points
    for u, v in zip(us[in_bounds], vs[in_bounds]):
        cv2.circle(overlay, (int(u), int(v)), 1, (0, 255, 0), -1)
    
    # Blend with original
    alpha = 0.6
    blended = cv2.addWeighted(rgb_image, 1-alpha, overlay, alpha, 0)
    
    # Add statistics
    cv2.putText(blended, f"Projected points: {np.sum(in_bounds)}", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(blended, "Green dots = point cloud data", 
               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(blended, "Draw bbox in areas with green dots", 
               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Save
    cv2.imwrite('projection_overlay.png', blended)
    print("\n✓ Saved projection_overlay.png")
    
    # Display
    cv2.namedWindow("Projection Overlay", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Projection Overlay", 1000, 750)
    cv2.imshow("Projection Overlay", blended)
    
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("  Tips:")
    print("="*60)
    print("- Green dots show where you have point cloud data")
    print("- Draw bboxes only in green areas")
    print("- If an area has no green dots, no 3D points will be found")
    print("- Sparse areas may give less accurate 3D positions")
    print("="*60 + "\n")


if __name__ == "__main__":
    import os
    if not os.path.exists('captured_color.png'):
        print("❌ captured_color.png not found")
        exit(1)
    if not os.path.exists('garment_pointcloud.ply'):
        print("❌ garment_pointcloud.ply not found")
        exit(1)
    
    visualize_projection_overlay()

