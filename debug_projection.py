#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug projection issues
调试投影问题
"""

import numpy as np
import open3d as o3d
import cv2
from robot_libs.realsense_image_module import RealSenseImage


def debug_projection():
    print("\n" + "="*60)
    print("  Debugging Projection")
    print("="*60 + "\n")
    
    # 1. Load point cloud (base frame)
    print("[1/4] Loading point cloud...")
    pcd_base = o3d.io.read_point_cloud('garment_pointcloud_base.ply')
    points_base = np.asarray(pcd_base.points)
    print(f"✓ Loaded {len(points_base)} points")
    print(f"  Base frame range:")
    print(f"    X: {points_base[:,0].min():.3f} ~ {points_base[:,0].max():.3f}")
    print(f"    Y: {points_base[:,1].min():.3f} ~ {points_base[:,1].max():.3f}")
    print(f"    Z: {points_base[:,2].min():.3f} ~ {points_base[:,2].max():.3f}")
    
    # 2. Load calibration
    print("\n[2/4] Loading calibration...")
    calib = np.load('calibration_results/camera_calibration_right_arm_20251226-234855.npz')
    T_cam2base = calib['T_cam2base']
    print(f"✓ Calibration loaded")
    
    # 3. Transform back to camera frame
    print("\n[3/4] Transforming to camera frame...")
    T_base2cam = np.linalg.inv(T_cam2base)
    
    # Sample a few points
    sample_indices = np.random.choice(len(points_base), min(10, len(points_base)), replace=False)
    
    for i in sample_indices[:3]:  # Show first 3
        pt_base = points_base[i]
        pt_cam_homo = T_base2cam @ np.append(pt_base, 1)
        pt_cam = pt_cam_homo[:3]
        
        print(f"\nPoint {i}:")
        print(f"  Base: [{pt_base[0]:6.3f}, {pt_base[1]:6.3f}, {pt_base[2]:6.3f}]")
        print(f"  Cam:  [{pt_cam[0]:6.3f}, {pt_cam[1]:6.3f}, {pt_cam[2]:6.3f}]")
        print(f"  Z > 0? {pt_cam[2] > 0}")
    
    # Transform all points
    pts_homo = np.hstack([points_base, np.ones((len(points_base), 1))])
    pts_cam = (T_base2cam @ pts_homo.T).T[:, :3]
    
    valid_z = pts_cam[:, 2] > 0.01
    print(f"\n✓ Points with Z > 0: {np.sum(valid_z)}/{len(points_base)}")
    
    if np.sum(valid_z) == 0:
        print("\n❌ No points in front of camera!")
        print("   This means the calibration or point cloud is incorrect.")
        return
    
    # 4. Project to image
    print("\n[4/4] Projecting to image...")
    camera = RealSenseImage(SN_number="046322250624")
    K = camera.o3d_intrinsics.intrinsic_matrix
    camera.close()
    
    print(f"\nCamera intrinsics K:")
    print(K)
    
    # Project valid points
    pts_cam_valid = pts_cam[valid_z]
    pts_2d = (K @ pts_cam_valid.T).T
    pts_2d = pts_2d / pts_2d[:, 2:3]  # Normalize
    
    us = pts_2d[:, 0]
    vs = pts_2d[:, 1]
    
    print(f"\nProjected points range:")
    print(f"  u: {us.min():.1f} ~ {us.max():.1f}")
    print(f"  v: {vs.min():.1f} ~ {vs.max():.1f}")
    
    # Check how many are in image bounds
    img_width, img_height = 640, 480
    in_bounds = (us >= 0) & (us < img_width) & (vs >= 0) & (vs < img_height)
    print(f"\n✓ Points in image bounds: {np.sum(in_bounds)}/{len(us)}")
    
    # Visualize projection
    print("\n[5/5] Creating visualization...")
    img = cv2.imread('captured_color.png')
    
    # Draw all projected points
    for u, v in zip(us[in_bounds], vs[in_bounds]):
        cv2.circle(img, (int(u), int(v)), 1, (0, 255, 0), -1)
    
    cv2.imwrite('projection_debug.png', img)
    print("✓ Saved projection_debug.png")
    print("  Check this image to see where points are projected")
    
    # Show with OpenCV
    cv2.imshow('Projection Debug', img)
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    debug_projection()

