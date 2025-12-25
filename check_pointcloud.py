#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Check point cloud coordinate systems
检查点云坐标系
"""

import numpy as np
import open3d as o3d

print("\n" + "="*60)
print("  Checking Point Cloud Coordinate Systems")
print("="*60 + "\n")

# Check original point cloud (camera frame)
print("[1/2] Original point cloud (camera frame):")
pcd_cam = o3d.io.read_point_cloud('garment_pointcloud.ply')
pts_cam = np.asarray(pcd_cam.points)

print(f"  Points: {len(pts_cam)}")
print(f"  X range: {pts_cam[:,0].min():7.3f} ~ {pts_cam[:,0].max():7.3f}")
print(f"  Y range: {pts_cam[:,1].min():7.3f} ~ {pts_cam[:,1].max():7.3f}")
print(f"  Z range: {pts_cam[:,2].min():7.3f} ~ {pts_cam[:,2].max():7.3f}")

# Check transformed point cloud (base frame)
print("\n[2/2] Transformed point cloud (base frame):")
pcd_base = o3d.io.read_point_cloud('garment_pointcloud_base.ply')
pts_base = np.asarray(pcd_base.points)

print(f"  Points: {len(pts_base)}")
print(f"  X range: {pts_base[:,0].min():7.3f} ~ {pts_base[:,0].max():7.3f}")
print(f"  Y range: {pts_base[:,1].min():7.3f} ~ {pts_base[:,1].max():7.3f}")
print(f"  Z range: {pts_base[:,2].min():7.3f} ~ {pts_base[:,2].max():7.3f}")

print("\n" + "="*60)
print("  Analysis")
print("="*60)

print("\nCamera frame (original):")
print("  - Z should be NEGATIVE (RealSense convention: Z points into scene)")
print(f"  - Your Z: {pts_cam[:,2].min():.3f} ~ {pts_cam[:,2].max():.3f}")
if pts_cam[:,2].max() < 0:
    print("  ✓ Correct! Points are in front of camera")
else:
    print("  ⚠ Unexpected! Z should be negative")

print("\nBase frame (transformed):")
print("  - Z should be POSITIVE (points above base)")
print(f"  - Your Z: {pts_base[:,2].min():.3f} ~ {pts_base[:,2].max():.3f}")
if pts_base[:,2].min() > 0:
    print("  ✓ Correct! Points are above base")
else:
    print("  ⚠ Unexpected! Z should be positive")

print("\n" + "="*60)
print("  Key Finding")
print("="*60)
print("\nThe issue is: when projecting back to image, we need to use")
print("the ORIGINAL camera frame point cloud, not the transformed one!")
print("\nFor bbox to 3D mapping:")
print("  1. Use garment_pointcloud.ply (camera frame)")
print("  2. Project directly to image (already in camera frame)")
print("  3. For each bbox, get corresponding 3D points")
print("  4. THEN transform those points to base frame")
print("="*60 + "\n")

