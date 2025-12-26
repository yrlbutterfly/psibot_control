#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试点云坐标转换
验证相机坐标系 → 机械臂基座坐标系的转换
"""

import numpy as np
import open3d as o3d
import os

def test_point_transform():
    print("\n" + "="*60)
    print("  点云坐标转换测试")
    print("="*60 + "\n")
    
    # 1. Load calibration matrix
    print("[1/4] 加载标定矩阵...")
    calib_file = 'calibration_results/camera_calibration_right_arm_20251226-234855.npz'
    
    if not os.path.exists(calib_file):
        print(f"❌ 标定文件不存在: {calib_file}")
        return
    
    calib = np.load(calib_file)
    T_cam2base = calib['T_cam2base']
    
    print("✓ 标定矩阵加载成功")
    
    # 2. Load point cloud
    print("\n[2/4] 加载点云...")
    pcd_file = 'garment_pointcloud.ply'
    
    if not os.path.exists(pcd_file):
        print(f"❌ 点云文件不存在: {pcd_file}")
        print("   请先运行 get_garment_pointcloud.py 获取点云")
        return
    
    pcd_cam = o3d.io.read_point_cloud(pcd_file)
    points_cam = np.asarray(pcd_cam.points)
    
    print(f"✓ 点云加载成功，共 {len(points_cam)} 个点")
    print("\n相机坐标系下的点云范围:")
    print(f"  X: {points_cam[:,0].min():7.3f} ~ {points_cam[:,0].max():7.3f} 米")
    print(f"  Y: {points_cam[:,1].min():7.3f} ~ {points_cam[:,1].max():7.3f} 米")
    print(f"  Z: {points_cam[:,2].min():7.3f} ~ {points_cam[:,2].max():7.3f} 米")
    
    # 3. Transform to base frame
    print("\n[3/4] 转换到基座坐标系...")
    pcd_base = o3d.geometry.PointCloud(pcd_cam)
    pcd_base.transform(T_cam2base)
    
    points_base = np.asarray(pcd_base.points)
    
    print("✓ 转换完成")
    print("\n基座坐标系下的点云范围:")
    print(f"  X: {points_base[:,0].min():7.3f} ~ {points_base[:,0].max():7.3f} 米")
    print(f"  Y: {points_base[:,1].min():7.3f} ~ {points_base[:,1].max():7.3f} 米")
    print(f"  Z: {points_base[:,2].min():7.3f} ~ {points_base[:,2].max():7.3f} 米")
    print(f"\n质心位置: ({points_base[:,0].mean():.3f}, {points_base[:,1].mean():.3f}, {points_base[:,2].mean():.3f})")
    
    # 4. Save transformed point cloud
    output_file = 'garment_pointcloud_base.ply'
    o3d.io.write_point_cloud(output_file, pcd_base)
    print(f"\n✓ 转换后的点云已保存到: {output_file}")
    
    print("\n" + "="*60)
    print("  ✓ 转换完成")
    print("="*60 + "\n")

if __name__ == "__main__":
    test_point_transform()

