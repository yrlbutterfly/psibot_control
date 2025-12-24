#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时点云捕获测试
从相机实时获取RGB-D数据并生成点云
"""

import cv2
import open3d as o3d
import numpy as np
from robot_libs.realsense_image_module import RealSenseImage, generate_pcd

def main():
    print("="*60)
    print("实时点云捕获测试")
    print("="*60)
    
    # 初始化相机
    camera_sn = "046322250624"  # 你的相机序列号
    print(f"\n初始化相机 {camera_sn}...")
    camera = RealSenseImage(SN_number=camera_sn)
    
    # 获取相机内参
    intrinsics = camera.o3d_intrinsics
    print(f"✓ 相机内参: {intrinsics.width}x{intrinsics.height}")
    print(f"  fx={intrinsics.intrinsic_matrix[0,0]:.2f}, fy={intrinsics.intrinsic_matrix[1,1]:.2f}")
    
    # 实时捕获
    print("\n正在捕获当前场景...")
    color_img, depth_img = camera.capture_frame()
    
    print(f"✓ RGB图像: {color_img.shape}, dtype={color_img.dtype}")
    print(f"✓ 深度图: {depth_img.shape}, dtype={depth_img.dtype}")
    
    # 检查深度图数据
    valid_depth = depth_img[depth_img > 0]
    if len(valid_depth) > 0:
        print(f"\n深度统计:")
        print(f"  有效深度点: {len(valid_depth)}/{depth_img.size} ({len(valid_depth)/depth_img.size*100:.1f}%)")
        print(f"  深度范围: {valid_depth.min():.0f} - {valid_depth.max():.0f} mm")
        print(f"  平均深度: {valid_depth.mean():.0f} mm ({valid_depth.mean()/1000:.2f} m)")
    else:
        print("⚠️ 警告: 没有有效的深度数据!")
    
    # 保存当前捕获的图像（用于调试）
    print("\n保存当前捕获的图像...")
    cv2.imwrite("captured_color.png", cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))
    cv2.imwrite("captured_depth.png", depth_img)
    print("✓ 已保存: captured_color.png, captured_depth.png")
    
    # 生成点云
    print("\n生成点云...")
    pcd = generate_pcd(color_img, depth_img, intrinsics, visualize_flag=False)
    
    # 检查点云
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    print(f"✓ 点云生成完成:")
    print(f"  总点数: {len(points)}")
    
    if len(points) > 0:
        print(f"  X范围: {points[:,0].min():.3f} ~ {points[:,0].max():.3f} m")
        print(f"  Y范围: {points[:,1].min():.3f} ~ {points[:,1].max():.3f} m")
        print(f"  Z范围: {points[:,2].min():.3f} ~ {points[:,2].max():.3f} m")
        
        # 保存点云
        o3d.io.write_point_cloud("captured_pointcloud.ply", pcd)
        print("✓ 已保存: captured_pointcloud.ply")
        
        # 可视化
        print("\n打开3D可视化窗口...")
        print("提示:")
        print("  - 鼠标左键拖动: 旋转")
        print("  - 鼠标右键拖动: 平移")
        print("  - 滚轮: 缩放")
        print("  - 按 'Q' 或关闭窗口退出")
        
        # 添加坐标系参考
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        
        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name="实时点云",
            width=1024,
            height=768,
            left=50,
            top=50
        )
    else:
        print("❌ 错误: 点云为空!")
        print("可能的原因:")
        print("  1. 深度相机没有获取到有效数据")
        print("  2. 场景距离太近或太远")
        print("  3. 光照条件不好")
    
    print("\n✓ 测试完成")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

