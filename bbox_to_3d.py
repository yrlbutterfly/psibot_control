#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 2D 边界框映射到 3D 点云坐标

本模块的主要功能：
1. 将3D点云投影到2D图像平面
2. 根据2D边界框提取对应的3D点
3. 计算边界框对应区域的3D质心位置
"""

import numpy as np


def project_pcd_to_image(pcd_points_cam, camera_intrinsics):
    """
    将相机坐标系的3D点云投影到图像平面
    
    功能说明：
    此函数处理 generate_pcd() 生成的点云坐标系统：
    - generate_pcd() 应用的变换矩阵: [[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]]
    - 保存的点云具有: Y轴翻转, Z轴翻转 (Z为负表示向前)
    - 我们需要撤销这个变换以进行标准的针孔相机投影
    
    参数:
        pcd_points_cam: (N, 3) 相机坐标系中的点 (已变换)
        camera_intrinsics: (3, 3) 相机内参矩阵
    
    返回:
        us: (N,) 像素u坐标（横坐标）
        vs: (N,) 像素v坐标（纵坐标）
        valid_mask: (N,) 布尔掩码，标记有效点（在相机前方）
    """
    # 检查点云是否在变换后的坐标系中 (Z为负)
    mean_z = np.mean(pcd_points_cam[:, 2])
    
    if mean_z < 0:
        # 点云在变换后的坐标系中，需要转换回标准坐标系以进行投影
        # 撤销变换：将Y和Z轴翻转回来
        pts_standard = pcd_points_cam.copy()
        pts_standard[:, 1] = -pts_standard[:, 1]  # 撤销Y轴翻转
        pts_standard[:, 2] = -pts_standard[:, 2]  # 撤销Z轴翻转
        
        # 变换系统的有效深度检查
        valid_depth = pcd_points_cam[:, 2] < -0.01  # Z为负表示向前
    else:
        # 已经是标准坐标系
        pts_standard = pcd_points_cam.copy()
        valid_depth = pcd_points_cam[:, 2] > 0.01  # Z为正表示向前
    
    # 标准针孔相机投影：[u, v, w]^T = K @ [X, Y, Z]^T
    # 避免除零错误
    pts_standard[:, 2] = np.where(np.abs(pts_standard[:, 2]) < 1e-6, 1e-6, pts_standard[:, 2])
    
    # 投影到图像平面
    pts_2d = (camera_intrinsics @ pts_standard.T).T  # (N, 3)
    pts_2d = pts_2d / pts_2d[:, 2:3]  # 用Z归一化
    
    # 提取像素坐标
    us = pts_2d[:, 0]  # 横坐标
    vs = pts_2d[:, 1]  # 纵坐标
    
    return us, vs, valid_depth


def get_3d_center_from_bbox(bbox, pcd_points_cam, us, vs, valid_mask, img_width, img_height, T_cam2base, sample_stride=3):
    """
    从2D边界框中提取对应的3D点并计算质心（使用最近邻匹配算法）
    
    算法流程（改进的鲁棒性版本）：
    1. 获取所有点云点的有效2D投影
    2. 对边界框内的每个像素（按步长采样），找到最近的点云2D投影
    3. 记录该最近点的3D坐标
    4. 计算所有选中3D点的质心（在相机坐标系中）
    5. 将质心转换到基座坐标系
    
    此方法对衣物边缘的稀疏点云更加鲁棒。
    
    参数:
        bbox: [x_min, y_min, x_max, y_max] 像素坐标系中的边界框
        pcd_points_cam: (N, 3) 相机坐标系中的点云
        us, vs: 所有点云点的投影像素坐标
        valid_mask: 有效投影的布尔掩码
        img_width, img_height: 图像尺寸
        T_cam2base: (4x4) 相机到基座的变换矩阵
        sample_stride: 边界框内像素采样的步长（默认3）
    
    返回:
        center_3d_base: (3,) 基座坐标系中的3D质心位置，如果未找到点则返回None
        center_3d_cam: (3,) 相机坐标系中的3D质心位置，如果未找到点则返回None
    """
    x_min, y_min, x_max, y_max = bbox
    
    # 筛选有效的点云投影
    valid_us = us[valid_mask]
    valid_vs = vs[valid_mask]
    valid_points_cam = pcd_points_cam[valid_mask]
    
    if len(valid_us) == 0:
        print("  警告：未找到有效的点云投影")
        return None
    
    # 构建有效点云投影的2D坐标数组 (N, 2)
    pcd_2d = np.column_stack([valid_us, valid_vs])
    
    # 在边界框内按步长采样像素
    bbox_pixels = []
    for u in range(int(x_min), int(x_max) + 1, sample_stride):
        for v in range(int(y_min), int(y_max) + 1, sample_stride):
            bbox_pixels.append([u, v])
    
    if len(bbox_pixels) == 0:
        print("  警告：边界框中未采样到像素")
        return None
    
    bbox_pixels = np.array(bbox_pixels)  # (M, 2) - M个采样像素
    
    # 对每个边界框像素，找到最近的点云2D投影
    # 计算成对距离：(M, N)，其中 M = 边界框像素数，N = 有效点云点数
    # 使用广播：bbox_pixels[:, None, :] 是 (M, 1, 2)，pcd_2d[None, :, :] 是 (1, N, 2)
    distances = np.linalg.norm(bbox_pixels[:, None, :] - pcd_2d[None, :, :], axis=2)  # (M, N)
    
    # 找到每个边界框像素的最近点
    nearest_indices = np.argmin(distances, axis=1)  # (M,) - 最近点的索引
    nearest_distances = np.min(distances, axis=1)  # (M,) - 到最近点的距离
    
    # 调试：检查最近邻距离是否合理
    print(f"  最近邻距离：最小={nearest_distances.min():.1f}px, "
          f"最大={nearest_distances.max():.1f}px, 平均={nearest_distances.mean():.1f}px")
    
    # ⚠️ 警告：检查点云覆盖情况
    if nearest_distances.mean() > 50:
        print(f"  ⚠️⚠️⚠️ 警告：平均最近邻距离过大({nearest_distances.mean():.1f}px)")
        print(f"  ⚠️ 这说明bbox区域几乎没有点云覆盖！")
        print(f"  ⚠️ 计算的3D位置可能严重偏离实际位置！")
    elif nearest_distances.mean() > 20:
        print(f"  ⚠️ 注意：平均最近邻距离较大({nearest_distances.mean():.1f}px)")
        print(f"  ⚠️ bbox区域点云稀疏，结果可能不准确")
    
    # 获取对应的3D点（保留重复以实现加权）
    # 注意：不要删除重复点 - 重复的点应该有更大的权重！
    selected_3d_points = valid_points_cam[nearest_indices]  # (M, 3)
    
    # 调试：统计唯一点的数量
    unique_count = len(np.unique(nearest_indices))
    print(f"  从{unique_count}个唯一点云点中选择了{len(selected_3d_points)}个点（包含重复）")
    
    # ⚠️ 深度层检查：如果选中的点远离主要点云，可能找错了层
    all_depths = valid_points_cam[:, 2]
    main_depth_median = np.median(all_depths)
    selected_depth_median = np.median(selected_3d_points[:, 2])
    depth_diff = abs(selected_depth_median - main_depth_median)
    
    if depth_diff > 0.5:  # 相差超过50cm
        print(f"  ⚠️⚠️⚠️ 严重警告：选中点的深度({selected_depth_median:.2f}m)远离主要点云({main_depth_median:.2f}m)")
        print(f"  ⚠️ 相差{depth_diff:.2f}m - 可能找到了背景/桌面的点！")
        print(f"  ⚠️ 请检查bbox区域是否有衣物点云覆盖！")
    elif depth_diff > 0.2:  # 相差20-50cm
        print(f"  ⚠️ 警告：选中点的深度({selected_depth_median:.2f}m)与主要点云({main_depth_median:.2f}m)有差异")
        print(f"  ⚠️ 相差{depth_diff:.2f}m - 请验证结果是否合理")
    
    # 基于深度使用MAD（中位数绝对偏差）过滤异常值（对异常值鲁棒）
    depths = selected_3d_points[:, 2]
    median_depth = np.median(depths)
    mad = np.median(np.abs(depths - median_depth))  # 中位数绝对偏差
    
    # 基于MAD使用自适应阈值
    # 如果MAD非常小，使用最小阈值以避免过度过滤
    if mad < 0.01:  # 非常紧密的聚类（< 1cm变化）
        tolerance = 0.05  # 至少5cm容差
    elif mad < 0.1:  # 中等聚类
        tolerance = max(3 * mad, 0.05)  # 3倍MAD但至少5cm
    else:  # 分散的聚类
        tolerance = max(3 * mad, 0.15)  # 3倍MAD但至少15cm
    
    # 根据深度过滤点
    depth_filter = np.abs(depths - median_depth) <= tolerance
    filtered_3d_points = selected_3d_points[depth_filter]
    
    if len(filtered_3d_points) == 0:
        print(f"  警告：所有点都被过滤！使用未过滤的点。")
        filtered_3d_points = selected_3d_points
    
    # 打印过滤统计信息
    num_outliers = len(selected_3d_points) - len(filtered_3d_points)
    if num_outliers > 0:
        print(f"  过滤了{num_outliers}个异常值（MAD={mad:.3f}m，容差={tolerance:.3f}m）")
        print(f"  深度范围：{depths.min():.3f}->{filtered_3d_points[:, 2].min():.3f} 到 "
              f"{depths.max():.3f}->{filtered_3d_points[:, 2].max():.3f}m")
    
    # 步骤1：计算加权质心（通过重复加权）
    centroid_cam = filtered_3d_points.mean(axis=0)
    
    print(f"  计算的质心：({centroid_cam[0]:.3f}, {centroid_cam[1]:.3f}, {centroid_cam[2]:.3f})m")
    
    # 步骤2：在原始点云中找到距离此质心最近的点
    # 这确保最终结果是衣物上的实际点，而不是插值点
    distances_to_centroid = np.linalg.norm(valid_points_cam - centroid_cam, axis=1)
    closest_idx = np.argmin(distances_to_centroid)
    closest_point_cam = valid_points_cam[closest_idx]
    
    print(f"  最近点到质心：({closest_point_cam[0]:.3f}, {closest_point_cam[1]:.3f}, {closest_point_cam[2]:.3f})m")
    print(f"  到质心的距离：{distances_to_centroid[closest_idx]:.3f}m")
    
    print(f"  边界框：[{x_min:.0f}, {y_min:.0f}, {x_max:.0f}, {y_max:.0f}]")
    print(f"  采样了{len(bbox_pixels)}个像素 -> {len(filtered_3d_points)}个加权点 -> 1个最终点")
    
    # 将最近点转换到基座坐标系
    center_homo = np.append(closest_point_cam, 1)  # 转换为齐次坐标
    center_3d_base = (T_cam2base @ center_homo)[:3]  # 应用变换矩阵
    
    # Debug: Print transformation details
    print(f"\n  [坐标转换调试]")
    print(f"  相机坐标 (变换后): ({closest_point_cam[0]:.3f}, {closest_point_cam[1]:.3f}, {closest_point_cam[2]:.3f})m")
    print(f"  T_cam2base平移: ({T_cam2base[0,3]:.3f}, {T_cam2base[1,3]:.3f}, {T_cam2base[2,3]:.3f})m")
    print(f"  基座坐标: ({center_3d_base[0]:.3f}, {center_3d_base[1]:.3f}, {center_3d_base[2]:.3f})m")
    print(f"  ⚠️  如果基座Z={center_3d_base[2]:.3f}m看起来不对，可能需要重新标定")
    
    # Return both camera frame and base frame positions
    return center_3d_base, closest_point_cam


def map_bboxes_to_3d(bboxes_dict, pcd_points_cam, camera_intrinsics, T_cam2base, img_width, img_height):
    """
    将多个标注的边界框映射到3D位置（基座坐标系）
    
    功能说明：
    此函数批量处理多个边界框，为每个边界框计算对应的3D位置。
    所有点云只投影一次，然后复用投影结果，提高效率。
    
    参数:
        bboxes_dict: 边界框字典，键为标签，值为边界框坐标 [x1,y1,x2,y2]
        pcd_points_cam: (N, 3) 相机坐标系中的点云
        camera_intrinsics: (3, 3) 相机内参矩阵
        T_cam2base: (4, 4) 相机到基座的变换矩阵
        img_width, img_height: 图像尺寸
    
    返回:
        positions_3d_base: 3D位置字典，键为标签，值为基座坐标系中的3D坐标 [x,y,z]
    """
    # 只投影一次所有点（提高效率）
    us, vs, valid_mask = project_pcd_to_image(pcd_points_cam, camera_intrinsics)
    
    positions_3d_base = {}
    
    # 遍历每个边界框
    for label, bbox in bboxes_dict.items():
        print(f"\n处理边界框: {label}")
        result = get_3d_center_from_bbox(bbox, pcd_points_cam, us, vs, valid_mask, 
                                         img_width, img_height, T_cam2base)
        
        if result is not None:
            center_base, center_cam = result
            positions_3d_base[label] = center_base
            print(f"  {label}: ({center_base[0]:.3f}, {center_base[1]:.3f}, {center_base[2]:.3f}) m [基座坐标系]")
        else:
            print(f"  {label}: 边界框中未找到点")
            positions_3d_base[label] = None
    
    return positions_3d_base


if __name__ == "__main__":
    # 简单测试（使用虚拟数据）
    print("测试 bbox_to_3d 模块...")
    
    # 加载标定文件
    import os
    if os.path.exists('calibration_results/camera_calibration_right_arm_20251222-222131.npz'):
        calib = np.load('calibration_results/camera_calibration_right_arm_20251222-222131.npz')
        T_cam2base = calib['T_cam2base']
        print("✓ 标定文件已加载")
    else:
        print("✗ 未找到标定文件")
        exit(1)
    
    # 虚拟相机内参（请替换为真实值）
    camera_intrinsics = np.array([
        [900, 0, 640],
        [0, 900, 360],
        [0, 0, 1]
    ])
    
    # 如果可用，使用真实点云进行测试
    try:
        import open3d as o3d
        has_o3d = True
    except ImportError:
        has_o3d = False
    
    if has_o3d and os.path.exists('garment_pointcloud_base.ply'):
        print("\n✓ 找到真实点云，使用它进行测试...")
        pcd = o3d.io.read_point_cloud('garment_pointcloud_base.ply')
        pcd_points = np.asarray(pcd.points)
        print(f"  加载了 {len(pcd_points)} 个点")
    else:
        print("\n⚠ 未找到真实点云，创建虚拟数据...")
        # 根据标定创建合理位置的虚拟点云
        # 典型衣物位置：X~0.2, Y~0.0, Z~1.5
        pcd_points = np.array([
            [0.2, 0.0, 1.5],
            [0.25, 0.05, 1.5],
            [0.15, -0.05, 1.5],
            [0.3, 0.1, 1.6],
        ])
        print(f"  创建了 {len(pcd_points)} 个虚拟点")
    
    # 投影到图像
    us, vs, valid = project_pcd_to_image(pcd_points, camera_intrinsics, T_cam2base)
    print(f"\n✓ 投影了 {np.sum(valid)}/{len(pcd_points)} 个有效点")
    
    if np.sum(valid) > 0:
        print(f"  u 范围: {us[valid].min():.1f} - {us[valid].max():.1f}")
        print(f"  v 范围: {vs[valid].min():.1f} - {vs[valid].max():.1f}")
    else:
        print("  ⚠ 没有有效点被投影（所有点都在相机后方或视野外）")
        print("  这对模块测试来说是正常的，但请检查你的标定用于实际使用。")
    
    # 测试边界框映射
    if np.sum(valid) > 0:
        # 在投影点周围创建边界框
        bboxes = {
            "test_area": [us[valid].min() - 10, vs[valid].min() - 10, 
                         us[valid].max() + 10, vs[valid].max() + 10]
        }
        
        print("\n测试边界框到3D映射...")
        positions = map_bboxes_to_3d(bboxes, pcd_points, camera_intrinsics, T_cam2base, 1280, 720)
        
        if positions["test_area"] is not None:
            print(f"\n✓ 成功计算3D中心")
        else:
            print(f"\n⚠ 计算3D中心失败")
    else:
        print("\n⚠ 跳过边界框测试（没有有效点）")
    
    print("\n✓ bbox_to_3d 模块测试完成")
