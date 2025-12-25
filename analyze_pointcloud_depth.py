#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze point cloud depth distribution
分析点云的深度分布，找出异常点
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from bbox_to_3d import project_pcd_to_image
from robot_libs.realsense_image_module import RealSenseImage
import cv2


def analyze_pointcloud_depth():
    """Analyze depth distribution of garment point cloud"""
    
    print("\n" + "="*60)
    print("  Point Cloud Depth Analysis")
    print("="*60 + "\n")
    
    # Load point cloud
    print("[1/3] Loading point cloud...")
    pcd = o3d.io.read_point_cloud('garment_pointcloud.ply')
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    print(f"✓ Loaded {len(points)} points")
    print(f"\n3D Statistics:")
    print(f"  X range: {points[:, 0].min():.3f} ~ {points[:, 0].max():.3f} m")
    print(f"  Y range: {points[:, 1].min():.3f} ~ {points[:, 1].max():.3f} m")
    print(f"  Z range: {points[:, 2].min():.3f} ~ {points[:, 2].max():.3f} m")
    
    # Analyze depth distribution
    depths = points[:, 2]
    print(f"\nDepth Statistics:")
    print(f"  Mean: {depths.mean():.3f} m")
    print(f"  Median: {np.median(depths):.3f} m")
    print(f"  Std: {depths.std():.3f} m")
    print(f"  25th percentile: {np.percentile(depths, 25):.3f} m")
    print(f"  75th percentile: {np.percentile(depths, 75):.3f} m")
    print(f"  95th percentile: {np.percentile(depths, 95):.3f} m")
    
    # Plot depth histogram
    print("\n[2/3] Plotting depth histogram...")
    plt.figure(figsize=(12, 4))
    
    plt.subplot(1, 2, 1)
    plt.hist(depths, bins=50, edgecolor='black', alpha=0.7)
    plt.axvline(np.median(depths), color='r', linestyle='--', label=f'Median: {np.median(depths):.3f}m')
    plt.axvline(np.percentile(depths, 95), color='orange', linestyle='--', label=f'95th: {np.percentile(depths, 95):.3f}m')
    plt.xlabel('Depth (m)')
    plt.ylabel('Count')
    plt.title('Depth Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Cumulative distribution
    plt.subplot(1, 2, 2)
    sorted_depths = np.sort(depths)
    cumulative = np.arange(1, len(sorted_depths) + 1) / len(sorted_depths) * 100
    plt.plot(sorted_depths, cumulative)
    plt.axhline(95, color='orange', linestyle='--', alpha=0.5)
    plt.axvline(np.percentile(depths, 95), color='orange', linestyle='--', 
                label=f'95th percentile: {np.percentile(depths, 95):.3f}m')
    plt.xlabel('Depth (m)')
    plt.ylabel('Cumulative %')
    plt.title('Cumulative Depth Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('depth_distribution.png', dpi=150)
    print("✓ Saved depth_distribution.png")
    
    # Project to image and visualize by depth
    print("\n[3/3] Creating depth-colored overlay...")
    rgb_image = cv2.imread('captured_color.png')
    camera = RealSenseImage(SN_number="046322250624")
    K = camera.o3d_intrinsics.intrinsic_matrix
    camera.close()
    
    us, vs, valid = project_pcd_to_image(points, K)
    
    img_h, img_w = rgb_image.shape[:2]
    in_bounds = (
        valid &
        (us >= 0) & (us < img_w) &
        (vs >= 0) & (vs < img_h)
    )
    
    # Create depth-colored overlay
    overlay = rgb_image.copy()
    
    # Normalize depths for coloring
    valid_depths = depths[in_bounds]
    depth_min, depth_max = valid_depths.min(), valid_depths.max()
    
    # Color by depth: blue (close) to red (far)
    for u, v, d in zip(us[in_bounds], vs[in_bounds], valid_depths):
        # Normalize depth to [0, 1]
        norm_d = (d - depth_min) / (depth_max - depth_min)
        # Color: blue (0) -> green (0.5) -> red (1)
        if norm_d < 0.5:
            color = (int(255 * (1 - 2*norm_d)), int(255 * 2*norm_d), 255)  # Blue to Green
        else:
            color = (0, int(255 * (2 - 2*norm_d)), int(255 * (2*norm_d - 1)))  # Green to Red
        
        cv2.circle(overlay, (int(u), int(v)), 2, color, -1)
    
    # Blend
    alpha = 0.7
    blended = cv2.addWeighted(rgb_image, 1-alpha, overlay, alpha, 0)
    
    # Add legend
    cv2.putText(blended, f"Depth visualization (Blue=close, Red=far)", 
               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(blended, f"Min depth: {depth_min:.2f}m (blue)", 
               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(blended, f"Max depth: {depth_max:.2f}m (red)", 
               (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    cv2.imwrite('depth_colored_overlay.png', blended)
    print("✓ Saved depth_colored_overlay.png")
    
    # Suggest depth filter threshold
    median_depth = np.median(depths)
    depth_95th = np.percentile(depths, 95)
    
    print("\n" + "="*60)
    print("  Analysis & Recommendations")
    print("="*60)
    
    # Check if there are outliers
    if depth_max - median_depth > 0.5:  # More than 50cm difference
        print("\n⚠ WARNING: Large depth range detected!")
        print(f"  Median depth: {median_depth:.3f}m")
        print(f"  Max depth: {depth_max:.3f}m")
        print(f"  Difference: {depth_max - median_depth:.3f}m")
        print("\n💡 Recommendation:")
        print(f"  Consider filtering points with depth > {depth_95th:.3f}m")
        print(f"  This would remove {100 - 95:.0f}% of outliers")
        
        # Ask if user wants to create filtered point cloud
        print("\n" + "-"*60)
        response = input("Create filtered point cloud? (y/n): ")
        if response.lower() == 'y':
            # Filter outliers
            depth_threshold = depth_95th
            valid_points_mask = depths <= depth_threshold
            
            filtered_points = points[valid_points_mask]
            filtered_colors = colors[valid_points_mask]
            
            # Create filtered point cloud
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
            filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
            
            # Save
            o3d.io.write_point_cloud('garment_pointcloud_filtered.ply', filtered_pcd)
            
            print(f"\n✓ Filtered point cloud saved!")
            print(f"  Original: {len(points)} points")
            print(f"  Filtered: {len(filtered_points)} points")
            print(f"  Removed: {len(points) - len(filtered_points)} outliers")
            print(f"  Saved to: garment_pointcloud_filtered.ply")
            print("\n💡 Use this filtered point cloud in interactive_bbox_to_3d.py")
    else:
        print("\n✓ Depth distribution looks reasonable")
    
    print("\n" + "="*60 + "\n")
    
    # Show plots
    plt.show()


if __name__ == "__main__":
    import os
    if not os.path.exists('garment_pointcloud.ply'):
        print("❌ garment_pointcloud.ply not found")
        print("Please run get_garment_pointcloud.py first")
        exit(1)
    if not os.path.exists('captured_color.png'):
        print("❌ captured_color.png not found")
        print("Please run get_garment_pointcloud.py first")
        exit(1)
    
    analyze_pointcloud_depth()

