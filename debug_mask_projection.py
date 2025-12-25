#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug mask and projection mismatch
调试mask和投影不匹配的问题
"""

import cv2
import numpy as np
import open3d as o3d
from robot_libs.realsense_image_module import RealSenseImage
from bbox_to_3d import project_pcd_to_image


def debug_mask_projection():
    print("\n" + "="*60)
    print("  Debugging Mask and Projection Mismatch")
    print("="*60 + "\n")
    
    # 1. Load mask
    print("[1/5] Loading mask...")
    mask = cv2.imread('garment_mask.png', cv2.IMREAD_GRAYSCALE)
    mask_bool = mask > 127
    print(f"✓ Mask shape: {mask.shape}")
    print(f"  Mask pixels: {np.sum(mask_bool)} / {mask.size}")
    
    # 2. Load RGB
    print("\n[2/5] Loading RGB image...")
    rgb = cv2.imread('captured_color.png')
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    print(f"✓ RGB shape: {rgb.shape}")
    
    # Check if mask and RGB match
    if mask.shape != rgb.shape[:2]:
        print(f"❌ ERROR: Mask shape {mask.shape} != RGB shape {rgb.shape[:2]}")
        return
    
    # 3. Load point cloud
    print("\n[3/5] Loading point cloud...")
    pcd = o3d.io.read_point_cloud('garment_pointcloud.ply')
    points = np.asarray(pcd.points)
    print(f"✓ Point cloud: {len(points)} points")
    
    # 4. Get camera intrinsics
    print("\n[4/5] Getting camera intrinsics...")
    camera = RealSenseImage(SN_number="046322250624")
    K = camera.o3d_intrinsics.intrinsic_matrix
    camera.close()
    print(f"✓ Camera intrinsics:")
    print(K)
    
    # 5. Project point cloud
    print("\n[5/5] Projecting and comparing...")
    us, vs, valid = project_pcd_to_image(points, K)
    
    # Filter to image bounds
    h, w = rgb.shape[:2]
    in_bounds = (
        valid &
        (us >= 0) & (us < w) &
        (vs >= 0) & (vs < h)
    )
    
    print(f"✓ Valid points: {np.sum(valid)}/{len(points)}")
    print(f"✓ In bounds: {np.sum(in_bounds)}/{np.sum(valid)}")
    
    # Create visualization comparing mask and projection
    vis_mask = np.zeros((h, w, 3), dtype=np.uint8)
    vis_proj = np.zeros((h, w, 3), dtype=np.uint8)
    vis_both = rgb.copy()
    
    # Mask in blue
    vis_mask[mask_bool] = [0, 0, 255]
    vis_both[mask_bool] = vis_both[mask_bool] * 0.5 + np.array([0, 0, 255]) * 0.5
    
    # Projection in green
    for u, v in zip(us[in_bounds], vs[in_bounds]):
        u_int, v_int = int(round(u)), int(round(v))
        if 0 <= u_int < w and 0 <= v_int < h:
            vis_proj[v_int, u_int] = [0, 255, 0]
            vis_both[v_int, u_int] = [0, 255, 0]
    
    # Calculate overlap
    mask_pixels = set()
    for v in range(h):
        for u in range(w):
            if mask_bool[v, u]:
                mask_pixels.add((u, v))
    
    proj_pixels = set()
    for u, v in zip(us[in_bounds], vs[in_bounds]):
        u_int, v_int = int(round(u)), int(round(v))
        if 0 <= u_int < w and 0 <= v_int < h:
            proj_pixels.add((u_int, v_int))
    
    overlap = mask_pixels & proj_pixels
    print(f"\n[Statistics]")
    print(f"  Mask pixels:       {len(mask_pixels)}")
    print(f"  Projected pixels:  {len(proj_pixels)}")
    print(f"  Overlap pixels:    {len(overlap)}")
    print(f"  Overlap ratio:     {len(overlap) / len(mask_pixels) * 100:.1f}%")
    
    # Save comparison images
    cv2.imwrite('debug_mask_only.png', cv2.cvtColor(vis_mask, cv2.COLOR_RGB2BGR))
    cv2.imwrite('debug_proj_only.png', cv2.cvtColor(vis_proj, cv2.COLOR_RGB2BGR))
    cv2.imwrite('debug_both.png', cv2.cvtColor(vis_both, cv2.COLOR_RGB2BGR))
    
    print(f"\n✓ Saved debug images:")
    print(f"  - debug_mask_only.png (blue = SAM mask)")
    print(f"  - debug_proj_only.png (green = projected points)")
    print(f"  - debug_both.png (overlay)")
    
    # Display
    cv2.namedWindow("Mask (blue) vs Projection (green)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Mask (blue) vs Projection (green)", 1000, 750)
    cv2.imshow("Mask (blue) vs Projection (green)", cv2.cvtColor(vis_both, cv2.COLOR_RGB2BGR))
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Diagnosis
    print("\n" + "="*60)
    print("  Diagnosis")
    print("="*60)
    
    if len(overlap) / len(mask_pixels) > 0.8:
        print("✓ Good match! Mask and projection align well.")
    elif len(overlap) / len(mask_pixels) > 0.5:
        print("⚠ Partial match. Some misalignment detected.")
        print("  Possible causes:")
        print("  - Mask includes areas without depth data")
        print("  - SAM over-segmented the garment")
    else:
        print("❌ Poor match! Significant misalignment.")
        print("  This indicates a serious problem:")
        print("  - RGB and depth may not be aligned properly")
        print("  - Camera intrinsics may be wrong")
        print("  - Point cloud generation has issues")
    
    print("="*60 + "\n")


if __name__ == "__main__":
    import os
    if not os.path.exists('garment_mask.png'):
        print("❌ garment_mask.png not found")
        exit(1)
    if not os.path.exists('captured_color.png'):
        print("❌ captured_color.png not found")
        exit(1)
    if not os.path.exists('garment_pointcloud.ply'):
        print("❌ garment_pointcloud.ply not found")
        exit(1)
    
    debug_mask_projection()

