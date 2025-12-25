#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取衣物点云的脚本
结合 SAM 分割 + RealSense 深度图生成衣物的3D点云
"""

import cv2
import numpy as np
import open3d as o3d
from robot_libs.realsense_image_module import RealSenseImage, generate_pcd
import os
import torch


class GarmentPointCloudExtractor:
    """Extract garment point cloud using SAM 2 segmentation and depth data"""
    
    def __init__(self, camera_sn, sam_checkpoint="sam2.1_hiera_large.pt"):
        # Initialize camera
        self.camera = RealSenseImage(SN_number=camera_sn)
        self.intrinsics = self.camera.o3d_intrinsics
        
        # Initialize SAM 2 model
        print("[INFO] Loading SAM 2.1 model...")
        if not os.path.exists(sam_checkpoint):
            raise FileNotFoundError(f"SAM checkpoint not found: {sam_checkpoint}\n"
                                    "Please download from: https://github.com/facebookresearch/segment-anything-2")
        
        # Determine device
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[INFO] Using device: {device}")
        
        # Build SAM 2 model (now using installed package)
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        
        # Use relative config name for installed SAM 2 package
        model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
        sam2_model = build_sam2(model_cfg, sam_checkpoint, device=device)
        self.predictor = SAM2ImagePredictor(sam2_model)
        print("[INFO] SAM 2.1 model loaded successfully")
    
    def capture_frames(self):
        """Capture RGB and depth frames from camera"""
        color_img, depth_img = self.camera.capture_frame()
        return color_img, depth_img
    
    def interactive_segment(self, color_img):
        """
        Interactive segmentation: user clicks on garment to get mask
        Returns: binary mask (H x W) where True = garment pixels
        """
        clicked_points = []
        clicked_labels = []  # 1 for positive, 0 for negative
        
        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:  # Left click: add positive point
                clicked_points.append((x, y))
                clicked_labels.append(1)
                print(f"[+] Added positive point at ({x}, {y})")
            elif event == cv2.EVENT_RBUTTONDOWN:  # Right click: add negative point
                clicked_points.append((x, y))
                clicked_labels.append(0)
                print(f"[-] Added negative point at ({x}, {y})")
        
        print("\n" + "="*50)
        print("Interactive Segmentation Instructions:")
        print("  - Left Click: Add positive point (on garment)")
        print("  - Right Click: Add negative point (background)")
        print("  - Press SPACE: Confirm selection")
        print("  - Press ESC: Cancel")
        print("="*50 + "\n")
        
        # Set up display window
        display_img = color_img.copy()
        cv2.namedWindow("Select Garment", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Select Garment", 800, 600)
        cv2.setMouseCallback("Select Garment", on_mouse)
        
        mask = None
        
        while True:
            # Draw clicked points
            temp_img = display_img.copy()
            for i, (pt, label) in enumerate(zip(clicked_points, clicked_labels)):
                color = (0, 255, 0) if label == 1 else (0, 0, 255)
                cv2.circle(temp_img, pt, 5, color, -1)
                cv2.putText(temp_img, str(i+1), (pt[0]+10, pt[1]-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # If we have points, run SAM prediction
            if clicked_points:
                self.predictor.set_image(color_img)
                input_points = np.array(clicked_points)
                input_labels = np.array(clicked_labels)
                
                masks, scores, _ = self.predictor.predict(
                    point_coords=input_points,
                    point_labels=input_labels,
                    multimask_output=False
                )
                mask = masks[0]
                
                # Overlay mask on image (ensure mask is boolean)
                mask_bool = mask.astype(bool)
                mask_overlay = temp_img.copy()
                mask_overlay[mask_bool] = mask_overlay[mask_bool] * 0.5 + np.array([0, 255, 0]) * 0.5
                temp_img = mask_overlay.astype(np.uint8)
            
            cv2.imshow("Select Garment", temp_img)
            key = cv2.waitKey(10) & 0xFF
            
            if key == 27:  # ESC: cancel
                cv2.destroyAllWindows()
                return None
            elif key == 32:  # SPACE: confirm
                if mask is not None:
                    cv2.destroyAllWindows()
                    return mask
                else:
                    print("[WARNING] Please click on the garment first!")
        
        cv2.destroyAllWindows()
        return mask
    
    def mask_to_pointcloud(self, color_img, depth_img, mask):
        """
        Convert masked RGB-D image to 3D point cloud
        
        Args:
            color_img: RGB image (H x W x 3)
            depth_img: Depth image (H x W) in mm
            mask: Binary mask (H x W)
        
        Returns:
            garment_pcd: Open3D PointCloud object (in camera coordinate system)
            garment_points: Numpy array (N x 3) in meters
        """
        # Ensure mask is boolean
        mask = mask.astype(bool)
        
        print(f"[DEBUG] Mask size: {np.sum(mask)} pixels")
        print(f"[DEBUG] Depth in mask range: {depth_img[mask].min()}-{depth_img[mask].max()} mm")
        
        # Apply mask to depth image (set non-garment areas to 0)
        masked_depth = depth_img.copy()
        masked_depth[~mask] = 0
        
        # Generate point cloud only for masked region
        masked_pcd = generate_pcd(color_img, masked_depth, self.intrinsics, visualize_flag=False)
        
        # Convert to numpy arrays
        garment_points = np.asarray(masked_pcd.points)
        garment_colors = np.asarray(masked_pcd.colors)
        
        print(f"[DEBUG] Generated {len(garment_points)} raw points")
        if len(garment_points) > 0:
            print(f"[DEBUG] Point cloud Z range: {garment_points[:, 2].min():.3f} - {garment_points[:, 2].max():.3f} m")
        
        # Step 1: Remove invalid points (near-zero depth)
        valid = np.abs(garment_points[:, 2]) > 0.01
        garment_points = garment_points[valid]
        garment_colors = garment_colors[valid]
        print(f"[DEBUG] After basic filtering: {len(garment_points)} points")
        
        # Step 2: Remove outliers based on depth clustering
        # This filters out background/table points that are far from the main garment
        if len(garment_points) > 100:  # Only if we have enough points
            depths = np.abs(garment_points[:, 2])
            
            # Use median and MAD (robust to outliers)
            median_depth = np.median(depths)
            mad = np.median(np.abs(depths - median_depth))
            
            print(f"[DEBUG] Depth statistics: median={median_depth:.3f}m, MAD={mad:.3f}m")
            
            # Adaptive threshold based on MAD
            if mad < 0.05:  # Very tight cluster
                tolerance = 0.3  # Allow ±30cm from median
            else:
                tolerance = max(3 * mad, 0.3)  # 3*MAD but at least 30cm
            
            # Filter points by depth
            depth_filter = np.abs(depths - median_depth) <= tolerance
            filtered_points = garment_points[depth_filter]
            filtered_colors = garment_colors[depth_filter]
            
            num_removed = len(garment_points) - len(filtered_points)
            if num_removed > 0:
                print(f"[INFO] Removed {num_removed} depth outliers (tolerance=±{tolerance:.3f}m)")
                print(f"[INFO] Depth range: {depths.min():.3f}m -> {filtered_points[:, 2].min():.3f}m to")
                print(f"                    {depths.max():.3f}m -> {filtered_points[:, 2].max():.3f}m")
                garment_points = filtered_points
                garment_colors = filtered_colors
            else:
                print(f"[INFO] No depth outliers detected (all within ±{tolerance:.3f}m)")
        
        print(f"[DEBUG] After outlier filtering: {len(garment_points)} points")
        
        # Step 3: Statistical outlier removal (removes isolated points)
        # This removes scattered points that are far from neighbors
        if len(garment_points) > 50:
            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector(garment_points)
            temp_pcd.colors = o3d.utility.Vector3dVector(garment_colors)
            
            # Remove statistical outliers
            # nb_neighbors: number of neighbors to consider
            # std_ratio: standard deviation ratio (lower = more aggressive)
            cl, ind = temp_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            
            garment_points = np.asarray(cl.points)
            garment_colors = np.asarray(cl.colors)
            
            num_removed = len(temp_pcd.points) - len(garment_points)
            if num_removed > 0:
                print(f"[INFO] Removed {num_removed} spatial outliers (isolated points)")
        
        print(f"[DEBUG] Final point count: {len(garment_points)} points")
        
        # Create filtered point cloud
        garment_pcd = o3d.geometry.PointCloud()
        garment_pcd.points = o3d.utility.Vector3dVector(garment_points)
        garment_pcd.colors = o3d.utility.Vector3dVector(garment_colors)
        
        print(f"[INFO] Extracted {len(garment_points)} points from garment")
        return garment_pcd, garment_points
    
    def visualize_pointcloud(self, pcd):
        """Visualize point cloud in Open3D viewer"""
        # Get point cloud bounds for coordinate frame sizing
        points = np.asarray(pcd.points)
        if len(points) == 0:
            print("[WARNING] No points to visualize!")
            return
        
        # Print point cloud statistics
        print(f"\n[INFO] Point Cloud Statistics (Camera Frame):")
        print(f"  X range: {points[:, 0].min():.3f} ~ {points[:, 0].max():.3f} m")
        print(f"  Y range: {points[:, 1].min():.3f} ~ {points[:, 1].max():.3f} m")
        print(f"  Z range: {points[:, 2].min():.3f} ~ {points[:, 2].max():.3f} m")
        print(f"  Center: ({points[:, 0].mean():.3f}, {points[:, 1].mean():.3f}, {points[:, 2].mean():.3f})")
        
        # Note: Z is negative because of coordinate system transform
        # This is correct for robot base frame alignment
        print(f"\n[NOTE] Coordinate system after generate_pcd transform:")
        print(f"       X: right, Y: up (flipped), Z: backward (flipped)")
        print(f"       Z values are negative (matches T_cam2base calibration)")
        
        # Create visualization copy - flip Z for more intuitive viewing
        pcd_vis = o3d.geometry.PointCloud(pcd)
        pcd_vis.transform([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        
        # Add coordinate frame for reference (scale based on point cloud size)
        pcd_extent = np.max(points.max(axis=0) - points.min(axis=0))
        frame_size = max(0.1, pcd_extent * 0.2)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
        
        print(f"\n[INFO] Opening visualization window...")
        print(f"  - Use mouse to rotate/pan/zoom")
        print(f"  - Press 'Q' to close")
        print(f"  - Coordinate axes: Red=X, Green=Y, Blue=Z")
        
        # Create visualizer with custom settings for better point visibility
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Garment Point Cloud", width=800, height=600)
        vis.add_geometry(pcd_vis)
        vis.add_geometry(coord_frame)
        
        # Set render options for larger points
        render_option = vis.get_render_option()
        render_option.point_size = 3.0  # Increase point size (default is 1.0)
        render_option.background_color = np.array([0.1, 0.1, 0.1])  # Dark background
        
        vis.run()
        vis.destroy_window()
    
    def save_pointcloud(self, pcd, filepath="garment_pointcloud.ply"):
        """Save point cloud to file"""
        o3d.io.write_point_cloud(filepath, pcd)
        print(f"[INFO] Point cloud saved to {filepath}")


def main():
    # Configuration
    CAMERA_SN = "046322250624"  # TODO: Change to your camera SN
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"  # Path to SAM 2.1 weights
    
    # Initialize extractor
    extractor = GarmentPointCloudExtractor(
        camera_sn=CAMERA_SN,
        sam_checkpoint=SAM_CHECKPOINT
    )
    
    # Step 1: Capture frames
    print("[INFO] Capturing frames...")
    color_img, depth_img = extractor.capture_frames()
    
    # Save raw images for reference
    cv2.imwrite("captured_color.png", cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))
    cv2.imwrite("captured_depth.png", depth_img)
    print("[INFO] Saved captured_color.png and captured_depth.png")
    
    # Step 2: Interactive segmentation
    mask = extractor.interactive_segment(color_img)
    
    if mask is None:
        print("[WARNING] Segmentation cancelled")
        return
    
    # Save mask
    mask_img = (mask * 255).astype(np.uint8)
    cv2.imwrite("garment_mask.png", mask_img)
    print("[INFO] Saved garment_mask.png")
    
    # Step 3: Generate point cloud
    print("[INFO] Generating point cloud...")
    garment_pcd, garment_points = extractor.mask_to_pointcloud(color_img, depth_img, mask)
    
    # Step 4: Visualize and save
    extractor.visualize_pointcloud(garment_pcd)
    extractor.save_pointcloud(garment_pcd, "garment_pointcloud.ply")
    
    print("\n" + "="*50)
    print(f"✓ Point cloud extraction complete!")
    print(f"  - Total points: {len(garment_points)}")
    print(f"  - Saved to: garment_pointcloud.ply")
    print("="*50)


if __name__ == "__main__":
    main()

