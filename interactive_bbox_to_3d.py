#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Bbox to 3D Mapper (Camera Frame) - Real-time Camera Mode
交互式bbox到3D质心映射工具（相机坐标系）- 实时相机模式

Two-stage workflow:
1. First: Select garment region using SAM2 interactive segmentation (reusing GarmentPointCloudExtractor)
2. Second: Draw bboxes on garment and extract 3D centers

This tool visualizes and computes 3D centers in CAMERA coordinate system.
All 3D coordinates are in the camera frame (not robot base frame).
Captures data from camera in real-time.
"""

import cv2
import numpy as np
import open3d as o3d
from robot_libs.realsense_image_module import RealSenseImage
from bbox_to_3d import project_pcd_to_image, get_3d_center_from_bbox
from get_garment_pointcloud import GarmentPointCloudExtractor


class InteractiveBboxTo3D:
    """Interactive tool for drawing bboxes and viewing their 3D centers (real-time camera)"""
    
    def __init__(self, camera_sn, sam_checkpoint="sam2.1_hiera_large.pt"):
        # Initialize garment extractor (includes camera and SAM model)
        print("="*60)
        print("  Interactive Bbox to 3D Mapper - Real-time Camera Mode")
        print("="*60)
        
        self.extractor = GarmentPointCloudExtractor(
            camera_sn=camera_sn,
            sam_checkpoint=sam_checkpoint
        )
        self.camera = self.extractor.camera
        self.K = self.extractor.intrinsics.intrinsic_matrix
        
        # Capture initial data and select garment
        self.initial_setup()
        
        # State for drawing
        self.drawing = False
        self.start_point = None
        self.current_bbox = None
        self.bboxes = []  # List of (bbox, label, center_3d)
        self.label_counter = 1
        
        self.window_name = "Interactive Bbox to 3D (Real-time Camera)"
        
        print("\n" + "="*60)
        print("  Instructions:")
        print("="*60)
        print("1. Click and drag to draw a bounding box")
        print("2. Release to confirm - 3D center will be computed")
        print("3. Press 'r' to reselect garment region")
        print("4. Press 's' to save results")
        print("5. Press 'c' to clear all bboxes")
        print("6. Press 'v' to visualize 3D point cloud with centers")
        print("7. Press 'q' or ESC to quit")
        print("="*60 + "\n")
    
    def initial_setup(self):
        """Initial camera capture and garment selection"""
        print("\n" + "="*60)
        print("  Step 1: Capture Image and Select Garment")
        print("="*60)
        
        # Capture frames using extractor
        self.last_color_bgr, self.last_depth_uint16 = self.extractor.capture_frames()
        
        # Save captured images for reference
        cv2.imwrite("captured_color.png", self.last_color_bgr)
        cv2.imwrite("captured_depth.png", self.last_depth_uint16)
        print("[INFO] Saved captured_color.png and captured_depth.png")
        
        # Interactive segmentation using extractor
        self.garment_mask = self.extractor.interactive_segment(self.last_color_bgr)
        
        if self.garment_mask is None:
            print("[ERROR] No garment selected. Exiting...")
            self.camera.close()
            exit(1)
        
        # Save mask
        mask_img = (self.garment_mask * 255).astype(np.uint8)
        cv2.imwrite("garment_mask.png", mask_img)
        print("[INFO] Saved garment_mask.png")
        
        # Generate garment point cloud
        print("\n" + "="*60)
        print("  Step 2: Generate Garment Point Cloud")
        print("="*60)
        self.update_garment_pointcloud()
    
    def update_garment_pointcloud(self):
        """Generate garment point cloud based on selected mask"""
        # Use extractor's mask_to_pointcloud method (includes filtering)
        garment_pcd, garment_points = self.extractor.mask_to_pointcloud(
            self.last_color_bgr, 
            self.last_depth_uint16, 
            self.garment_mask
        )
        
        # Extract point cloud data
        self.points = garment_points
        self.colors = np.asarray(garment_pcd.colors)
        
        # Convert to RGB for display
        self.rgb = cv2.cvtColor(self.last_color_bgr, cv2.COLOR_BGR2RGB)
        self.img_h, self.img_w = self.rgb.shape[:2]
        
        print(f"\n✓ RGB: {self.img_w}x{self.img_h}")
        print(f"✓ Garment point cloud: {len(self.points)} points")
        if len(self.points) > 0:
            print(f"✓ Z range: {self.points[:, 2].min():.3f} to {self.points[:, 2].max():.3f}m")
        
        # Project points to image
        print("\nProjecting points to image...")
        self.us, self.vs, valid = project_pcd_to_image(self.points, self.K)
        self.valid_mask = valid & (self.us >= 0) & (self.us < self.img_w) & \
                         (self.vs >= 0) & (self.vs < self.img_h)
        
        print(f"✓ Valid projected points: {np.sum(self.valid_mask)}/{len(self.points)}")
        
        # Create display image with projection overlay
        self.base_img = self.rgb.copy()
        for u, v in zip(self.us[self.valid_mask], self.vs[self.valid_mask]):
            u_int, v_int = int(u), int(v)
            if 0 <= u_int < self.img_w and 0 <= v_int < self.img_h:
                # Make projection semi-transparent
                alpha = 0.3
                self.base_img[v_int, u_int] = (
                    self.base_img[v_int, u_int] * (1 - alpha) + 
                    np.array([0, 255, 0]) * alpha
                ).astype(np.uint8)
        
        # Save point cloud for reference
        self.extractor.save_pointcloud(garment_pcd, "garment_pointcloud.ply")
        print("✓ Garment point cloud updated and saved!")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for bbox drawing"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            self.current_bbox = None
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing and self.start_point is not None:
                self.current_bbox = (
                    min(self.start_point[0], x),
                    min(self.start_point[1], y),
                    max(self.start_point[0], x),
                    max(self.start_point[1], y)
                )
        
        elif event == cv2.EVENT_LBUTTONUP:
            if self.drawing and self.start_point is not None:
                self.drawing = False
                
                # Finalize bbox
                x_min = min(self.start_point[0], x)
                y_min = min(self.start_point[1], y)
                x_max = max(self.start_point[0], x)
                y_max = max(self.start_point[1], y)
                
                # Make sure bbox has minimum size
                if abs(x_max - x_min) > 10 and abs(y_max - y_min) > 10:
                    bbox = [x_min, y_min, x_max, y_max]
                    
                    # Debug: check points in bbox
                    inside_2d = (
                        (self.us >= x_min) & (self.us <= x_max) &
                        (self.vs >= y_min) & (self.vs <= y_max) &
                        self.valid_mask
                    )
                    num_points = np.sum(inside_2d)
                    print(f"\n[DEBUG] Bbox: [{x_min}, {y_min}, {x_max}, {y_max}]")
                    print(f"[DEBUG] Points in bbox: {num_points}")
                    
                    if num_points > 0:
                        selected_pts = self.points[inside_2d]
                        print(f"[DEBUG] Selected points 3D range:")
                        print(f"  X: {selected_pts[:, 0].min():.3f} ~ {selected_pts[:, 0].max():.3f}")
                        print(f"  Y: {selected_pts[:, 1].min():.3f} ~ {selected_pts[:, 1].max():.3f}")
                        print(f"  Z: {selected_pts[:, 2].min():.3f} ~ {selected_pts[:, 2].max():.3f}")
                    
                    # Compute 3D center in camera frame
                    # Use identity matrix to stay in camera coordinate system
                    T_cam2base = np.eye(4)  # No transformation - stay in camera frame
                    
                    result = get_3d_center_from_bbox(
                        bbox, self.points, self.us, self.vs, 
                        self.valid_mask, self.img_w, self.img_h, T_cam2base
                    )
                    
                    if result is not None:
                        center_3d_identity, center_3d_cam = result
                        # Both should be the same since we used identity matrix
                        label = f"P{self.label_counter}"
                        # Store camera frame coordinates
                        self.bboxes.append((bbox, label, center_3d_cam))
                        self.label_counter += 1
                        print(f"✓ {label}: 3D center (camera) = ({center_3d_cam[0]:.3f}, {center_3d_cam[1]:.3f}, {center_3d_cam[2]:.3f})m\n")
                    else:
                        print(f"✗ No 3D points found in bbox!\n")
                
                self.current_bbox = None
                self.start_point = None
    
    def draw_display(self):
        """Draw current state"""
        display = self.base_img.copy()
        
        # Draw confirmed bboxes
        for bbox, label, center_3d in self.bboxes:
            x_min, y_min, x_max, y_max = [int(v) for v in bbox]
            
            # Draw bbox
            cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
            
            # Draw label
            cv2.putText(display, label, (x_min, y_min - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Draw center marker
            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2
            cv2.circle(display, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Draw 3D coordinates (convert to float to avoid numpy format issues)
            coord_text = f"3D:({float(center_3d[0]):.2f},{float(center_3d[1]):.2f},{float(center_3d[2]):.2f})"
            cv2.putText(display, coord_text, (x_min, y_max + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # Draw current bbox being drawn
        if self.current_bbox is not None:
            x_min, y_min, x_max, y_max = [int(v) for v in self.current_bbox]
            cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (0, 255, 255), 2)
        
        # Add instruction text
        cv2.putText(display, "Draw bbox | 'r':refresh | 's':save | 'c':clear | 'v':3D | 'q':quit",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return display
    
    def save_results(self):
        """Save annotated image and 3D coordinates"""
        if len(self.bboxes) == 0:
            print("No bboxes to save!")
            return
        
        # Save annotated image
        display = self.draw_display()
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imwrite('interactive_bbox_result.png', display_bgr)
        print(f"\n✓ Saved: interactive_bbox_result.png")
        
        # Save 3D coordinates to text file
        with open('interactive_bbox_3d_coords.txt', 'w') as f:
            f.write("Label,Bbox[x_min,y_min,x_max,y_max],3D_Center_Camera[x,y,z]\n")
            for bbox, label, center_3d in self.bboxes:
                bbox_str = f"[{bbox[0]:.0f},{bbox[1]:.0f},{bbox[2]:.0f},{bbox[3]:.0f}]"
                coord_str = f"[{float(center_3d[0]):.4f},{float(center_3d[1]):.4f},{float(center_3d[2]):.4f}]"
                f.write(f"{label},{bbox_str},{coord_str}\n")
        print(f"✓ Saved: interactive_bbox_3d_coords.txt")
    
    def visualize_3d(self):
        """Visualize point cloud with 3D centers marked"""
        if len(self.bboxes) == 0:
            print("No bboxes to visualize!")
            return
        
        print("\nOpening 3D visualization (Camera Coordinate System)...")
        
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        
        # Create coordinate frame (Camera frame: X=right, Y=up, Z=backward)
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        
        # Create spheres for each 3D center
        geometries = [pcd, coord_frame]
        colors = [
            [1, 0, 0],  # Red
            [0, 1, 0],  # Green
            [0, 0, 1],  # Blue
            [1, 1, 0],  # Yellow
            [1, 0, 1],  # Magenta
            [0, 1, 1],  # Cyan
        ]
        
        for i, (bbox, label, center_3d) in enumerate(self.bboxes):
            # Create sphere at 3D center
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
            sphere.translate(center_3d)
            sphere.paint_uniform_color(colors[i % len(colors)])
            geometries.append(sphere)
            
            print(f"  {label}: ({float(center_3d[0]):.3f}, {float(center_3d[1]):.3f}, {float(center_3d[2]):.3f})m")
        
        o3d.visualization.draw_geometries(
            geometries,
            window_name="3D Centers on Point Cloud (Camera Frame)",
            width=1024,
            height=768
        )
    
    def run(self):
        """Run interactive loop"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1200, 900)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        try:
            while True:
                display = self.draw_display()
                display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
                cv2.imshow(self.window_name, display_bgr)
                
                key = cv2.waitKey(10) & 0xFF
                
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('r'):  # Reselect garment
                    print("\n[Re-selecting garment region...]")
                    # Capture new frames using extractor
                    self.last_color_bgr, self.last_depth_uint16 = self.extractor.capture_frames()
                    
                    # Save captured images
                    cv2.imwrite("captured_color.png", self.last_color_bgr)
                    cv2.imwrite("captured_depth.png", self.last_depth_uint16)
                    print("[INFO] Updated captured images")
                    
                    # Interactive segmentation using extractor
                    mask = self.extractor.interactive_segment(self.last_color_bgr)
                    
                    if mask is not None:
                        self.garment_mask = mask
                        
                        # Save mask
                        mask_img = (self.garment_mask * 255).astype(np.uint8)
                        cv2.imwrite("garment_mask.png", mask_img)
                        print("[INFO] Updated garment_mask.png")
                        
                        # Regenerate garment point cloud
                        self.update_garment_pointcloud()
                        
                        # Clear current drawing state but keep saved bboxes
                        self.drawing = False
                        self.start_point = None
                        self.current_bbox = None
                    else:
                        print("[INFO] Garment reselection cancelled.")
                elif key == ord('s'):  # Save
                    self.save_results()
                elif key == ord('c'):  # Clear
                    self.bboxes = []
                    self.label_counter = 1
                    print("Cleared all bboxes")
                elif key == ord('v'):  # Visualize 3D
                    self.visualize_3d()
        
        finally:
            cv2.destroyAllWindows()
            self.camera.close()
            print("\nCamera closed.")
        
        # Final summary
        print("\n" + "="*60)
        print(f"  Summary: {len(self.bboxes)} bboxes created")
        print(f"  (All coordinates in Camera Frame)")
        print("="*60)
        for bbox, label, center_3d in self.bboxes:
            print(f"{label}: 3D (cam) = ({float(center_3d[0]):.3f}, {float(center_3d[1]):.3f}, {float(center_3d[2]):.3f})m")
        print("="*60 + "\n")


def main():
    # Real-time camera mode with SAM2 garment segmentation
    # Configuration
    CAMERA_SN = '046322250624'  # Update this to your camera serial number
    SAM_CHECKPOINT = "sam2.1_hiera_large.pt"
    
    print("\n" + "="*60)
    print("  Interactive Bbox to 3D Tool")
    print("  Two-stage workflow:")
    print("    1. Select garment region (SAM2 segmentation)")
    print("    2. Draw bboxes to extract 3D centers")
    print("="*60 + "\n")
    
    tool = InteractiveBboxTo3D(camera_sn=CAMERA_SN, sam_checkpoint=SAM_CHECKPOINT)
    tool.run()


if __name__ == "__main__":
    main()

