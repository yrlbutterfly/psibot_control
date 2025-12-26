#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualize 2D Bbox to 3D Point Cloud Mapping
可视化 2D 边界框到 3D 点云的映射
"""

import cv2
import numpy as np
import open3d as o3d
from bbox_to_3d import map_bboxes_to_3d
from robot_libs.realsense_image_module import RealSenseImage


class BboxTo3DVisualizer:
    """Interactive visualizer for bbox to 3D mapping"""
    
    def __init__(self, rgb_image, pcd_cam, pcd_base, camera_intrinsics, T_cam2base):
        """
        Initialize visualizer
        
        Args:
            rgb_image: RGB image (H x W x 3)
            pcd_cam: Point cloud in camera coordinate system (for projection)
            pcd_base: Point cloud in base coordinate system (for visualization)
            camera_intrinsics: (3x3) camera intrinsic matrix
            T_cam2base: (4x4) transformation matrix
        """
        self.rgb_image = rgb_image.copy()
        self.pcd_cam = pcd_cam
        self.pcd_base = pcd_base
        self.pcd_points_cam = np.asarray(pcd_cam.points)
        self.camera_intrinsics = camera_intrinsics
        self.T_cam2base = T_cam2base
        
        self.img_height, self.img_width = rgb_image.shape[:2]
        
        # Drawing state
        self.bboxes = {}  # {label: [x1, y1, x2, y2]}
        self.current_bbox = None
        self.drawing = False
        self.start_point = None
        
        # Current label
        self.current_label = "point_1"
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for drawing bboxes"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.current_bbox = [
                    min(self.start_point[0], x),
                    min(self.start_point[1], y),
                    max(self.start_point[0], x),
                    max(self.start_point[1], y)
                ]
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            if self.current_bbox:
                self.bboxes[self.current_label] = self.current_bbox
                print(f"✓ Added bbox '{self.current_label}': {self.current_bbox}")
                
                # Auto increment label
                num = int(self.current_label.split('_')[1])
                self.current_label = f"point_{num + 1}"
                
                self.current_bbox = None
    
    def draw_display(self):
        """Draw current state on image"""
        display = self.rgb_image.copy()
        
        # Draw saved bboxes
        for label, bbox in self.bboxes.items():
            x1, y1, x2, y2 = bbox
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw current bbox being drawn
        if self.current_bbox:
            x1, y1, x2, y2 = self.current_bbox
            cv2.rectangle(display, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(display, self.current_label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Instructions
        cv2.putText(display, "Drag to draw bbox | SPACE: Compute 3D | ESC: Exit",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"Next label: {self.current_label}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return display
    
    def run(self):
        """Run interactive visualization"""
        print("\n" + "="*60)
        print("  Interactive Bbox to 3D Visualizer")
        print("="*60)
        print("\nInstructions:")
        print("  - Drag mouse to draw bounding box")
        print("  - Press SPACE to compute 3D positions")
        print("  - Press ESC to exit")
        print("="*60 + "\n")
        
        cv2.namedWindow("Draw Bounding Boxes", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Draw Bounding Boxes", 1000, 750)
        cv2.setMouseCallback("Draw Bounding Boxes", self.mouse_callback)
        
        while True:
            display = self.draw_display()
            cv2.imshow("Draw Bounding Boxes", cv2.cvtColor(display, cv2.COLOR_RGB2BGR))
            
            key = cv2.waitKey(10) & 0xFF
            
            if key == 27:  # ESC
                break
            elif key == 32:  # SPACE
                if len(self.bboxes) > 0:
                    self.compute_and_visualize_3d()
                else:
                    print("⚠ No bboxes drawn yet!")
        
        cv2.destroyAllWindows()
    
    def compute_and_visualize_3d(self):
        """Compute 3D positions and visualize"""
        print("\n" + "="*60)
        print("  Computing 3D Positions")
        print("="*60 + "\n")
        
        # Compute 3D positions (in base frame)
        positions_3d_base = map_bboxes_to_3d(
            self.bboxes,
            self.pcd_points_cam,
            self.camera_intrinsics,
            self.T_cam2base,
            self.img_width,
            self.img_height
        )
        
        # Visualize in 3D
        print("\n" + "="*60)
        print("  3D Visualization")
        print("="*60)
        print("  - Point cloud: white")
        print("  - Detected positions: colored spheres")
        print("  - Close window to continue")
        print("="*60 + "\n")
        
        geometries = [self.pcd_base]
        
        # Create colored spheres for detected positions
        colors = [
            [1, 0, 0],  # Red
            [0, 1, 0],  # Green
            [0, 0, 1],  # Blue
            [1, 1, 0],  # Yellow
            [1, 0, 1],  # Magenta
            [0, 1, 1],  # Cyan
        ]
        
        for i, (label, pos) in enumerate(positions_3d_base.items()):
            if pos is not None:
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
                sphere.translate(pos)
                sphere.paint_uniform_color(colors[i % len(colors)])
                geometries.append(sphere)
                
                # Add text (approximated by coordinate frame)
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=0.05, origin=pos
                )
                geometries.append(frame)
        
        # Visualize
        o3d.visualization.draw_geometries(
            geometries,
            window_name="3D Positions",
            width=1000,
            height=800
        )


def main():
    """Main function"""
    print("\n" + "="*60)
    print("  Bbox to 3D Mapping Visualizer")
    print("="*60 + "\n")
    
    # Load data
    print("[1/5] Loading RGB image...")
    if not os.path.exists('captured_color.png'):
        print("❌ captured_color.png not found")
        print("   Please run get_garment_pointcloud.py first")
        return
    
    rgb_image = cv2.imread('captured_color.png')
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    print(f"✓ Image loaded: {rgb_image.shape}")
    
    print("\n[2/5] Loading point clouds...")
    if not os.path.exists('garment_pointcloud.ply'):
        print("❌ garment_pointcloud.ply not found")
        print("   Please run get_garment_pointcloud.py first")
        return
    
    if not os.path.exists('garment_pointcloud_base.ply'):
        print("❌ garment_pointcloud_base.ply not found")
        print("   Please run test_point_transform.py first")
        return
    
    pcd_cam = o3d.io.read_point_cloud('garment_pointcloud.ply')
    pcd_base = o3d.io.read_point_cloud('garment_pointcloud_base.ply')
    print(f"✓ Camera frame: {len(pcd_cam.points)} points")
    print(f"✓ Base frame:   {len(pcd_base.points)} points")
    
    print("\n[3/5] Loading calibration...")
    calib = np.load('calibration_results/camera_calibration_right_arm_20251226-234855.npz')
    T_cam2base = calib['T_cam2base']
    print("✓ Calibration loaded")
    
    print("\n[4/5] Getting camera intrinsics...")
    # Use real camera intrinsics
    camera = RealSenseImage(SN_number="046322250624")
    camera_intrinsics = camera.o3d_intrinsics.intrinsic_matrix
    camera.close()
    print("✓ Camera intrinsics obtained")
    print(f"\nIntrinsics:\n{camera_intrinsics}")
    
    print("\n[5/5] Starting visualizer...")
    visualizer = BboxTo3DVisualizer(
        rgb_image,
        pcd_cam,
        pcd_base,
        camera_intrinsics,
        T_cam2base
    )
    
    visualizer.run()
    
    print("\n✓ Visualization complete")


if __name__ == "__main__":
    import os
    main()

