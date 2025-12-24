#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D Bbox to 3D Point Projection
将VLM输出的2D边界框转换为3D空间中的点
参考 Isaac Sim 代码中的投影逻辑
"""

import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple
import open3d as o3d


class BboxTo3DProjector:
    """Project 2D bounding boxes to 3D points using point cloud"""
    
    def __init__(self, camera_intrinsics, T_cam2base):
        """
        Args:
            camera_intrinsics: Open3D camera intrinsics or dict with fx, fy, cx, cy
            T_cam2base: 4x4 transformation matrix from camera to robot base (numpy array)
        """
        # Extract intrinsic parameters
        if hasattr(camera_intrinsics, 'intrinsic_matrix'):
            K = camera_intrinsics.intrinsic_matrix
            self.fx = K[0, 0]
            self.fy = K[1, 1]
            self.cx = K[0, 2]
            self.cy = K[1, 2]
        else:
            self.fx = camera_intrinsics['fx']
            self.fy = camera_intrinsics['fy']
            self.cx = camera_intrinsics['cx']
            self.cy = camera_intrinsics['cy']
        
        self.T_cam2base = T_cam2base
        print(f"[INFO] Initialized projector with intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}")
    
    def project_pcd_to_pixels(self, pcd_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Project 3D point cloud (in camera frame) to 2D pixel coordinates
        
        Args:
            pcd_points: (N, 3) array of points in camera coordinate system
        
        Returns:
            us: (N,) array of pixel x coordinates
            vs: (N,) array of pixel y coordinates
            mask: (N,) boolean array indicating valid projections
        """
        n = pcd_points.shape[0]
        us = np.zeros(n, dtype=np.float32)
        vs = np.zeros(n, dtype=np.float32)
        mask = np.zeros(n, dtype=bool)
        
        for i, pt in enumerate(pcd_points):
            x, y, z = pt
            
            # Check if point is in front of camera
            if z <= 0:
                continue
            
            # Project to pixel coordinates
            u = self.fx * (x / z) + self.cx
            v = self.fy * (y / z) + self.cy
            
            # Check if within image bounds (assuming standard resolution)
            if 0 <= u < 640 and 0 <= v < 480:  # TODO: Make image size configurable
                us[i] = u
                vs[i] = v
                mask[i] = True
        
        return us, vs, mask
    
    def bbox_to_3d_center(
        self,
        bbox: List[float],
        pcd_points: np.ndarray,
        us: np.ndarray,
        vs: np.ndarray,
        mask: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        Find 3D center of points whose projections fall inside the 2D bbox
        
        Args:
            bbox: [x_min, y_min, x_max, y_max] in pixel coordinates
            pcd_points: (N, 3) array of 3D points in camera frame
            us, vs, mask: Output from project_pcd_to_pixels()
        
        Returns:
            center_3d: (3,) array representing 3D centroid in camera frame, or None if no points found
        """
        x_min, y_min, x_max, y_max = bbox
        
        # Find points whose pixel projections are inside bbox
        inside_bbox = (
            (us >= x_min) &
            (us <= x_max) &
            (vs >= y_min) &
            (vs <= y_max) &
            mask
        )
        
        if not np.any(inside_bbox):
            print(f"[WARNING] No points found inside bbox {bbox}")
            return None
        
        # Compute 3D centroid
        points_inside = pcd_points[inside_bbox]
        center_3d = points_inside.mean(axis=0)
        
        print(f"[INFO] Bbox {bbox} -> 3D center: {center_3d} ({len(points_inside)} points)")
        return center_3d
    
    def vlm_points_to_3d_map(
        self,
        vlm_points: List[Dict],
        pcd_points: np.ndarray,
        img_width: int = 640,
        img_height: int = 480
    ) -> Dict[str, np.ndarray]:
        """
        Convert VLM output points (with labels and bboxes) to 3D coordinates
        
        Args:
            vlm_points: List of dicts with format:
                [{"label": "left_cuff", "bbox": [x_min, y_min, x_max, y_max]}, ...]
            pcd_points: (N, 3) point cloud in camera frame
            img_width, img_height: Image dimensions (for coordinate conversion if needed)
        
        Returns:
            label_to_3d: Dict mapping label names to 3D points in camera frame
        """
        # Project point cloud to pixels
        us, vs, mask = self.project_pcd_to_pixels(pcd_points)
        
        label_to_3d = {}
        
        for item in vlm_points:
            if not isinstance(item, dict):
                continue
            
            label = item.get("label")
            bbox = item.get("bbox")
            
            if label is None or bbox is None or len(bbox) != 4:
                print(f"[WARNING] Invalid VLM point format: {item}")
                continue
            
            # Convert bbox to absolute pixel coordinates if needed
            # (Assuming VLM outputs are already in pixel coordinates)
            x_min, y_min, x_max, y_max = [float(v) for v in bbox]
            
            # Get 3D center for this bbox
            center_3d = self.bbox_to_3d_center(
                bbox=[x_min, y_min, x_max, y_max],
                pcd_points=pcd_points,
                us=us,
                vs=vs,
                mask=mask
            )
            
            if center_3d is not None:
                label_to_3d[label] = center_3d
        
        return label_to_3d
    
    def cam_to_base(self, point_cam: np.ndarray) -> np.ndarray:
        """
        Transform point from camera frame to robot base frame
        
        Args:
            point_cam: (3,) array in camera coordinates
        
        Returns:
            point_base: (3,) array in base coordinates
        """
        point_cam_homo = np.append(point_cam, 1.0)  # Convert to homogeneous coordinates
        point_base_homo = self.T_cam2base @ point_cam_homo
        return point_base_homo[:3]
    
    def visualize_bbox_on_image(
        self,
        image: np.ndarray,
        vlm_points: List[Dict],
        save_path: Optional[str] = None
    ) -> np.ndarray:
        """
        Visualize VLM bounding boxes on the image
        
        Args:
            image: RGB image (H, W, 3)
            vlm_points: List of VLM point dicts
            save_path: Optional path to save visualization
        
        Returns:
            vis_img: Image with drawn bboxes and labels
        """
        vis_img = image.copy()
        
        for item in vlm_points:
            if not isinstance(item, dict):
                continue
            
            label = item.get("label")
            bbox = item.get("bbox")
            
            if bbox is None or len(bbox) != 4:
                continue
            
            x_min, y_min, x_max, y_max = [int(v) for v in bbox]
            
            # Draw rectangle
            cv2.rectangle(vis_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            # Draw label
            if label:
                cv2.putText(
                    vis_img,
                    str(label),
                    (x_min, max(y_min - 5, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
        
        if save_path:
            cv2.imwrite(save_path, cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR))
            print(f"[INFO] Saved visualization to {save_path}")
        
        return vis_img


# ===== Test / Example Usage =====
def test_projection():
    """Test the projection pipeline with dummy data"""
    import open3d as o3d
    
    # Load camera intrinsics
    intrinsics = o3d.io.read_pinhole_camera_intrinsic("camera_intrinsics.json")
    
    # Load calibration (cam2base transform)
    calib_file = "calibration_results/camera_calibration_XXXXXX.npz"  # TODO: Update path
    calib_data = np.load(calib_file)
    T_cam2base = calib_data['T_cam2base']
    
    # Initialize projector
    projector = BboxTo3DProjector(intrinsics, T_cam2base)
    
    # Example VLM output (假设VLM返回了这些关键点)
    vlm_points = [
        {"label": "left_cuff", "bbox": [100, 200, 150, 250]},
        {"label": "right_cuff", "bbox": [450, 200, 500, 250]},
        {"label": "left_shoulder", "bbox": [120, 100, 180, 150]},
        {"label": "right_shoulder", "bbox": [420, 100, 480, 150]}
    ]
    
    # Load point cloud (from previous step)
    garment_pcd = o3d.io.read_point_cloud("garment_pointcloud.ply")
    pcd_points = np.asarray(garment_pcd.points)
    
    # Convert VLM points to 3D (camera frame)
    label_to_3d_cam = projector.vlm_points_to_3d_map(vlm_points, pcd_points)
    
    # Convert to base frame
    label_to_3d_base = {}
    for label, point_cam in label_to_3d_cam.items():
        point_base = projector.cam_to_base(point_cam)
        label_to_3d_base[label] = point_base
        print(f"{label}:")
        print(f"  Camera frame: {point_cam}")
        print(f"  Base frame:   {point_base}")
    
    # Visualize (if image available)
    color_img = cv2.imread("captured_color.png")
    if color_img is not None:
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        projector.visualize_bbox_on_image(color_img, vlm_points, "bbox_visualization.png")


if __name__ == "__main__":
    test_projection()

