import cv2
import numpy as np
from robot_libs.realsense_image_module import RealSenseImage
import os

def project_3d_to_2d(pts_3d, K, transform):
    pts_cam = transform @ pts_3d
    # 允许负Z轴投影，但需要标记
    is_behind = pts_cam[2] <= 0
    if abs(pts_cam[2]) < 1e-10: return np.array([np.nan, np.nan]), True
    
    pts_2d = K @ (pts_cam[:3] / pts_cam[2])
    return pts_2d[:2], is_behind

def draw_xyz_axis(img, transform, label, K, scale=0.1, thickness=3, shift_to_center=False):
    """
    绘制坐标轴
    """
    h, w = img.shape[:2]
    
    # 定义轴的端点 (齐次坐标)
    axis_points = np.array([
        [0, 0, 0, 1], # 原点
        [scale, 0, 0, 1], # X (Red)
        [0, scale, 0, 1], # Y (Green)
        [0, 0, scale, 1]  # Z (Blue)
    ]).T

    # 如果需要强制画在指定位置（只看方向）
    # 注意：这里的transform已经是shifted过的，所以不需要特殊处理，直接投影
    # 但如果shift_to_center=True，我们假设transform是原始的，需要在这里做shift?
    # 为了简化，我们假设传入的transform已经是决定好位置的。
    # 只需处理投影逻辑。

    pts_cam = transform @ axis_points
    
    # 简单的背后检查
    if pts_cam[2, 0] <= 0: return False

    pts_2d = K @ (pts_cam[:3, :] / pts_cam[2, :])
    pts_2d = pts_2d[:2, :] # shape (2, 4)
    
    origin = tuple(np.round(pts_2d[:, 0]).astype(int))
    xx = tuple(np.round(pts_2d[:, 1]).astype(int))
    yy = tuple(np.round(pts_2d[:, 2]).astype(int))
    zz = tuple(np.round(pts_2d[:, 3]).astype(int))
    
    # 检查原点是否在图内
    if not (0 <= origin[0] < w and 0 <= origin[1] < h):
        return False

    cv2.arrowedLine(img, origin, xx, (0, 0, 255), thickness, cv2.LINE_AA) # X Red
    cv2.arrowedLine(img, origin, yy, (0, 255, 0), thickness, cv2.LINE_AA) # Y Green
    cv2.arrowedLine(img, origin, zz, (255, 0, 0), thickness, cv2.LINE_AA) # Z Blue
    cv2.putText(img, label, (origin[0]-20, origin[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return True

def make_shifted_transform(original_transform, target_u, target_v, K, z_depth=0.5):
    """创建平移后的变换矩阵，用于在图像特定位置显示方向"""
    K_inv = np.linalg.inv(K)
    p_cam = K_inv @ np.array([target_u, target_v, 1.0]) * z_depth
    
    new_transform = np.eye(4)
    new_transform[:3, :3] = original_transform[:3, :3]
    new_transform[:3, 3] = p_cam
    return new_transform

def format_pos(pos):
    """格式化位置字符串: 左/右, 上/下, 前/后"""
    x, y, z = pos
    lr = "Right" if x > 0 else "Left"
    ud = "Down" if y > 0 else "Up"
    fb = "Front" if z > 0 else "Behind"
    return f"{lr}:{abs(x)*100:.0f}cm {ud}:{abs(y)*100:.0f}cm {fb}:{abs(z)*100:.0f}cm"

def main():
    left_calib_file = "calibration_results/camera_calibration_left_arm_20251226-234053.npz"
    right_calib_file = "calibration_results/camera_calibration_right_arm_20251226-234855.npz"
    
    print(f"[INFO] Loading Left Calibration: {left_calib_file}")
    left_data = np.load(left_calib_file)
    T_cam2base_left = left_data['T_cam2base']
    T_base2cam_left = np.linalg.inv(T_cam2base_left)
    
    print(f"[INFO] Loading Right Calibration: {right_calib_file}")
    right_data = np.load(right_calib_file)
    T_cam2base_right = right_data['T_cam2base']
    T_base2cam_right = np.linalg.inv(T_cam2base_right)

    print("[INFO] Initializing Camera...")
    try:
        camera = RealSenseImage()
    except Exception as e:
        print(f"[ERROR] Camera init failed: {e}")
        return

    try:
        K = camera.o3d_intrinsics.intrinsic_matrix
    except:
        K = np.array([[605.0, 0, 320.0], [0, 605.0, 240.0], [0, 0, 1.0]])

    print("\n" + "="*60)
    print("  Dual Arm Base Origin Locator")
    print("  Camera Frame Definition: +Z=Front, +X=Right, +Y=Down")
    print("="*60 + "\n")

    while True:
        color_img = camera.capture_rgb_frame()
        color_img_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
        h, w = color_img_bgr.shape[:2]

        # 1. Calculate Origin Positions in Camera Frame
        origin_left_cam = T_base2cam_left[:3, 3]
        origin_right_cam = T_base2cam_right[:3, 3]
        
        # 2. Display Text Info
        # Left Info (Top Left)
        cv2.putText(color_img_bgr, "L-Origin (Rel. to Cam):", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(color_img_bgr, format_pos(origin_left_cam), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Right Info (Top Right)
        label_text = "R-Origin (Rel. to Cam):"
        pos_text = format_pos(origin_right_cam)
        
        label_w = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
        pos_w = cv2.getTextSize(pos_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0]
        
        cv2.putText(color_img_bgr, label_text, (w - label_w - 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(color_img_bgr, pos_text, (w - pos_w - 10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 3. Draw Orientation Indicators (Bottom Corners)
        T_vis_left = make_shifted_transform(T_base2cam_left, 60, h-60, K)
        draw_xyz_axis(color_img_bgr, T_vis_left, "L-Base", K, scale=0.05, thickness=2)

        T_vis_right = make_shifted_transform(T_base2cam_right, w-60, h-60, K)
        draw_xyz_axis(color_img_bgr, T_vis_right, "R-Base", K, scale=0.05, thickness=2)

        # 4. Try to draw actual origins if in view (or close)
        # Left
        if origin_left_cam[2] > 0: # If in front
            uv_left, _ = project_3d_to_2d(np.array([0,0,0,1]), K, T_base2cam_left)
            if not np.any(np.isnan(uv_left)):
                pt = (int(uv_left[0]), int(uv_left[1]))
                # Draw a line from center to it if out of view? 
                # Or just draw it if in view
                if 0 <= pt[0] < w and 0 <= pt[1] < h:
                    cv2.circle(color_img_bgr, pt, 5, (0, 255, 0), -1)
                    cv2.putText(color_img_bgr, "REAL L-ORG", pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Right
        if origin_right_cam[2] > 0:
            uv_right, _ = project_3d_to_2d(np.array([0,0,0,1]), K, T_base2cam_right)
            if not np.any(np.isnan(uv_right)):
                pt = (int(uv_right[0]), int(uv_right[1]))
                if 0 <= pt[0] < w and 0 <= pt[1] < h:
                    cv2.circle(color_img_bgr, pt, 5, (0, 255, 255), -1)
                    cv2.putText(color_img_bgr, "REAL R-ORG", pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Dual Arm Bases", color_img_bgr)
        
        if cv2.waitKey(30) == ord('q'):
            break

    camera.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
