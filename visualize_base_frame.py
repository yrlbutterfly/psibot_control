import cv2
import numpy as np
from robot_libs.realsense_image_module import RealSenseImage
import time

def project_3d_to_2d(pts_3d, K, transform):
    pts_cam = transform @ pts_3d
    # 允许负Z轴投影（虽然在相机背面），主要看像素坐标
    if abs(pts_cam[2]) < 1e-10: return np.array([np.nan, np.nan])
    pts_2d = K @ (pts_cam[:3] / pts_cam[2])
    return pts_2d[:2]

def draw_xyz_axis(color, transform, scale=0.1, K=np.eye(3), thickness=3, shift_to_center=False):
    '''
    绘制坐标轴
    shift_to_center: 如果为True，忽略实际位置，强制画在图像中心，只显示方向
    '''
    h, w = color.shape[:2]
    
    # 定义轴的端点 (齐次坐标)
    # X红, Y绿, Z蓝
    axis_points = np.array([
        [0, 0, 0, 1], # 原点
        [scale, 0, 0, 1], # X
        [0, scale, 0, 1], # Y
        [0, 0, scale, 1]  # Z
    ]).T

    # 计算变换后的点
    if shift_to_center:
        # 取出旋转矩阵
        R = transform[:3, :3]
        # 创建一个新的变换矩阵，旋转不变，平移设为相机前方一定距离
        center_transform = np.eye(4)
        center_transform[:3, :3] = R
        # 将原点移到相机前方 0.5米处 (Z=0.5)，且在图像中心 (X=0, Y=0)
        # 这样投影后就在画面正中间
        # P_cam = [0, 0, 0.5]
        # 但我们这里的 transform 是 "Base to Camera"，即 P_cam = T * P_base
        # 我们想画的是 Base 的轴。
        # 实际上我们只需要把 P_cam 的平移部分改一下
        
        # 简单做法：直接在相机坐标系下构造点，然后投影
        # P_cam_origin = [0, 0, 0.5]
        # P_cam_x = P_cam_origin + R[:,0] * scale
        # ...
        
        origin_cam = np.array([0, 0, 0.5]) # 放相机前0.5米
        x_cam = origin_cam + R[:, 0] * scale
        y_cam = origin_cam + R[:, 1] * scale
        z_cam = origin_cam + R[:, 2] * scale
        
        pts_cam = np.vstack([origin_cam, x_cam, y_cam, z_cam]).T
        
        # 投影
        pts_2d = K @ pts_cam
        pts_2d = pts_2d[:2] / pts_2d[2]
        pts_2d = pts_2d.T # shape (4, 2)
        
        origin = tuple(np.round(pts_2d[0]).astype(int))
        xx = tuple(np.round(pts_2d[1]).astype(int))
        yy = tuple(np.round(pts_2d[2]).astype(int))
        zz = tuple(np.round(pts_2d[3]).astype(int))
        
    else:
        # 正常投影
        pts_cam = transform @ axis_points
        if np.any(pts_cam[2, :] <= 0): # 如果有任何点在相机背面
             return color, False
             
        pts_2d = K @ (pts_cam[:3, :] / pts_cam[2, :])
        pts_2d = pts_2d[:2, :] # shape (2, 4)
        
        origin = tuple(np.round(pts_2d[:, 0]).astype(int))
        xx = tuple(np.round(pts_2d[:, 1]).astype(int))
        yy = tuple(np.round(pts_2d[:, 2]).astype(int))
        zz = tuple(np.round(pts_2d[:, 3]).astype(int))

    img = color.copy()
    cv2.arrowedLine(img, origin, xx, (0, 0, 255), thickness, cv2.LINE_AA) # X Red
    cv2.arrowedLine(img, origin, yy, (0, 255, 0), thickness, cv2.LINE_AA) # Y Green
    cv2.arrowedLine(img, origin, zz, (255, 0, 0), thickness, cv2.LINE_AA) # Z Blue
    
    return img, True

def visualize_base_origin():
    # ... (初始化代码与之前相同) ...
    calib_file = "calibration_results/camera_calibration_right_arm_20251226-234855.npz"
    camera_sn = "046322250624"
    
    print(f"[INFO] 初始化相机 (SN: {camera_sn})...")
    try:
        camera = RealSenseImage(SN_number=camera_sn)
    except Exception as e:
        print(f"[ERROR] {e}"); return

    print(f"[INFO] 加载标定文件: {calib_file}")
    try:
        calib_data = np.load(calib_file)
        T_cam2base = calib_data['T_cam2base'] if 'T_cam2base' in calib_data else None
        if T_cam2base is None: return
    except: return

    try:
        K = camera.o3d_intrinsics.intrinsic_matrix
    except:
        K = np.array([[616.0, 0, 320.0], [0, 616.0, 240.0], [0, 0, 1.0]])

    print("\n" + "="*60)
    print("  可视化基座坐标系 (Base Frame)")
    print("  [DEBUG] 将打印基座在相机画面中的像素坐标")
    print("  同时在画面中心显示基座坐标系的方向")
    print("="*60 + "\n")

    while True:
        color_img = camera.capture_rgb_frame()
        color_img_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
        h, w = color_img_bgr.shape[:2]

        # 1. 计算真实的基座原点
        T_Base2Cam = np.linalg.inv(T_cam2base)
        origin_pos_cam = T_Base2Cam[:3, 3] # 基座原点在相机系下的坐标 (米)
        
        # 投影原点
        origin_uv = project_3d_to_2d(np.array([0,0,0,1]), K, T_Base2Cam)
        
        # 2. 在画面中心绘制方向 (Shifted)
        vis_img, _ = draw_xyz_axis(color_img_bgr, T_Base2Cam, scale=0.1, K=K, thickness=3, shift_to_center=True)
        
        # 3. 显示调试信息
        debug_text = f"Base Origin (Pixel): u={origin_uv[0]:.0f}, v={origin_uv[1]:.0f}"
        in_view = (0 <= origin_uv[0] < w) and (0 <= origin_uv[1] < h)
        status = " [IN VIEW]" if in_view else " [OUT OF VIEW]"
        
        cv2.putText(vis_img, "Center: Base Orientation", (w//2 - 100, h//2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(vis_img, debug_text + status, (20, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 尝试画真实的箭头（如果看得到）
        vis_img, drawn = draw_xyz_axis(vis_img, T_Base2Cam, scale=0.2, K=K, thickness=2, shift_to_center=False)
        if drawn:
             cv2.putText(vis_img, "Real Base Origin", (int(origin_uv[0]), int(origin_uv[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Robot Base Frame", vis_img)
        
        if cv2.waitKey(30) == ord('q'):
            break

    camera.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    visualize_base_origin()
