import sys
import os
import cv2
import numpy as np
import time
import open3d as o3d
import torch
from scipy.spatial.transform import Rotation as R

# 引入你现有的模块
from robot_libs.realsense_image_module import RealSenseImage, generate_pcd
from robot_libs.realman_arm_module import ArmControl
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# ---------------------------------------------------------
# 1. 配置区域
# ---------------------------------------------------------
CAMERA_SN = "134222070573"       # 你的相机SN (参考 camera.py)
ROBOT_IP = "192.168.100.101"     # 你的机械臂IP (参考 control.py, 右臂)
CALIB_FILE = "calibration_results/camera_calibration_xxxx.npz" # 你的标定文件路径
SAM_CHECKPOINT = "sam2.1_hiera_large.pt" # SAM 2.1权重路径

# ---------------------------------------------------------
# 2. 辅助类：加载标定结果 (参考 vis_cam_to_base.py)
# ---------------------------------------------------------
class CalibrationHandler:
    def __init__(self, calib_file):
        if not os.path.exists(calib_file):
            raise FileNotFoundError(f"标定文件未找到: {calib_file}")
        
        data = np.load(calib_file)
        if 'T_cam2base' in data:
            self.T_cam2base = data['T_cam2base']
        elif 'R_cam2base' in data and 't_cam2base' in data:
            self.T_cam2base = np.eye(4)
            self.T_cam2base[:3, :3] = data['R_cam2base']
            self.T_cam2base[:3, 3] = data['t_cam2base'].flatten()
        else:
            raise ValueError("标定文件格式错误")
        print("标定矩阵加载成功:\n", self.T_cam2base)

    def cam_to_base(self, point_cam):
        """将相机坐标系下的点 (x,y,z) 转换到基座坐标系"""
        p_cam_homo = np.append(point_cam, 1.0) # 转齐次坐标 [x,y,z,1]
        p_base_homo = self.T_cam2base @ p_cam_homo
        return p_base_homo[:3]

# ---------------------------------------------------------
# 3. 辅助函数：SAM 分割 & 3D点提取 (整合 sam.py + get_pc.py)
# ---------------------------------------------------------
def get_target_point_from_image(color_img, depth_img, intrinsics, predictor):
    """
    交互式：点击图片 -> SAM分割 -> 计算Mask中心 -> 映射回3D点
    """
    # 1. 交互式选点 (参考 sam.py)
    clicked_point = []
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_point.append((x, y))
            print(f"用户点击: {x}, {y}")

    print("请在弹出的窗口中点击要抓取的物体，然后按任意键继续...")
    temp_img = color_img.copy()
    cv2.namedWindow("Select Target")
    cv2.setMouseCallback("Select Target", on_mouse)
    while not clicked_point:
        cv2.imshow("Select Target", temp_img)
        if cv2.waitKey(10) & 0xFF == 27: # ESC退出
            break
    cv2.destroyAllWindows()

    if not clicked_point:
        return None

    # 2. 运行 SAM (参考 sam.py)
    predictor.set_image(color_img)
    input_point = np.array([clicked_point[0]])
    input_label = np.array([1])
    masks, _, _ = predictor.predict(point_coords=input_point, point_labels=input_label, multimask_output=False)
    target_mask = masks[0] # 拿到最好的 Mask

    # 3. 结合深度图计算 3D 坐标 (参考 get_pc.py)
    # 我们不需要生成整个点云，只要计算 Mask 区域的 3D 中心即可
    
    # 获取 Mask 区域内的所有像素坐标 (v, u) -> (y, x)
    ys, xs = np.where(target_mask)
    if len(xs) == 0: return None

    # 获取对应的深度值 (单位通常是 mm，需要转 m)
    depths = depth_img[ys, xs].astype(float) / 1000.0
    
    # 简单的过滤：去掉深度为0的无效点
    valid = depths > 0
    if not np.any(valid): return None
    
    xs = xs[valid]
    ys = ys[valid]
    depths = depths[valid]

    # 反投影：Pixel (u, v, z) -> Camera (X, Y, Z)
    # X = (u - cx) * z / fx
    # Y = (v - cy) * z / fy
    fx = intrinsics.intrinsic_matrix[0, 0]
    fy = intrinsics.intrinsic_matrix[1, 1]
    cx = intrinsics.intrinsic_matrix[0, 2]
    cy = intrinsics.intrinsic_matrix[1, 2]

    X_points = (xs - cx) * depths / fx
    Y_points = (ys - cy) * depths / fy
    Z_points = depths

    # 计算点云中心 (Centroid) 作为抓取点
    # 也可以用更复杂的逻辑，比如 PCA 计算主轴方向
    cam_point = np.array([np.mean(X_points), np.mean(Y_points), np.mean(Z_points)])
    
    print(f"目标在相机坐标系下的位置: {cam_point}")
    return cam_point

# ---------------------------------------------------------
# 4. 主流程
# ---------------------------------------------------------
def main():
    # A. 初始化硬件
    camera = RealSenseImage(SN_number=CAMERA_SN)
    robot = ArmControl(ip=ROBOT_IP)
    calib = CalibrationHandler(CALIB_FILE)
    
    # B. 初始化 SAM 2
    print("正在加载 SAM 2.1 模型...")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"使用设备: {device}")
    # Use official config path from SAM 2 package
    model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
    sam2_model = build_sam2(model_cfg, SAM_CHECKPOINT, device=device)
    predictor = SAM2ImagePredictor(sam2_model)
    print("模型加载完成")

    # C. 移动到拍照姿态 (参考 control.py 里的初始位置)
    # robot.move_joint(...) # 如果需要的话

    # D. 拍照 & 感知
    # 等待相机稳定
    time.sleep(1) 
    color_img, depth_img = camera.capture_frame()
    
    # 获取目标点 (相机系)
    target_cam = get_target_point_from_image(color_img, depth_img, camera.o3d_intrinsics, predictor)
    
    if target_cam is None:
        print("未选择目标或无法计算坐标")
        return

    # E. 坐标转换 (关键！)
    target_base = calib.cam_to_base(target_cam)
    print(f"目标在基座坐标系下的位置: {target_base}")

    # F. 执行运动 (Motion Planning 基础)
    # 注意：这里需要定义抓取时的姿态 (Orientation)
    # 假设我们垂直向下抓取 (需要根据你的手眼标定和夹爪方向调整)
    # 欧拉角 [Roll, Pitch, Yaw] (弧度或角度，取决于 robot_libs 的实现，看代码是 xyzrpy)
    
    # 这里我们复用 control.py 里见过的欧拉角或者自己定义
    # 比如：末端垂直向下
    rx, ry, rz = 3.14, 0, 0 
    
    target_pose = [target_base[0], target_base[1], target_base[2], rx, ry, rz]
    
    print(f"即将移动到: {target_pose}")
    user_confirm = input("按回车确认移动，输入 n 取消: ")
    if user_confirm.lower() != 'n':
        # 使用 move_pose_Cmd 进行笛卡尔空间规划
        robot.move_pose_Cmd(target_pose, speed=10)
        print("移动完成")

    # G. 清理
    robot.close()

if __name__ == "__main__":
    main()