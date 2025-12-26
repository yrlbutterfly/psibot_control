#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定结果验证脚本
检查标定质量并可视化验证
"""

import numpy as np
import json
import cv2
from robot_libs.realsense_image_module import RealSenseImage
from robot_libs.realman_arm_module import ArmControl
from scipy.spatial.transform import Rotation as R
import sys

def print_header(text):
    print("\n" + "="*60)
    print(f"  {text}")
    print("="*60 + "\n")

def load_calibration(calib_file):
    """加载标定结果"""
    print(f"加载标定文件: {calib_file}")
    
    # 加载 npz
    data = np.load(calib_file)
    T_cam2base = data['T_cam2base']
    
    print("✓ 标定矩阵加载成功")
    print("\ncam2base 变换矩阵:")
    print(T_cam2base)
    
    # 提取平移和旋转
    t = T_cam2base[:3, 3]
    R_matrix = T_cam2base[:3, :3]
    euler = R.from_matrix(R_matrix).as_euler('xyz', degrees=True)
    
    print(f"\n平移 (米): X={t[0]:.4f}, Y={t[1]:.4f}, Z={t[2]:.4f}")
    print(f"旋转 (度): Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")
    
    return T_cam2base

def project_3d_to_2d(pts_3d, K, transform):
    """将3D点投影到2D图像平面"""
    pts_cam = transform @ pts_3d
    if abs(pts_cam[2]) < 1e-10:
        raise ValueError("Z值接近零，无法投影")
    pts_2d = K @ (pts_cam[:3] / pts_cam[2])
    return pts_2d[:2]

def quat_to_rot_matrix(q):
    """四元数转旋转矩阵"""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

def pose_to_transformation(pose):
    """姿态向量转变换矩阵"""
    translation = pose[:3]
    euler_angles = pose[3:]
    rotation = R.from_euler('xyz', euler_angles, degrees=False)
    rotation_matrix = rotation.as_matrix()
    
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    return transformation_matrix

def draw_xyz_axis(color, ob_in_cam, K, scale=0.1, thickness=3, debug=True):
    """绘制XYZ坐标轴"""
    xx = np.array([scale, 0, 0, 1]).astype(float)
    yy = np.array([0, scale, 0, 1]).astype(float)
    zz = np.array([0, 0, scale, 1]).astype(float)
    origin_3d = np.array([0, 0, 0, 1])
    
    # 计算末端在相机坐标系下的位置
    tcp_in_cam = ob_in_cam @ origin_3d
    
    try:
        origin = project_3d_to_2d(origin_3d, K, ob_in_cam)
        x_point = project_3d_to_2d(xx, K, ob_in_cam)
        y_point = project_3d_to_2d(yy, K, ob_in_cam)
        z_point = project_3d_to_2d(zz, K, ob_in_cam)
    except Exception as e:
        if debug:
            print(f"⚠ 投影失败: {e}")
            print(f"   末端在相机系: X={tcp_in_cam[0]:.3f}, Y={tcp_in_cam[1]:.3f}, Z={tcp_in_cam[2]:.3f}")
        return color.copy(), False
    
    # 检查投影结果
    for point in [origin, x_point, y_point, z_point]:
        if not np.all(np.isfinite(point)):
            if debug:
                print(f"⚠ 投影点包含无效值")
            return color.copy(), False
    
    # 转换为整数坐标
    origin = tuple(np.round(origin).astype(int))
    xx = tuple(np.round(x_point).astype(int))
    yy = tuple(np.round(y_point).astype(int))
    zz = tuple(np.round(z_point).astype(int))
    
    # 检查是否在图像范围内
    h, w = color.shape[:2]
    in_view = True
    for i, (point, name) in enumerate([(origin, "原点"), (xx, "X"), (yy, "Y"), (zz, "Z")]):
        if point[0] < 0 or point[0] >= w or point[1] < 0 or point[1] >= h:
            if debug and i == 0:  # 只打印原点的警告
                print(f"⚠ 坐标轴超出视野: {name} at ({point[0]}, {point[1]}), 图像大小: {w}x{h}")
                print(f"   末端在相机系: Z={tcp_in_cam[2]:.3f}m")
            in_view = False
    
    if not in_view:
        # 即使超出视野，也尝试绘制一个标记点显示大概位置
        tmp = color.copy()
        # 限制原点在图像内
        cx = max(0, min(w-1, origin[0]))
        cy = max(0, min(h-1, origin[1]))
        cv2.circle(tmp, (cx, cy), 15, (0, 255, 255), -1)  # 黄色圆点
        cv2.putText(tmp, "TCP (Out of View)", (cx+20, cy), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return tmp, False
    
    tmp = color.copy()
    # 绘制坐标轴 - 使用更粗更明显的线条
    tmp = cv2.arrowedLine(tmp, origin, xx, (0, 0, 255), thickness+2, cv2.LINE_AA, tipLength=0.3)  # X红色
    tmp = cv2.arrowedLine(tmp, origin, yy, (0, 255, 0), thickness+2, cv2.LINE_AA, tipLength=0.3)  # Y绿色
    tmp = cv2.arrowedLine(tmp, origin, zz, (255, 0, 0), thickness+2, cv2.LINE_AA, tipLength=0.3)  # Z蓝色
    
    # 添加标签
    cv2.putText(tmp, "X", xx, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(tmp, "Y", yy, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(tmp, "Z", zz, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    
    if debug:
        print(f"✓ 坐标轴绘制成功: 原点=({origin[0]}, {origin[1]})")
    
    return tmp, True

def visualize_calibration(camera_sn, robot_ip, calib_file):
    """实时可视化验证标定质量"""
    print_header("开始可视化验证")
    
    # 加载标定
    T_cam2base = load_calibration(calib_file)
    
    # 初始化硬件
    print("\n初始化硬件...")
    camera = RealSenseImage(SN_number=camera_sn)
    robot = ArmControl(ip=robot_ip)
    
    # 获取相机内参
    intrinsics = camera.o3d_intrinsics
    K = intrinsics.intrinsic_matrix
    
    print("✓ 硬件初始化完成")
    print("\n操作说明:")
    print("  - 移动机器人末端到不同位置")
    print("  - 观察坐标轴是否准确跟随末端")
    print("  - 按 ESC 键退出")
    print("  - 按 's' 键保存当前画面")
    
    cv2.namedWindow("Calibration Verification", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration Verification", 800, 600)
    
    frame_count = 0
    save_count = 0
    try:
        while True:
            frame_count += 1
            # 获取相机画面
            color_image = camera.capture_rgb_frame()
            
            # 获取机器人位姿
            tcp_pose = robot.get_current_pose()
            T_tcp2base = pose_to_transformation(tcp_pose)
            
            # 计算 tcp 在相机系下的位姿
            T_tcp2cam = np.linalg.inv(T_cam2base) @ T_tcp2base
            
            # 绘制坐标轴
            vis_img, axis_visible = draw_xyz_axis(color_image, T_tcp2cam, K, scale=0.08, thickness=4, debug=(frame_count % 30 == 0))
            
            # 显示机器人位姿信息
            pose_text = f"TCP: X={tcp_pose[0]:.3f} Y={tcp_pose[1]:.3f} Z={tcp_pose[2]:.3f}"
            cv2.putText(vis_img, pose_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 显示坐标轴状态
            status_text = "Axis: VISIBLE" if axis_visible else "Axis: OUT OF VIEW"
            status_color = (0, 255, 0) if axis_visible else (0, 0, 255)
            cv2.putText(vis_img, status_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # 显示提示
            cv2.putText(vis_img, "ESC: Exit | S: Save | Move robot to see axis", (10, vis_img.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # 转换为BGR显示
            vis_img_bgr = cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR)
            cv2.imshow("Calibration Verification", vis_img_bgr)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('s'):
                save_path = f"calibration_verify_{save_count:03d}.png"
                cv2.imwrite(save_path, vis_img_bgr)
                print(f"✓ 保存画面到 {save_path}")
                save_count += 1
    
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        cv2.destroyAllWindows()
        robot.close()
        print("\n✓ 验证完成")

def main():
    print_header("标定结果验证工具")
    
    print("可用的标定结果:")
    print("  1. 右臂 (camera_calibration_right_arm_20251226-234855.npz)")
    print("  2. 左臂 (camera_calibration_left_arm_20251226-234053.npz)")
    
    choice = input("\n选择要验证的标定 (1/2): ").strip()
    
    if choice == '1':
        calib_file = "calibration_results/camera_calibration_right_arm_20251226-234855.npz"
        camera_sn = "046322250624"
        robot_ip = "192.168.100.101"
        print("\n已选择: 右臂标定")
    elif choice == '2':
        calib_file = "calibration_results/camera_calibration_left_arm_20251226-234053.npz"
        camera_sn = "046322250624"
        robot_ip = "192.168.100.100"
        print("\n已选择: 左臂标定")
    else:
        print("无效选择")
        return
    
    # 开始可视化验证
    visualize_calibration(camera_sn, robot_ip, calib_file)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

