import cv2
import numpy as np
import apriltag
from scipy.spatial.transform import Rotation
import logging
import time
import os
import json
import sys

# Add parent directory to Python path to find robot_libs
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from realsense_image_module import Image
# from realman_arm_module import ArmControl
from robot_libs.realsense_image_module import RealSenseImage
from robot_libs.realman_arm_module import ArmControl


class RealManCameraCalibrator:
    def __init__(self, camera_sn="013222072349", robot_ip="192.168.100.100"):
        # 初始化RealSense相机
        self.camera = RealSenseImage(SN_number=camera_sn)

        # 初始化机器人臂
        self.robot = ArmControl(ip=robot_ip)

        # AprilTag检测器设置
        self.tag_size = 0.075  # 75mm标签尺寸
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families='tag25h9')
        )

        # 获取相机内参
        intrinsics = self.camera.get_intrinsics()
        self.camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([
            intrinsics.coeffs[0], intrinsics.coeffs[1],
            intrinsics.coeffs[2], intrinsics.coeffs[3], 0
        ]).reshape(5, 1)

        # 初始化日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def detect_apriltag(self, image):
        """在图像中检测AprilTag"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        if not results:
            return None, None, None

        # 如果检测到多个标签，选择最大的一个
        if len(results) > 1:
            results.sort(key=lambda x: x.decision_margin, reverse=True)

        tag = results[0]
        corners = tag.corners
        pose_R, pose_t = self.estimate_pose(corners)
        return corners, pose_R, pose_t

    def estimate_pose(self, corners):
        """使用PnP从标签角点估算姿态"""
        half_size = self.tag_size / 2
        # 标签角点的3D坐标 (顺序: 左上, 右上, 右下, 左下)
        object_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ])

        image_points = corners.reshape(-1, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )

        if not success:
            return None, None

        # 将旋转向量转换为旋转矩阵
        R_tag2cam, _ = cv2.Rodrigues(rvec)
        t_tag2cam = tvec

        return R_tag2cam, t_tag2cam

    def collect_calibration_data(self, num_poses=15):
        """收集标定数据，带有可视化和按键保存功能"""
        calibration_data = []
        pose_count = 0

        # 创建显示窗口
        cv2.namedWindow("AprilTag Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("AprilTag Detection", 800, 600)

        print("\n=== 标定数据采集 ===")
        print("1. 将机器人移动到不同位置")
        print("2. 确保AprilTag标签在相机视野内")
        print("3. 按 'c' 键保存当前数据")
        print("4. 按 'q' 键退出采集过程")
        print(f"目标: 收集 {num_poses} 个不同位姿\n")

        last_pose = None

        while pose_count < num_poses:
            if last_pose is None:
                print("请移动机器人到初始位置...")
            else:
                print(f"已采集: {pose_count}/{num_poses} - 移动机器人到新位置...")

            while True:
                # 拍摄图像并检测AprilTag
                color_frame = self.camera.capture_rgb_frame()
                corners, tag_R, tag_t = self.detect_apriltag(color_frame)

                # 绘制检测结果
                vis_img = color_frame.copy()

                if corners is not None:
                    # 绘制AprilTag检测结果
                    for i, corner in enumerate(corners):
                        pt = tuple(corner.astype(int))
                        cv2.circle(vis_img, pt, 5, (0, 255, 0), -1)
                        cv2.putText(vis_img, str(i), (pt[0] + 10, pt[1] + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                    # 绘制坐标轴
                    axis_length = self.tag_size
                    rvec, _ = cv2.Rodrigues(tag_R)
                    axes_points, _ = cv2.projectPoints(
                        np.array([
                            [0, 0, 0],
                            [axis_length, 0, 0],
                            [0, axis_length, 0],
                            [0, 0, axis_length]
                        ]),
                        rvec, tag_t, self.camera_matrix, self.dist_coeffs
                    )

                    origin = tuple(axes_points[0].ravel().astype(int))
                    x_end = tuple(axes_points[1].ravel().astype(int))
                    y_end = tuple(axes_points[2].ravel().astype(int))
                    z_end = tuple(axes_points[3].ravel().astype(int))

                    cv2.line(vis_img, origin, x_end, (0, 0, 255), 2)  # X轴 (红色)
                    cv2.line(vis_img, origin, y_end, (0, 255, 0), 2)  # Y轴 (绿色)
                    cv2.line(vis_img, origin, z_end, (255, 0, 0), 2)  # Z轴 (蓝色)

                # 显示机器人当前位姿
                robot_pose = self.robot.get_current_pose()
                pose_text = f"X: {robot_pose[0]:.3f}, Y: {robot_pose[1]:.3f}, Z: {robot_pose[2]:.3f}"
                cv2.putText(vis_img, pose_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                rpy_text = f"R: {robot_pose[3]:.3f}, P: {robot_pose[4]:.3f}, Y: {robot_pose[5]:.3f}"
                cv2.putText(vis_img, rpy_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if tag_t is not None:
                    camera_frame_pose_text = f"cam_X: {tag_t[0][0]:.3f}, cam_Y: {tag_t[1][0]:.3f}, cam_Z: {tag_t[2][0]:.3f}"
                    cv2.putText(vis_img, camera_frame_pose_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255),
                                2)

                # 显示当前已采集的数量
                cv2.putText(vis_img, f"collected: {pose_count}/{num_poses}", (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                # 显示操作指南
                cv2.putText(vis_img, "press 'c' to save current pose", (10, vis_img.shape[0] - 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(vis_img, "press 'q' to exit", (10, vis_img.shape[0] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # 显示图像
                cv2.imshow("AprilTag Detection", vis_img)
                key = cv2.waitKey(1)

                if key == ord('q'):
                    break

                if key == ord('c'):
                    # 只有当检测到标签时才保存
                    if corners is None or tag_R is None or tag_t is None:
                        print("未检测到有效的AprilTag，请调整机器人位置。")
                        continue

                    # 获取当前机器人位姿
                    robot_pose = self.robot.get_current_pose()

                    # 如果是第一个位姿或与上一个位姿差异足够大才保存
                    if last_pose is None:
                        diff_threshold = 0.0
                    else:
                        pos_diff = np.linalg.norm(np.array(robot_pose[0:3]) - np.array(last_pose[0:3]))
                        rot_diff = np.linalg.norm(np.array(robot_pose[3:6]) - np.array(last_pose[3:6]))
                        diff_threshold = 0.05 if pos_diff > 0.05 or rot_diff > 0.05 else 1.0

                    if diff_threshold < 1.0:
                        # 将旋转部分转换为旋转矩阵
                        robot_rpy = robot_pose[3:6]  # Roll, Pitch, Yaw
                        robot_R = Rotation.from_euler('xyz', robot_rpy).as_matrix()
                        robot_t = np.array(robot_pose[0:3]).reshape(3, 1)

                        # 保存标定数据
                        calibration_data.append({
                            'robot_pose': robot_pose,
                            'robot_R': robot_R,
                            'robot_t': robot_t,
                            'tag_corners': corners,
                            'tag_R': tag_R,
                            'tag_t': tag_t
                        })

                        pose_count += 1
                        last_pose = robot_pose
                        print(f"✓ 成功保存位姿 {pose_count}/{num_poses}")

                        # 保存可视化图像以便检查
                        save_dir = "calibration_images"
                        os.makedirs(save_dir, exist_ok=True)
                        cv2.imwrite(f"{save_dir}/pose_{pose_count}.jpg", vis_img)
                    else:
                        print("位姿与前一个过于相似，请移动机器人到新位置。")

                    # 如果已经收集足够的数据，跳出内循环
                    if pose_count >= num_poses:
                        break

            # 如果按q退出了内循环
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        print(f"已成功收集 {pose_count} 个位姿的标定数据。")
        return calibration_data

    def calibrate_camera(self, calibration_data):
        """校准相机位置，直接计算cam2base变换"""
        print("\n===== 开始相机标定 =====")

        # 收集数据准备标定
        base2gripper_R = []  # 基座到机器人末端的旋转
        base2gripper_t = []  # 基座到机器人末端的平移
        tag2cam_R = []  # 标签到相机的旋转
        tag2cam_t = []  # 标签到相机的平移

        # 收集所有位姿数据并进行必要的转换
        for data in calibration_data:
            # 获取并反转 gripper2base -> base2gripper
            gripper2base_R = data['robot_R']
            gripper2base_t = data['robot_t']

            # 计算基座到末端的变换(反转)
            base2gripper_R.append(gripper2base_R.T)
            base2gripper_t.append(-gripper2base_R.T @ gripper2base_t)

            # 标签到相机的变换(不需要反转)
            tag2cam_R.append(data['tag_R'])
            tag2cam_t.append(data['tag_t'])

        # OpenCV需要旋转向量而不是旋转矩阵
        base2gripper_Rvecs = [cv2.Rodrigues(R)[0] for R in base2gripper_R]
        tag2cam_Rvecs = [cv2.Rodrigues(R)[0] for R in tag2cam_R]

        # 使用不同方法计算cam2base变换，打印结果
        methods = {
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF
        }

        best_method = 'PARK'  # 根据参考代码，PARK方法效果较好
        R_cam2base, t_cam2base = None, None

        for method_name, method_id in methods.items():
            R, t = cv2.calibrateHandEye(
                base2gripper_Rvecs,  # 基座到末端的旋转
                base2gripper_t,  # 基座到末端的平移
                tag2cam_Rvecs,  # 标签到相机的旋转
                tag2cam_t,  # 标签到相机的平移
                method=method_id  # 标定方法
            )

            print(f"方法 {method_name}:")
            print(f"R_cam2base:\n{R}")
            print(f"t_cam2base:\n{t}")
            print("-" * 40)

            if method_name == best_method:
                R_cam2base = R
                t_cam2base = t

        # 创建cam2base变换矩阵
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()

        print(f"\n已选择 {best_method} 方法的结果作为最终的相机到基座变换 (cam2base)")

        # 为了兼容性，也计算cam2gripper
        T_gripper2base = np.eye(4)
        last_data = calibration_data[-1]
        T_gripper2base[:3, :3] = last_data['robot_R']
        T_gripper2base[:3, 3] = last_data['robot_t'].flatten()

        # cam2gripper = inv(gripper2base) @ cam2base
        T_base2gripper = np.linalg.inv(T_gripper2base)
        T_cam2gripper = T_base2gripper @ T_cam2base

        R_cam2gripper = T_cam2gripper[:3, :3]
        t_cam2gripper = T_cam2gripper[:3, 3].reshape(3, 1)

        # 返回结果
        return {
            'R_cam2gripper': R_cam2gripper,
            't_cam2gripper': t_cam2gripper,
            'T_cam2gripper': T_cam2gripper,
            'R_cam2base': R_cam2base,
            't_cam2base': t_cam2base,
            'T_cam2base': T_cam2base,
            'method': best_method
        }

    def save_calibration(self, calibration_result):
        """保存标定结果到文件"""
        # 创建保存目录
        os.makedirs('calibration_results', exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"calibration_results/camera_calibration_{timestamp}.npz"

        np.savez(
            filename,
            # 相机到末端的变换 (cam2gripper)
            R_cam2gripper=calibration_result['R_cam2gripper'],
            t_cam2gripper=calibration_result['t_cam2gripper'],
            T_cam2gripper=calibration_result['T_cam2gripper'],
            # 相机到基座的变换 (cam2base) - 这是主要结果
            R_cam2base=calibration_result['R_cam2base'],
            t_cam2base=calibration_result['t_cam2base'],
            T_cam2base=calibration_result['T_cam2base'],
            # 相机参数
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            tag_size=self.tag_size,
        )

        # 保存人类可读的JSON
        calibration_info = {
            "timestamp": timestamp,
            "tag_size_meters": float(self.tag_size),
        }

        json_filename = f"calibration_results/camera_info_{timestamp}.json"
        with open(json_filename, 'w') as f:
            json.dump(calibration_info, f, indent=4)

        print(f"标定结果已保存到 {filename}")

        # 打印结果
        print("\n===== 相机到机器人末端的变换 (cam2gripper) =====")
        print("4x4变换矩阵:")
        print(calibration_result['T_cam2gripper'])

        # 将旋转转换为欧拉角 (以度为单位)
        euler_angles = Rotation.from_matrix(calibration_result['R_cam2gripper']).as_euler('xyz', degrees=True)
        print("欧拉角 (度):")
        print(f"Roll (X): {euler_angles[0]:.2f}°, Pitch (Y): {euler_angles[1]:.2f}°, Yaw (Z): {euler_angles[2]:.2f}°")

        print("\n===== 相机到机器人基座的变换 (cam2base) =====")
        print("4x4变换矩阵:")
        print(calibration_result['T_cam2base'])

        # 将旋转转换为欧拉角 (以度为单位)
        base_euler = Rotation.from_matrix(calibration_result['R_cam2base']).as_euler('xyz', degrees=True)
        print("欧拉角 (度):")
        print(f"Roll (X): {base_euler[0]:.2f}°, Pitch (Y): {base_euler[1]:.2f}°, Yaw (Z): {base_euler[2]:.2f}°")

        # 获取平移向量，提取为x,y,z
        t = calibration_result['t_cam2base'].flatten()
        print(f"平移向量 (米): X: {t[0]:.4f}, Y: {t[1]:.4f}, Z: {t[2]:.4f}")

        return filename


def main():
    # 初始化标定器
    calibrator = RealManCameraCalibrator(
        camera_sn="027322071904",  # 更改为您的相机序列号
        robot_ip="192.168.100.101"  # 更改为您的机器人IP
    )

    try:
        # 收集标定数据
        print("开始收集标定数据...")
        calibration_data = calibrator.collect_calibration_data(num_poses=20)

        if len(calibration_data) < 3:
            print("收集的标定数据不足，至少需要3组数据。退出。")
            return

        # 执行相机标定，计算cam2gripper和固定的cam2base变换
        calibration_result = calibrator.calibrate_camera(calibration_data)

        if calibration_result is not None:
            # 如果误差可接受，保存标定结果
            calibrator.save_calibration(calibration_result)
        else:
            print("标定失败，无法计算变换关系。")

    finally:
        # 清理资源
        calibrator.camera.close()
        calibrator.robot.close()
        print("标定过程完成。")


if __name__ == "__main__":
    main()
