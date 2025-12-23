import cv2
import numpy as np
from robot_libs.realsense_image_module import RealSenseImage
from robot_libs.realman_arm_module import ArmControl
from scipy.spatial.transform import Rotation as R




class VisCamToBase:
    def __init__(self, camera_sn, robot_ip):
        self.camera = RealSenseImage(SN_number=camera_sn)
        self.robot = ArmControl(ip=robot_ip)
        intrinsics = self.camera.get_intrinsics()
        self.K = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        calibration_file = "/home/admin01/program/foundationpose_controller/calibration_results/camera_calibration_20250423-204545.npz"
        self.cam_to_base = self.calc_cam_to_base(calibration_file)


    def project_3d_to_2d(self, pts_3d, K, transform):
        """
        将3D点投影到2D图像平面
        
        参数:
        pts_3d: 3D点坐标 (齐次坐标)
        K: 相机内参矩阵
        transform: 世界到相机的变换矩阵
        
        返回:
        投影后的2D坐标
        """
        # 应用变换
        pts_cam = transform @ pts_3d
        
        # 防止除以零
        if abs(pts_cam[2]) < 1e-10:
            raise ValueError("Z值接近零，无法投影")
        
        # 投影到图像平面
        pts_2d = K @ (pts_cam[:3] / pts_cam[2])
        
        return pts_2d[:2]

    def transformation_to_pose(self,Trans_tcp):
        """
        将4x4齐次变换矩阵转换为姿态向量

        参数:
            Trans_tcp: 4x4的齐次变换矩阵

        """
        Transformed_tcp = Trans_tcp[:3, :]
        xyz = Transformed_tcp[:3, 3]
        rotation_matrix = Transformed_tcp[:3, :3]
        rpy = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
        xyzrpy = np.concatenate([xyz, rpy])
        return xyzrpy
    
    def pose_to_transformation(self,pose):
        """
        将姿态向量转换为4x4旋转平移矩阵

        参数:
            pose: [x, y, z, rx, ry, rz]，其中rx, ry, rz为旋转向量

        返回:
            transformation_matrix: 4x4的齐次变换矩阵
        """
        # 提取平移和旋转分量
        translation = pose[:3]
        euler_angles = pose[3:]

        # 计算旋转矩阵（使用欧拉角）
        rotation = R.from_euler('xyz', euler_angles, degrees=False)
        rotation_matrix = rotation.as_matrix()

        # 构建4x4的齐次变换矩阵
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        return transformation_matrix


    def draw_xyz_axis(self, color, ob_in_cam, scale=0.1, K=np.eye(3), thickness=3, transparency=0, is_input_rgb=False):
        '''
        绘制xyz坐标轴
        @color: BGR格式图像
        '''
        if is_input_rgb:
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            
        xx = np.array([1, 0, 0, 1]).astype(float)
        yy = np.array([0, 1, 0, 1]).astype(float)
        zz = np.array([0, 0, 1, 1]).astype(float)
        origin_3d = np.array([0, 0, 0, 1])
        
        # 缩放坐标轴
        xx[:3] = xx[:3] * scale
        yy[:3] = yy[:3] * scale
        zz[:3] = zz[:3] * scale
        
        # 投影3D点到2D图像平面
        try:
            origin = self.project_3d_to_2d(origin_3d, K, ob_in_cam)
            x_point = self.project_3d_to_2d(xx, K, ob_in_cam)
            y_point = self.project_3d_to_2d(yy, K, ob_in_cam)
            z_point = self.project_3d_to_2d(zz, K, ob_in_cam)
        except Exception as e:
            print(f"投影时出错: {e}")
            if is_input_rgb:
                return cv2.cvtColor(color.copy(), cv2.COLOR_BGR2RGB)
            return color.copy()
        
        # 检查投影结果是否有效
        for point in [origin, x_point, y_point, z_point]:
            if not np.all(np.isfinite(point)):
                print("投影点包含无效值(NaN或Inf)")
                if is_input_rgb:
                    return cv2.cvtColor(color.copy(), cv2.COLOR_BGR2RGB)
                return color.copy()
        
        # 转换为整数坐标并创建元组
        origin = tuple(np.round(origin).astype(int))
        xx = tuple(np.round(x_point).astype(int)) 
        yy = tuple(np.round(y_point).astype(int))
        zz = tuple(np.round(z_point).astype(int))
        
        # 检查坐标是否在图像范围内
        h, w = color.shape[:2]
        for point in [origin, xx, yy, zz]:
            if point[0] < 0 or point[0] >= w or point[1] < 0 or point[1] >= h:
                print(f"投影点{point}超出图像范围")
                if is_input_rgb:
                    return cv2.cvtColor(color.copy(), cv2.COLOR_BGR2RGB)
                return color.copy()
        
        line_type = cv2.LINE_AA
        arrow_len = 0
        tmp = color.copy()
        
        # 绘制X轴（红色）
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, xx, color=(0, 0, 255), thickness=thickness, line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1) > 0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        
        # 绘制Y轴（绿色）
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, yy, color=(0, 255, 0), thickness=thickness, line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1) > 0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        
        # 绘制Z轴（蓝色）
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, zz, color=(255, 0, 0), thickness=thickness, line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1) > 0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        
        tmp = tmp.astype(np.uint8)
        if is_input_rgb:
            tmp = cv2.cvtColor(tmp, cv2.COLOR_BGR2RGB)

        return tmp

    def quat_to_rot_matrix(self, q):
        w, x, y, z = q
        # Compute rotation matrix elements
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
        return R

    def calc_cam_to_base(self, calibration_file=None):
            """
            从标定文件加载或使用默认值计算base_to_camera变换矩阵
            
            参数:
                calibration_file: 保存标定结果的.npz文件路径
            """
            if calibration_file is None:
                # 使用默认值作为后备
                print("calibration_file is None...")
            else:
                # 从.npz文件加载标定结果
                print(f"从文件加载cam2base变换: {calibration_file}")
                calib_data = np.load(calibration_file)
                
                # 获取cam2base变换矩阵
                # 方法1：如果直接保存了完整的T_cam2base矩阵
                if 'T_cam2base' in calib_data:
                    T_cam2base = calib_data['T_cam2base']
                # 方法2：如果分别保存了R和t
                elif 'R_cam2base' in calib_data and 't_cam2base' in calib_data:
                    R_cam2base = calib_data['R_cam2base']
                    t_cam2base = calib_data['t_cam2base']
                    
                    # 构建完整的变换矩阵
                    T_cam2base = np.eye(4)
                    T_cam2base[:3, :3] = R_cam2base
                    T_cam2base[:3, 3] = t_cam2base.flatten()
                else:
                    raise ValueError("标定文件格式不正确，找不到T_cam2base或R_cam2base/t_cam2base")
                
                # 取逆：从cam2base转为base2cam
                T_base2cam = T_cam2base# 注意这里返回的就是cam2base 忘记改名了 懒得改了

                # print(T_base2cam)
                # exit()
                
                # 提取旋转和平移部分
                rotation = T_base2cam[:3, :3]
                translation = T_base2cam[:3, 3]
                
                # 将旋转矩阵转换为四元数
                from scipy.spatial.transform import Rotation
                r = Rotation.from_matrix(rotation)
                quat = r.as_quat()  # 返回(x,y,z,w)顺序
                qx, qy, qz, qw = quat  # 调整为(w,x,y,z)顺序
                tx, ty, tz = translation
                
                print(f"已加载并转换cam2base -> base2cam:")
                print(f"四元数: w={qw:.6f}, x={qx:.6f}, y={qy:.6f}, z={qz:.6f}")
                print(f"平移: x={tx:.6f}, y={ty:.6f}, z={tz:.6f}")
            
            # 构建变换矩阵
            quat = [qw, qx, qy, qz]
            rotation = self.quat_to_rot_matrix(quat)
            base_to_camera = np.eye(4)
            base_to_camera[:3, :3] = rotation
            base_to_camera[:3, 3] = [tx, ty, tz]
            
            print("Base to Camera Transformation Matrix:")
            print(base_to_camera)
            return base_to_camera 


    def calc_tcp_to_cam(self,T_tcp2base,T_cam2base):
        T_tcp2cam = np.linalg.inv(T_cam2base) @ T_tcp2base
        return T_tcp2cam
    


    def vis_cam_to_base(self):
        cv2.namedWindow("tcp_to_cam", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("tcp_to_cam", 800, 600)
        
        print("开始绘制tcp在相机系下的坐标轴...")
        while True:
            color_image = self.camera.capture_rgb_frame()
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            tcp_pose = self.robot.get_current_pose()
            T_t2b = self.pose_to_transformation(tcp_pose)
            T_Tcp2Cam = self.calc_tcp_to_cam(T_t2b,self.cam_to_base)
            print("T_Tcp2Cam:",self.transformation_to_pose(T_Tcp2Cam))
            vis_img = self.draw_xyz_axis(color_image, T_Tcp2Cam,scale=0.07, K=self.K,thickness=6, transparency=0, is_input_rgb=True)
            #realsense返回的是bgr格式，这里转换成rgb处理，但是最后用cv2显示的时候需要转换成bgr，所以这里使用了[...,::-1]
            cv2.imshow("tcp_to_cam", vis_img[...,::-1])
            cv2.waitKey(1)



if __name__ == "__main__":
    vis_cam_to_base = VisCamToBase(camera_sn="949122070355", robot_ip="192.168.100.100")
    vis_cam_to_base.vis_cam_to_base()