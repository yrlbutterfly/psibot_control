import pyrealsense2 as rs
import cv2
import numpy as np
import sys
from scipy.spatial.transform import Rotation as R
import open3d as o3d

sys.path.append("..")


o3d_intrinsics_path = 'o3d_intrinsics.txt'

def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels == 1]
    neg_points = coords[labels == 0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white',
               linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white',
               linewidth=1.25)


#----------------
def read_intrinsics(file_path):
    intrinsics_dict = {}
    with open(file_path, 'r') as f:
        for line in f:
            key, value = line.strip().split(": ")
            intrinsics_dict[key] = float(value) if key in ["fx", "fy", "cx", "cy"] else int(value)
    
    intrinsics = o3d.camera.PinholeCameraIntrinsic(
        width=intrinsics_dict["Width"],
        height=intrinsics_dict["Height"],
        fx=intrinsics_dict["fx"],
        fy=intrinsics_dict["fy"],
        cx=intrinsics_dict["cx"],
        cy=intrinsics_dict["cy"]
    )
    return intrinsics

def save_intrinsics(intrinsics, file_path):
    with open(file_path, 'w') as f:
        f.write(f"Width: {intrinsics.width}\n")
        f.write(f"Height: {intrinsics.height}\n")
        f.write(f"fx: {intrinsics.get_focal_length()[0]}\n")
        f.write(f"fy: {intrinsics.get_focal_length()[1]}\n")
        f.write(f"cx: {intrinsics.get_principal_point()[0]}\n")
        f.write(f"cy: {intrinsics.get_principal_point()[1]}\n")

def visualize_rgbd(color_image, depth_image):
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    
    cv2.imshow('Color Image', color_image)
    cv2.imshow('Depth Image', depth_colormap)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def generate_pcd(color_image, depth_image, intrinsics, visualize_flag=False):
    # Convert BGR to RGB
    color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    # Create Open3D images
    color_o3d = o3d.geometry.Image(color_image)
    depth_o3d = o3d.geometry.Image(depth_image)

    # Create RGBD image
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1000.0,  # Convert mm to meters
        depth_trunc=3.0,      # Truncate points beyond 3 meters
        convert_rgb_to_intensity=False
    )

    # Generate point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        intrinsics
    )
    
    if visualize_flag:
        print("Visualizing point cloud...")
        o3d.visualization.draw_geometries([pcd])

    # Adjust coordinate system
    # NOTE: Coordinate transform commented out to match external calibration data
    # If calibration was done WITHOUT this transform, keep it commented
    # If calibration was done WITH this transform, uncomment the line below
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd
#----------------



class RealSenseImage:
    def __init__(self, SN_number=None, device='cuda'):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if SN_number:
            self.config.enable_device(SN_number)  # 替换为第一个相机的序列号
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        self.profile = self.pipeline.start(self.config)

        # Get color stream intrinsics
        #----------------------------
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        color_intrinsics = color_profile.get_intrinsics()
        self.o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=color_intrinsics.width,
            height=color_intrinsics.height,
            fx=color_intrinsics.fx,
            fy=color_intrinsics.fy,
            cx=color_intrinsics.ppx,
            cy=color_intrinsics.ppy
        )
        #----------------------------

        self.align_to = rs.stream.color
        self.aligner = rs.align(self.align_to)

        self.device = device
        self.zfar = np.inf
        self.base_to_camera = self.calc_base_to_camera()

        self.stable_realsense()

    def stable_realsense(self):
        for i in range(10):
            self.capture_frame()

    def init_apriltag_detector(self):
        # 相机内参矩阵 (fx, fy 为焦距，cx, cy 为主点坐标)
        self.camera_matrix = np.array([[6.1636529541015625e+02, 0, 3.2377774047851562e+02],
                                [0, 6.1664202880859375e+02, 2.4262834167480469e+02],
                                [0, 0, 1]])

        # 相机的畸变系数，如果没有畸变，设置为零数组
        self.dist_coeffs = np.zeros((5, 1))  # 或者使用你相机标定得到的实际畸变系数

        # AprilTag的物理尺寸（单位：米），这是一个边长为 0.1 米的方形标签
        tag_size = 0.075  # 假设 AprilTag 的边长是 10 厘米

        # 定义AprilTag的4个角点在世界坐标系中的3D位置 (以中心为原点，z=0)
        self.object_points = np.array([[-tag_size / 2, -tag_size / 2, 0],  # 左上角
                                [ tag_size / 2, -tag_size / 2, 0],  # 右上角
                                [ tag_size / 2,  tag_size / 2, 0],  # 右下角
                                [-tag_size / 2,  tag_size / 2, 0]],  # 左下角
                                dtype=np.float32)

        # 初始化AprilTag检测器
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))  # 根据你使用的AprilTag家族选择
        self.apriltar_last_pose = np.array([0.5, -0.5, 0.5, -1.789, 0, -1.571])

    def capture_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.aligner.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.depth_image_np = np.asanyarray(depth_frame.get_data()).astype(np.uint16)
        self.color_image_np = np.asanyarray(color_frame.get_data())[:, :, :3]
        return self.color_image_np, self.depth_image_np

    def capture_rgb_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        self.color_image_np = np.asanyarray(color_frame.get_data())[:, :, :3]
        return self.color_image_np

    def estimate_pose(self, color_image):
        frame = color_image
        success = False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 检测AprilTag
        tags = self.at_detector.detect(gray)

        if len(tags) == 0:
            print("未检测到AprilTag标签")

        else:
            # 处理检测到的AprilTag
            for tag in tags:
                # 获取标签在图像中的4个角点（顺序是左上、右上、右下、左下）
                image_points = np.array([tag.corners[0],
                                         tag.corners[1],
                                         tag.corners[2],
                                         tag.corners[3]], dtype=np.float32)

                # 绘制 AprilTag 的边框
                for i in range(4):
                    start_point = tuple(image_points[i].astype(int))
                    end_point = tuple(image_points[(i + 1) % 4].astype(int))
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # 计算标签的位姿
                success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs)
                if success:
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    pose = np.hstack((rotation_matrix, tvec.reshape(-1, 1)))
                    pose = np.vstack((pose, np.array([0, 0, 0, 1])))

                    # 绘制坐标轴
                    frame = draw_pose_debug(frame, pose, self.camera_matrix)

        # cv2.imshow('RealSense Stream', frame)
        # cv2.waitKey(1)

        # Store pose in Redis    
        if success:
            pose_in_camera_frame = pose
            pose_in_base_frame = self.transform_pose(pose_in_camera_frame, self.base_to_camera)
            self.apriltar_last_pose = pose_in_base_frame.copy()
        else:
            pose_in_base_frame = self.apriltar_last_pose

        return pose_in_base_frame, frame, success

    def get_mask_from_color(self, color_image):
        self.lower_yellow = np.array([100, 80, 0])  # Lower bound for yellow in RGB
        self.upper_yellow = np.array([200, 180, 80])  # Upper bound for yellow in RGB

        # Create a mask for yellow color
        yellow_mask = np.all((color_image >= self.lower_yellow) & (color_image <= self.upper_yellow), axis=-1).astype(np.uint8) * 255

        # Perform connected component analysis
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(yellow_mask, connectivity=8)

        if num_labels > 1:
            # Find the largest connected component by area (excluding the background)
            largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            largest_mask = (labels == largest_label).astype(np.uint8) * 255
        else:
            largest_mask = np.zeros_like(yellow_mask)

        # Convert to boolean mask
        best_mask = (largest_mask > 0).astype(bool)
        return best_mask

    def calc_base_to_camera(self):
        qw = 0.23345722322171142
        qx = -0.9689635614174572
        qy = 0.023228428542749507
        qz = -0.07789596702166326
        tx = 0.449731057175721
        ty = -0.7906569428724092
        tz = 0.9535086112001296
        quat = [qw, qx, qy, qz]
        rotation = self.quat_to_rot_matrix(quat)
        base_to_camera = np.eye(4)
        base_to_camera[:3, :3] = rotation
        base_to_camera[:3, 3] = [tx, ty, tz]
        return base_to_camera

    def quat_to_rot_matrix(self, q):
        w, x, y, z = q
        # Compute rotation matrix elements
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
        return R

    def get_mask(self, mask):
        if len(mask.shape)==3:
            for c in range(3):
                if mask[...,c].sum()>0:
                    mask = mask[...,c]
                    break
        mask = cv2.resize(mask, (self.W,self.H), interpolation=cv2.INTER_NEAREST).astype(bool).astype(np.uint8)
        return mask

    def get_depth(self, depth):
        depth = depth/1e3
        depth = cv2.resize(depth, (self.W,self.H), interpolation=cv2.INTER_NEAREST)
        depth[(depth<0.1) | (depth>=self.zfar)] = 0
        return depth

    def get_color(self, color):
        color = color[...,:3]
        color = cv2.resize(color, (self.W,self.H), interpolation=cv2.INTER_NEAREST)
        return color

    def transform_pose(self, pose, transformation_matrix):
        transformed_pose = np.dot(transformation_matrix, pose)[:3, :]
        xyz = transformed_pose[:3, 3]
        rotation_matrix = transformed_pose[:3, :3]
        rpy = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
        xyzrpy = np.concatenate([xyz, rpy])
        return xyzrpy

    def draw_pose_debug(self, color_image, pose, block=True):
        color_image = self.get_color(color_image)
        center_pose = pose @ np.linalg.inv(self.to_origin)
        vis = draw_posed_3d_box(self.K, img=color_image, ob_in_cam=center_pose, bbox=self.bbox)
        vis = draw_xyz_axis(color_image, ob_in_cam=center_pose, scale=0.1, K=self.K, thickness=3, transparency=0, is_input_rgb=True)
        cv2.imshow("Pose Debug", vis[...,::-1])
        cv2.waitKey(1)  # Wait for key press to close

    def visualize_rgb_image(self, image, image_name="RGB Image"):
        """
        Visualize the RGB image using OpenCV.
        """
        # Convert the image from BGR to RGB if necessary
        if image.shape[2] == 4:
            image = image[:, :, :3]  # Drop alpha channel if present

        # Display the image
        cv2.imshow(image_name, image)
        cv2.waitKey(1)

    def store_pose_in_redis(self, pose):
        # Convert pose to a binary format and store in Redis
        pose_bytes = pose.astype(np.float32).tobytes()
        self.r.set("latest_pose", pose_bytes)

    def run(self):
        try:
            while True:
                # Capture images
                color_image, depth_image = self.capture_frame()
                
                # Estimate pose
                pose = self.estimate_pose(color_image, depth_image)
                
                # Store pose in Redis
                self.store_pose_in_redis(pose)
                
                # Optional: Add a delay to control the loop speed
                # time.sleep(0.01)  # Adjust as needed

        except KeyboardInterrupt:
            print("Stopping the capture loop.")
        finally:
            self.close()

    def close(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    image_processor = Image()
    init_color_image, init_depth_image = image_processor.capture_frame()
