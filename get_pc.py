import cv2
import open3d as o3d
from PIL import Image
import numpy as np
from robot_libs.realsense_image_module import generate_pcd

# === 点云生成测试脚本 ===
# 作用：测试从RGB图像和深度图生成3D点云的功能
# 依赖：robot_libs.realsense_image_module 中的 generate_pcd 函数

# 1. 读取深度图
# 必须使用 IMREAD_UNCHANGED 标志，以保留16位深度信息 (单位通常是毫米)
loaded_depth = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)  # 保持原始位深
print("加载的深度图形状:", loaded_depth.shape)
print(loaded_depth.dtype)

# 2. 读取RGB彩色图像
loaded_image = np.array(Image.open("image.png"))
print("loaded_depth_shape: ", loaded_image.shape)

# 3. 读取相机内参
# 相机内参决定了2D像素坐标如何映射到3D空间
# camera_intrinsics.json 应包含 fx, fy, cx, cy 等参数
loaded_intrinsics = o3d.io.read_pinhole_camera_intrinsic("camera_intrinsics.json")

# 4. 生成并可视化点云
# generate_pcd 会结合RGB颜色、深度信息和相机内参，生成 Open3D 的 PointCloud 对象
# visualize_flag=True 会弹出一个 3D 窗口显示点云
pcd = generate_pcd(loaded_image, loaded_depth, loaded_intrinsics, visualize_flag=True)  




