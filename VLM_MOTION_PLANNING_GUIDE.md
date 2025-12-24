# VLM + Motion Planning 实施指南

## 📋 概述

本指南帮助你在真实机器人系统上实现**视觉-语言模型(VLM) + 运动规划(Motion Planning)**的衣物折叠任务。

参考 Isaac Sim 的 `Fold_Tops_HALO_mp.py`，完整流程为：
1. **VLM** 分析RGB图像，生成折叠计划 + 关键点2D边界框
2. **点云投影** 将2D边界框转换为3D抓取点
3. **Motion Planning** 执行抓取-移动-放置动作原语

---

## 🔧 硬件要求

- ✅ RealSense 深度相机
- ✅ 双臂机械臂 (RealMan)
- ✅ 双灵巧手 (如意手)
- ✅ AprilTag 标定板 (75mm)

---

## 📦 软件依赖

### 基础依赖
```bash
pip install numpy opencv-python open3d scipy pyserial pymodbus
```

### VLM 依赖
```bash
pip install openai  # 用于调用 OpenAI 兼容的 VLM API
```

### SAM 分割 (可选，用于初始分割)
```bash
pip install segment-anything
# 下载 SAM 权重:
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

---

## 🚀 完整实施流程

### **阶段 1: 手眼标定** ⭐ (必须先完成)

**目标**: 获得相机坐标系到机器人基座坐标系的变换矩阵 `T_cam2base`

#### 1.1 准备工作
- 准备一个 **AprilTag 标签** (75mm 大小，tag25h9 family)
- 将标签固定在机械臂工作空间内的平面上
- 确保相机能清晰看到标签

#### 1.2 运行标定程序
```bash
python camera_calibreate.py
```

#### 1.3 操作步骤
1. 程序启动后会显示相机画面
2. 移动机械臂到不同位置，确保 AprilTag 在视野内
3. 按 **'c'** 键保存当前位姿
4. 重复 15-20 次，尽量覆盖不同角度和距离
5. 完成后程序自动计算并保存标定结果

#### 1.4 输出文件
- `calibration_results/camera_calibration_XXXXXX.npz` - 变换矩阵
- `calibration_results/camera_info_XXXXXX.json` - 元数据
- `calibration_images/pose_X.jpg` - 验证图片

#### 1.5 验证标定质量
```bash
python vis_cam_to_base.py
```

观察机械臂末端坐标轴是否正确投影到图像上。如果偏差很大，需要重新标定。

---

### **阶段 2: 点云获取与分割**

**目标**: 从RGB-D图像中提取衣物的3D点云

#### 2.1 测试点云生成
```bash
python get_pc.py
```

确保能够正确生成点云。

#### 2.2 使用 SAM 分割衣物
```bash
python get_garment_pointcloud.py
```

**操作步骤**:
1. 程序显示相机画面
2. **左键点击** 衣物区域（正样本）
3. **右键点击** 背景区域（负样本）
4. 按 **空格** 确认分割
5. 程序生成并保存衣物点云

**输出文件**:
- `garment_pointcloud.ply` - 衣物点云
- `garment_mask.png` - 分割掩码
- `captured_color.png` - RGB 图像

---

### **阶段 3: VLM 配置**

**目标**: 配置并测试 VLM API 连接

#### 3.1 启动本地 VLM 服务 (示例)

如果使用本地部署的 VLM (如 Qwen-VL):
```bash
# 使用 vLLM 部署
python -m vllm.entrypoints.openai.api_server \
    --model Qwen/Qwen-VL-Chat \
    --port 8000
```

#### 3.2 配置 API 参数

编辑 `vlm_motion_planning_main.py` 中的 CONFIG:
```python
CONFIG = {
    "vlm_base_url": "http://localhost:8000/v1",  # VLM API 地址
    "vlm_model_name": "qwen-vl-plus",  # 模型名称
    # ...
}
```

#### 3.3 测试 VLM
```bash
python vlm_interface.py captured_color.png
```

检查 VLM 是否能正确返回 JSON 格式的 plan 和 points。

---

### **阶段 4: 2D→3D 投影测试**

**目标**: 验证 VLM 输出的 2D bbox 能正确转换为 3D 点

#### 4.1 准备测试数据
确保以下文件存在:
- `captured_color.png` (RGB 图像)
- `garment_pointcloud.ply` (衣物点云)
- `calibration_results/camera_calibration_XXXXXX.npz` (标定结果)

#### 4.2 运行投影测试

编辑 `bbox_to_3d.py`，更新标定文件路径:
```python
calib_file = "calibration_results/camera_calibration_20250423-204545.npz"  # 你的文件
```

运行测试:
```bash
python bbox_to_3d.py
```

检查输出的 3D 坐标是否合理（在机器人工作空间内）。

---

### **阶段 5: Motion Primitives 测试**

**目标**: 在真实机器人上测试基本动作原语

#### 5.1 安全测试环境
⚠️ **重要**: 在测试运动前，确保:
- 机器人工作空间内无障碍物
- 有紧急停止按钮
- 第一次运行时速度设置较低

#### 5.2 测试单臂抓取
创建测试脚本 `test_motion.py`:
```python
from robot_libs.realman_arm_module import ArmControl
from robot_libs.aoyi_hand_module_modbus import Hand
from motion_primitives import MotionPrimitives
import numpy as np

# 初始化
left_arm = ArmControl(ip="192.168.100.100")
right_arm = ArmControl(ip="192.168.100.101")
left_hand = Hand(port="/dev/ttyUSB0")
right_hand = Hand(port="/dev/ttyUSB1")

motion = MotionPrimitives(left_arm, right_arm, left_hand, right_hand)

# 测试回到 home 位置
motion.move_to_home()

# 测试单臂抓取移动 (修改坐标为你的工作空间内的安全位置)
start_pos = np.array([0.3, 0.2, 0.1])  # 相对基座的坐标
target_pos = np.array([0.3, 0.3, 0.1])

motion.grasp_and_move("left", start_pos, target_pos, lift_height=0.15)

# 清理
left_arm.close()
right_arm.close()
```

**逐步验证**:
1. 先测试 `move_to_home()`
2. 再测试 `grasp_and_move()` 的每个阶段
3. 调整坐标和速度参数

---

### **阶段 6: 完整流程运行**

**目标**: 运行完整的 VLM + Motion Planning 流程

#### 6.1 更新配置

编辑 `vlm_motion_planning_main.py`，更新所有配置:
```python
CONFIG = {
    # Hardware (更新为你的硬件参数)
    "camera_sn": "134222070573",
    "left_arm_ip": "192.168.100.100",
    "right_arm_ip": "192.168.100.101",
    "left_hand_port": "/dev/ttyUSB0",
    "right_hand_port": "/dev/ttyUSB1",
    
    # Calibration (更新为你的标定文件)
    "calib_file": "calibration_results/camera_calibration_20250423-204545.npz",
    
    # VLM (更新为你的 VLM 配置)
    "vlm_base_url": "http://localhost:8000/v1",
    "vlm_model_name": "qwen-vl-plus",
    
    # ...
}
```

#### 6.2 运行主程序
```bash
python vlm_motion_planning_main.py
```

#### 6.3 预期行为
1. 程序初始化硬件和模型
2. 移动到 home 位置
3. 循环执行:
   - 拍照观测当前状态
   - VLM 生成下一步折叠动作
   - 将 2D bbox 转换为 3D 点
   - 执行运动原语
   - 返回 home 位置
4. 任务完成或达到最大步数后结束

#### 6.4 调试模式

程序会自动保存调试信息到 `debug_output/TIMESTAMP/`:
- `step_XX_raw.png` - 原始 RGB 图像
- `step_XX_bbox.png` - 带 VLM bbox 的可视化
- `step_XX_vlm_output.json` - VLM 原始输出
- `step_999_final.png` - 最终状态

---

## 🛠️ 常见问题排查

### 1. 标定质量差
**症状**: `vis_cam_to_base.py` 中坐标轴投影偏差很大

**解决方案**:
- 重新标定，增加采样位姿数量 (20+)
- 确保 AprilTag 检测质量高 (光照充足、标签平整)
- 采样位姿应覆盖不同角度和距离

### 2. VLM 返回格式错误
**症状**: 程序报错 "Cannot find JSON array in VLM output"

**解决方案**:
- 检查 VLM 模型是否正确加载
- 调整 prompt (修改 `vlm_interface.py` 中的 `VLM_FOLD_PROMPT`)
- 尝试不同的 VLM 模型

### 3. 3D 点坐标不合理
**症状**: 投影得到的 3D 点超出工作空间或为负值

**解决方案**:
- 检查标定矩阵 `T_cam2base` 是否正确
- 确认点云坐标系与相机坐标系一致
- 验证相机内参是否正确

### 4. 运动执行失败
**症状**: 机械臂报错或停止

**解决方案**:
- 检查目标位置是否在机械臂可达空间内
- 降低运动速度 (`motion_primitives.py` 中的 `speed` 参数)
- 检查机械臂关节限位

### 5. 衣物分割不准确
**症状**: SAM 分割包含大量背景

**解决方案**:
- 调整拍照角度和光照
- 使用更多的正负样本点
- 尝试不同的 SAM 模型 (vit_h, vit_l, vit_b)

---

## 📊 性能优化建议

### 1. 提高 VLM 推理速度
- 使用量化模型 (INT8/INT4)
- 使用 vLLM 等推理加速框架
- 部署在 GPU 服务器上

### 2. 提高运动规划速度
- 优化运动轨迹 (减少不必要的 waypoints)
- 使用更快的插补算法
- 并行化双臂运动

### 3. 提高整体成功率
- 增加中间状态检查 (例如抓取成功检测)
- 添加失败恢复机制
- 多次尝试机制

---

## 📝 代码结构总览

```
psibot_control/
├── vlm_motion_planning_main.py     # 主程序 (入口)
├── vlm_interface.py                # VLM 接口
├── bbox_to_3d.py                   # 2D→3D 投影
├── motion_primitives.py            # 运动原语
├── get_garment_pointcloud.py       # 点云分割
│
├── camera_calibreate.py            # 手眼标定
├── vis_cam_to_base.py              # 标定验证
├── get_pc.py                       # 点云测试
│
├── robot_libs/                     # 硬件接口库
│   ├── realman_arm_module.py      # 机械臂控制
│   ├── aoyi_hand_module_modbus.py # 灵巧手控制
│   └── realsense_image_module.py  # 相机接口
│
└── calibration_results/            # 标定结果 (自动生成)
    └── camera_calibration_XXXXXX.npz
```

---

## 🎯 下一步改进方向

1. **在线学习**: 根据执行结果微调 VLM
2. **碰撞检测**: 添加实时碰撞避免
3. **力控**: 使用力传感器实现柔顺抓取
4. **多视角**: 融合多个相机的观测
5. **泛化性**: 支持更多衣物类型 (裤子、裙子等)

---

## 📞 支持

如有问题，请参考:
- Isaac Sim 原始代码: `Fold_Tops_HALO_mp.py`
- 本项目 README: `快速开始.md`
- 相关论文和文档

---

**祝实验顺利! 🚀**

