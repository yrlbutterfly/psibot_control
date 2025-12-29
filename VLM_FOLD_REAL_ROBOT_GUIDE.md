# VLM驱动的真实机器人衣物折叠系统使用指南

## 概述

`vlm_fold_real_robot.py` 是一个完全自动化的衣物折叠系统，集成了以下功能：

1. **SAM自动分割** - 无需人工点击，自动识别衣物
2. **VLM自动规划** - 通过视觉语言模型自动生成折叠计划和关键点
3. **智能动作执行** - 根据VLM输出自动选择单臂/双臂折叠动作
4. **安全控制** - 每个阶段需要人工确认，动作分步执行
5. **完整记录** - 保存所有VLM输入输出和可视化结果

## 主要特性

### 1. 自动分割（SAM）
- 使用图像中心点作为提示，自动分割衣物
- 选择分数最高的mask作为分割结果
- 无需人工点击交互

### 2. VLM自动规划
- 输入：当前衣物的RGB图像
- 输出：
  - `plan`: 下一步折叠动作（JSON格式）
  - `points`: 关键点的bbox坐标
- 支持的关键点：
  - `left_cuff`, `right_cuff`
  - `left_hem`, `right_hem`
  - `left_shoulder`, `right_shoulder`

### 3. 智能动作执行
根据VLM的plan自动判断：
- **左手单独动作** - 调用 `mp_left_fold()`
- **右手单独动作** - 调用 `mp_right_fold()`
- **双手协同动作** - 调用 `mp_bimanual_fold()`

### 4. 安全特性
- ✅ 每个阶段开始前需要人工确认
- ✅ 动作分step执行（每步约5cm）
- ✅ 每步都可以按'q'退出
- ✅ 失败后可选择是否继续

### 5. 完整记录
在 `debug_vlm_real_robot/YYYYMMDD_HHMMSS/` 目录下保存：
- `vlm_results.txt` - VLM的原始输出
- `vlm_rgb/vlm_rgb_XXX.png` - 带bbox标注的图像
- `vlm_rgb/vlm_rgb_raw_XXX.png` - 原始RGB图像

## 环境配置

### 1. 安装依赖
```bash
pip install openai numpy opencv-python open3d
```

### 2. 设置VLM环境变量
```bash
export VLM_BASE_URL="http://localhost:8000/v1"  # VLM服务地址
export VLM_MODEL_NAME="Qwen/Qwen2-VL-7B-Instruct"  # 模型名称
export VLM_API_KEY="EMPTY"  # API密钥（本地服务可为空）
```

### 3. 准备配置文件
确保以下文件存在：
- `robot_photo_config.json` - 拍照位置的关节角度
- `robot_home_config.json` - Home位置的关节角度
- `calibration_results/camera_calibration_left_arm_*.npz` - 左臂手眼标定
- `calibration_results/camera_calibration_right_arm_*.npz` - 右臂手眼标定
- `sam2.1_hiera_large.pt` - SAM 2.1模型权重

## 使用方法

### 基本使用
```bash
python vlm_fold_real_robot.py
```

### 运行流程
程序会自动循环执行以下步骤（最多6次）：

1. **移动到拍照位置** - 自动移动机械臂到预设的Photo位置
2. **获取衣物点云** - 自动分割并生成3D点云
3. **移动回Home位置** - 避免遮挡，准备执行动作
4. **VLM推理** - 获取下一步折叠计划和关键点bbox
5. **解析VLM输出** - 提取动作类型和目标位置
6. **执行Motion Planning** - 分步执行折叠动作

每个subtask完成后会等待3秒让衣物稳定，然后进入下一轮。

### 交互操作

#### 阶段确认
在每个运动阶段前会提示：
```
[CONFIRM] Press ENTER to execute Phase X (or 'q' to quit):
```
- 按 **ENTER** 继续
- 输入 **q** 退出

#### 分步执行
在每个运动步骤中会提示：
```
Step X/N [x, y, z] >
```
- 按 **ENTER** 执行下一步
- 输入 **q** 退出

#### 失败处理
如果某个subtask失败，会提示：
```
Continue to next subtask? (y/n) [n]:
```
- 输入 **y** 继续尝试下一个subtask
- 按 **ENTER** 或输入 **n** 停止

## VLM Prompt说明

系统使用以下prompt与VLM交互：

### 输入
- RGB图像（base64编码）
- 标准T恤折叠参考步骤

### 输出格式
```json
[
  {
    "plan": [
      {
        "left": {"from": "left_cuff", "to": "right_shoulder"},
        "right": {"from": null, "to": null}
      }
    ]
  },
  {
    "points": [
      {"label": "left_cuff", "bbox": [100, 200, 150, 250]},
      {"label": "right_shoulder", "bbox": [300, 150, 350, 200]}
    ]
  }
]
```

### 标准折叠步骤
1. **Step 1**: 左袖向右肩折叠（右手闲置）
2. **Step 2**: 右袖向左肩折叠（左手闲置）
3. **Step 3**: 底部向上折叠（双手协同）

VLM每次只输出**下一步**动作，而不是完整的折叠计划。

## 配置参数

在 `main()` 函数中可以修改以下参数：

```python
# 相机配置
CAMERA_SN = "046322250624"  # RealSense相机序列号
SAM_CHECKPOINT = "sam2.1_hiera_large.pt"  # SAM模型路径

# 标定文件
CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_*.npz"
CALIB_FILE_RIGHT = "calibration_results/camera_calibration_right_arm_*.npz"

# 运动参数
LIFT_HEIGHT = 0.08  # 抬起高度（米）
MAX_SUBTASKS = 6    # 最大任务循环次数

# Debug配置
DEBUG_FLAG = True   # 是否保存调试信息
DEBUG_DIR = "debug_vlm_real_robot"  # 调试输出目录
```

## 与test_bbox_to_motion.py的区别

| 特性 | test_bbox_to_motion.py | vlm_fold_real_robot.py |
|------|------------------------|------------------------|
| 分割方式 | 人工点击 | SAM自动分割 |
| 点标注 | 人工画bbox | VLM自动输出bbox |
| 动作决策 | 人工选择单臂/双臂 | VLM自动判断 |
| 操作次数 | 单次执行 | 循环执行直到完成 |
| 可视化 | 实时显示需要确认 | 后台保存所有结果 |

## 故障排除

### 1. VLM连接失败
**错误**: `Connection refused` 或 `API key not found`

**解决方案**:
```bash
# 检查VLM服务是否运行
curl http://localhost:8000/v1/models

# 设置正确的环境变量
export VLM_BASE_URL="http://your-vlm-server:port/v1"
export VLM_API_KEY="your-api-key"
```

### 2. SAM分割失败
**错误**: `Auto-segmentation failed or mask too small`

**可能原因**:
- 衣物不在图像中心
- 衣物与背景对比度不够
- 衣物太小或太大

**解决方案**:
- 调整衣物位置到图像中心
- 改善光照条件
- 修改分割参数（在代码中调整center_point位置）

### 3. 标定文件未找到
**错误**: `Calibration files not found`

**解决方案**:
```bash
# 检查标定文件是否存在
ls calibration_results/

# 如果没有，需要先运行手眼标定
python camera_calibreate.py
```

### 4. VLM输出格式错误
**错误**: `Invalid plan format` 或 `VLM 输出 JSON 结构不符合预期`

**可能原因**:
- VLM模型输出格式不正确
- Prompt理解错误

**解决方案**:
- 检查 `debug_vlm_real_robot/*/vlm_results.txt` 查看原始输出
- 调整 `VLM_FOLD_PROMPT` 使其更加明确
- 尝试不同的VLM模型

### 5. 运动执行失败
**错误**: `Missing keypoints` 或 `Motion failed`

**可能原因**:
- VLM输出的bbox没有对应的3D点
- bbox区域内点云太少
- 目标位置超出机械臂工作空间

**解决方案**:
- 检查可视化图像中的bbox是否合理
- 调整相机角度或衣物位置
- 修改运动参数（在motion_primitives.py中）

## 高级使用

### 自定义VLM Prompt
修改 `VLM_FOLD_PROMPT` 变量来定制VLM的行为：

```python
VLM_FOLD_PROMPT = """
Your custom prompt here...
- Add specific instructions
- Modify keypoint names
- Change output format requirements
"""
```

### 调整自动分割策略
修改分割点位置以适应不同场景：

```python
# 使用多个采样点
sample_points = np.array([
    [img_width // 2, img_height // 2],      # 中心点
    [img_width // 3, img_height // 2],      # 左侧点
    [img_width * 2 // 3, img_height // 2],  # 右侧点
])
sample_labels = np.array([1, 1, 1])  # 都是前景点
```

### 扩展支持的关键点
在VLM prompt中添加新的关键点定义，并在 `execute_vlm_motion_plan` 中添加相应的处理逻辑。

## 性能优化建议

1. **VLM推理速度**: 使用量化模型或更小的模型可以加快推理速度
2. **SAM分割速度**: 考虑使用SAM的Fast版本
3. **运动执行速度**: 调整 `step_size` 和 `speed` 参数（注意安全）
4. **减少等待时间**: 根据实际情况调整 `time.sleep()` 的时长

## 未来改进方向

- [ ] 支持更多类型的衣物（裤子、毛衣等）
- [ ] 添加碰撞检测和避障
- [ ] 实现动态调整运动参数
- [ ] 支持VLM在线学习和优化
- [ ] 添加衣物状态评估（整齐度打分）
- [ ] 支持多轮对话式交互

## 参考资料

- [SAM 2.0 文档](https://github.com/facebookresearch/segment-anything-2)
- [OpenAI API 文档](https://platform.openai.com/docs/api-reference)
- [Motion Primitives 设计文档](motion_primitives.py)
- [手眼标定教程](手眼标定详细教程.md)

## 联系与支持

如有问题，请查看：
- `debug_vlm_real_robot/` 目录下的调试信息
- 终端输出的详细日志
- VLM服务器日志

建议在运行前先阅读 [快速开始.md](快速开始.md) 了解整体系统架构。

