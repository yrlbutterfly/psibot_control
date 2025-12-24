# SAM 2.1 设置和使用指南

## 📌 简介

本项目已升级到使用 **Segment Anything Model 2.1 (SAM 2.1)**，相比 SAM 1.0 有以下优势：
- ✅ 更高的分割精度
- ✅ 更快的推理速度
- ✅ 更好的交互体验
- ✅ 支持视频分割
- ✅ 更强的泛化能力

## 📦 安装步骤

### 1. 下载模型文件

模型文件大小约 **856 MB**，使用 `aria2c` 多线程下载：

```bash
cd /home/psibot/Documents/psibot_control

# 下载 SAM 2.1 Hiera Large 模型
aria2c -x 16 -s 16 \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt
```

或使用 `wget`：

```bash
wget https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt
```

### 2. 安装 Python 依赖

安装 SAM 2 和相关依赖：

```bash
# 激活虚拟环境（如果有的话）
conda activate psibot  # 或者 source venv/bin/activate

# 安装依赖
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install git+https://github.com/facebookresearch/segment-anything-2.git
```

或者使用项目的 requirements.txt：

```bash
pip install -r requirements.txt
```

### 3. 验证安装

```bash
python -c "from sam2.build_sam import build_sam2; print('SAM 2 安装成功！')"
```

## 🎯 使用方法

### 基础测试：图像分割

运行基础分割测试：

```bash
python sam.py
```

程序会：
1. 加载 `image.png`
2. 弹出窗口让你点击要分割的物体
3. 显示分割结果

### 获取衣物点云

结合深度相机获取衣物的 3D 点云：

```bash
python get_garment_pointcloud.py
```

操作步骤：
1. 程序启动相机并加载 SAM 2.1
2. 拍摄当前场景
3. **左键点击**衣物区域（添加前景点）
4. **右键点击**背景区域（排除背景）
5. 按 **空格键** 确认分割
6. 生成并保存点云文件

### 完整的视觉-运动规划流程

运行完整的 demo：

```bash
python main_planning_demo.py
```

## 📁 重要文件说明

| 文件 | 用途 |
|------|------|
| `sam2.1_hiera_large.pt` | SAM 2.1 模型权重（需下载） |
| `sam.py` | 基础图像分割测试脚本 |
| `get_garment_pointcloud.py` | 衣物点云提取工具 |
| `main_planning_demo.py` | 完整的运动规划 demo |
| `vlm_motion_planning_main.py` | VLM + 运动规划主程序 |

**注意**：配置文件 `sam2.1_hiera_l.yaml` 已包含在 SAM 2 Python 包中，无需手动创建！

## 🔧 配置说明

### 在代码中使用 SAM 2.1

```python
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# 配置（按照官方示例）
checkpoint = "./sam2.1_hiera_large.pt"
model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"

# 选择设备
device = "cuda" if torch.cuda.is_available() else "cpu"

# 加载模型 - 使用官方配置路径
sam2_model = build_sam2(model_cfg, checkpoint, device=device)
predictor = SAM2ImagePredictor(sam2_model)

# 使用（和 SAM 1.0 API 类似）
predictor.set_image(image)
masks, scores, logits = predictor.predict(
    point_coords=input_points,
    point_labels=input_labels,
    multimask_output=False
)
```

### GPU vs CPU

- **GPU（推荐）**：分割速度快，适合实时应用
  ```python
  device = "cuda"
  ```

- **CPU（备用）**：如果没有 GPU，仍然可用但较慢
  ```python
  device = "cpu"
  ```

## 🆚 SAM 1.0 vs SAM 2.1 对比

| 特性 | SAM 1.0 | SAM 2.1 |
|------|---------|---------|
| 模型文件 | sam_vit_h_4b8939.pth (2.4GB) | sam2.1_hiera_large.pt (856MB) |
| 包名 | segment_anything | sam2 |
| 推理速度 | 基准 | **快 2-3 倍** |
| 分割精度 | 优秀 | **更优秀** |
| 视频支持 | ❌ | ✅ |
| API | SamPredictor | SAM2ImagePredictor |

## 🐛 常见问题

### Q1: 下载速度太慢怎么办？

使用 `aria2c` 多线程下载，或者使用国内镜像：

```bash
# 如果有代理
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
```

### Q2: 导入错误 "ModuleNotFoundError: No module named 'sam2'"

确保已安装 SAM 2：

```bash
pip install git+https://github.com/facebookresearch/segment-anything-2.git
```

### Q3: CUDA out of memory 错误

如果 GPU 内存不足，可以：
1. 切换到 CPU 模式（修改 `device = "cpu"`）
2. 或使用更小的模型（需要下载其他版本）

### Q4: 配置文件相关问题

**不需要手动创建配置文件**！SAM 2 包已经包含了所有需要的配置。使用官方路径 `"configs/sam2.1/sam2.1_hiera_l.yaml"`，这个路径是相对于 SAM 2 包的安装目录，会自动找到正确的配置文件。

### Q5: ImportError 或配置文件找不到

如果遇到配置文件找不到的错误，可能是安装方式的问题。确保：

```bash
# 从源码安装（推荐）
pip install git+https://github.com/facebookresearch/segment-anything-2.git
```

这样安装会包含完整的 `configs/` 目录。

## 📚 更多资源

- [SAM 2 官方仓库](https://github.com/facebookresearch/segment-anything-2)
- [SAM 2 论文](https://ai.meta.com/research/publications/sam-2-segment-anything-in-images-and-videos/)
- [模型下载页面](https://github.com/facebookresearch/segment-anything-2#model-checkpoints)

## ✅ 升级完成检查清单

- [x] 下载 `sam2.1_hiera_large.pt` 模型文件
- [x] 安装 SAM 2 Python 包
- [x] 更新 `requirements.txt`
- [x] 更新 `sam.py`
- [x] 更新 `get_garment_pointcloud.py`
- [x] 更新 `main_planning_demo.py`
- [x] 更新 `vlm_motion_planning_main.py`
- [ ] 运行测试验证功能正常

**注意**：无需手动创建配置文件，SAM 2 包已包含所有配置！

## 🚀 下一步

模型下载完成后：

1. **测试 SAM 2.1**：
   ```bash
   python sam.py
   ```

2. **测试点云提取**：
   ```bash
   python get_garment_pointcloud.py
   ```

3. **运行完整 demo**：
   ```bash
   python main_planning_demo.py
   ```

祝你使用愉快！🎉

