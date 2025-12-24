# AprilTag 快速获取指南

## 🎯 需要什么

一个 **75mm × 75mm** 的 **tag25h9** 系列 AprilTag 标定板

---

## 方法 1: 打印 (推荐，快速，免费)

### 步骤 1: 下载标签图片

```bash
cd /home/psibot/Documents/psibot_control

# 下载 tag25h9 的第0号标签
wget https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag25h9/tag25_09_00000.png

# 如果 wget 不可用，使用 curl
curl -L -o tag25_09_00000.png https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag25h9/tag25_09_00000.png
```

**如果网络下载失败**，我也可以为你生成一个简单的标签。

### 步骤 2: 打印

#### 使用 GIMP / Photoshop 打印 (精确控制尺寸)

1. 打开 `tag25_09_00000.png`
2. 图像 → 缩放图像
   - 宽度: 75 mm
   - 高度: 75 mm
   - 分辨率: 300 DPI
3. 文件 → 打印
   - 纸张: A4
   - 缩放: 100% (不要缩放)
   - 位置: 居中

#### 使用 LibreOffice Draw 打印

1. 新建文档
2. 插入图片 `tag25_09_00000.png`
3. 右键 → 位置和大小
   - 宽度: 7.5 cm
   - 高度: 7.5 cm
   - 保持比例
4. 文件 → 打印

#### 使用命令行打印 (Linux)

```bash
# 安装 ImageMagick
sudo apt install imagemagick

# 调整图片大小到 75mm (约 283 像素在 96 DPI)
convert tag25_09_00000.png -resize 283x283 tag_75mm.png

# 创建 A4 白色背景，居中放置标签
convert -size 2480x3508 xc:white \
    tag_75mm.png -gravity center -composite \
    tag_print.png

# 打印 (需要连接打印机)
lpr tag_print.png
```

### 步骤 3: 验证尺寸

打印后，用尺子测量：

```
┌─────────────────┐
│                 │
│   ┌─────────┐   │  ← 白色边缘 (不需要精确)
│   │█████████│   │
│   │█       █│   │  ← 测量这个黑色外边缘
│   │█ APRIL █│   │     应该是 75mm × 75mm
│   │█  TAG  █│   │
│   │█       █│   │
│   │█████████│   │  ← 75 mm
│   └─────────┘   │
│      75 mm      │
└─────────────────┘
```

**⚠️ 重要**：
- 测量的是**黑色方块的外边缘**，不包括白边
- 误差应小于 ±1mm
- 如果误差太大，需要重新调整打印比例

### 步骤 4: 后处理

1. **裁剪**：可以裁掉多余的白边，但至少保留标签周围 2-3cm 白边
2. **固定**：
   - 贴在硬纸板或亚克力板上（推荐）
   - 或使用相框裱起来
   - 确保**平整、无褶皱、无反光**

3. **摆放**：
   - 用双面胶或胶带固定在桌面上
   - 确保标定过程中不会移动

---

## 方法 2: 购买 (最方便，最耐用)

### 淘宝购买

搜索关键词：
- "AprilTag 标定板"
- "tag25h9 75mm"
- "机器人标定板"

推荐商品特征：
- ✅ 材质：铝板或亚克力
- ✅ 尺寸：75mm (外框可以大一些，但标签本身要 75mm)
- ✅ 系列：tag25h9 或 tag36h11
- ✅ 价格：20-50 元

**优点**：
- 平整度好
- 耐用
- 尺寸精确

**缺点**：
- 需要等快递
- 费用

---

## 方法 3: 生成自定义 AprilTag (备用)

如果下载不了，我可以帮你生成一个简单的 SVG 格式标签：

```python
# generate_apriltag.py
import numpy as np
from PIL import Image, ImageDraw

# AprilTag tag25h9 的 ID=0 的编码 (简化版)
# 实际的 AprilTag 有更复杂的编码，这里用简化示意
def create_simple_marker(size_mm=75, dpi=300):
    """
    创建一个简化的方块标记 (不是真正的 AprilTag，但可以用于演示)
    如果要真正的标定，请使用方法1或2
    """
    # 计算像素尺寸
    size_inch = size_mm / 25.4
    size_px = int(size_inch * dpi)
    
    # 创建白色背景
    img = Image.new('RGB', (size_px, size_px), 'white')
    draw = ImageDraw.Draw(img)
    
    # 黑色外框
    border = int(size_px * 0.1)
    draw.rectangle([border, border, size_px-border, size_px-border], 
                   fill='black')
    
    # 白色内框 (用于模拟 AprilTag 结构)
    inner = int(size_px * 0.15)
    draw.rectangle([inner, inner, size_px-inner, size_px-inner], 
                   fill='white')
    
    # 保存
    img.save('simple_marker_75mm.png', dpi=(dpi, dpi))
    print(f"已生成简化标记: simple_marker_75mm.png")
    print("⚠️ 这不是真正的 AprilTag，仅用于测试！")
    print("   正式标定请使用真正的 AprilTag")

if __name__ == "__main__":
    create_simple_marker()
```

**⚠️ 注意**：上面生成的不是真正的 AprilTag，不能用于正式标定！

---

## ✅ 检查清单

标定前确认：

- [ ] AprilTag 尺寸正确 (75mm × 75mm)
- [ ] 标签系列正确 (tag25h9)
- [ ] 打印质量好 (黑白分明，无模糊)
- [ ] 固定在平整表面上
- [ ] 在机器人工作空间内
- [ ] 相机能看到标签
- [ ] 光照充足均匀

---

## 🔍 如何确认是否是 tag25h9

真正的 tag25h9 AprilTag 看起来像这样：

```
┌───────────────────┐
│ ░░░░░░░░░░░░░░░░░ │  ← 白边 (不重要)
│ ░░███████████░░░░ │
│ ░░█░░░░░░░░░█░░░░ │  ← 黑色外框
│ ░░█░███████░█░░░░ │
│ ░░█░█░░░░░█░█░░░░ │  ← 内部编码区 (黑白方块)
│ ░░█░█░███░█░█░░░░ │     这些方块的排列
│ ░░█░█░█░█░█░█░░░░ │     代表了标签的ID
│ ░░█░█░███░█░█░░░░ │
│ ░░█░█░░░░░█░█░░░░ │
│ ░░█░███████░█░░░░ │
│ ░░█░░░░░░░░░█░░░░ │
│ ░░███████████░░░░ │
│ ░░░░░░░░░░░░░░░░░ │
└───────────────────┘
```

**特征**：
- 有黑色外框
- 内部是 5×5 或 6×6 的黑白方块编码
- 整体对称但有方向性

---

## 📞 获取帮助

如果仍然无法获取 AprilTag：
1. 可以先用简单的棋盘格标定板代替（但精度会低一些）
2. 联系当地的创客空间，可能有现成的
3. 找附近的大学实验室借用

---

**准备好 AprilTag 后，继续阅读: 手眼标定详细教程.md**

