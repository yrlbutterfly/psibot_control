import cv2
import numpy as np
import matplotlib.pyplot as plt
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# === 图像分割测试脚本 ===
# 使用 Segment Anything Model 2 (SAM 2.1) 进行交互式图像分割
# 流程：加载图像 -> 加载模型 -> 鼠标点击目标 -> 模型生成分割掩码 (Mask)

# 1. 加载图像
image_path = "image.png"
image_bgr = cv2.imread(image_path)
image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

# 2. 加载 SAM 2.1 模型
# 需要下载对应的 checkpoint 文件 (sam2.1_hiera_large.pt)
sam_checkpoint = "sam2.1_hiera_large.pt"
model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"  # Official config path

# Determine device
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Build SAM 2 model - config path is relative to sam2 package
sam2_model = build_sam2(model_cfg, sam_checkpoint, device=device)
predictor = SAM2ImagePredictor(sam2_model)
predictor.set_image(image)  # Preprocess image encoding

# 3. 交互式选择点
# 弹出一个窗口，用户点击想要分割的物体
clicked_point = []

def on_mouse(event, x, y, flags, param):
    """鼠标点击回调函数"""
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point.append((x, y))
        # 在点击位置画一个红点
        cv2.circle(image_bgr, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Click on the object", image_bgr)

cv2.imshow("Click on the object", image_bgr)
cv2.setMouseCallback("Click on the object", on_mouse)
cv2.waitKey(0)
cv2.destroyAllWindows()

if len(clicked_point) == 0:
    print("你没有点击任何点")
    exit()

# 准备提示点 (Prompt)
input_point = np.array([clicked_point[0]])  # 只用第一个点击点作为提示
input_label = np.array([1])  # 1表示前景点 (即点击的是物体)，0表示背景点

# 4. 执行分割
# predict 方法根据提示点生成分割掩码
masks, scores, logits = predictor.predict(
    point_coords=input_point,
    point_labels=input_label,
    multimask_output=True,  # True: 返回多个候选掩码（不同模糊度/大小），False: 只返回一个最优掩码
)

# 5. 可视化结果
# 使用 Matplotlib 显示原图和生成的 Mask
plt.figure(figsize=(10, 10))
plt.imshow(image)
for i, mask in enumerate(masks):
    plt.imshow(mask, alpha=0.4) # alpha设置透明度，叠加显示
    plt.title(f"Mask {i} (score={scores[i]:.3f})")
    plt.scatter(*input_point[0], color='red') # 标记出点击的点
    plt.axis('off')
    plt.show()

