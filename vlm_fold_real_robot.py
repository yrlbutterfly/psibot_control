#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM驱动的真实机器人衣物折叠流程
VLM-Driven Real Robot Garment Folding Pipeline

功能特性:
1. SAM自动分割衣物点云（无需人工点击）
2. VLM自动输出bbox和折叠计划
3. VLM输出判断执行操作（单臂/双臂）
4. 安全特性：每阶段人工确认 + 分步执行
5. 完整记录：VLM输入/输出/bbox可视化
6. 拍照前自动移动到Photo位置
"""

import cv2
import numpy as np
import open3d as o3d
import os
import sys
import time
import json
import base64
import re
from typing import Dict, List, Tuple, Optional
from openai import OpenAI

# Import custom modules
from real_robot_controller import RealRobotController
from motion_primitives import mp_right_fold, mp_left_fold, mp_bimanual_fold

# ==================== VLM Prompt Definition ====================
VLM_FOLD_PROMPT = (
    "The image contains a piece of clothing. Infer the NEXT folding action.\n\n"
    "Output a single JSON array with two objects:\n"
    "[\n"
    "  { \"plan\": <plan_output> },\n"
    "  { \"points\": <points_output> }\n"
    "]\n\n"
    "PLAN OUTPUT:\n"
    "- Either a list containing EXACTLY ONE action (the next step), OR the string \"already finish folding\".\n"
    "- If already folded, output:\n"
    "[\n"
    "  { \"plan\": \"already finish folding\" },\n"
    "  { \"points\": [] }\n"
    "]\n\n"
    "ACTION FORMAT (single step):\n"
    "{\n"
    "  \"left\":  { \"from\": <keypoint>, \"to\": <keypoint> },\n"
    "  \"right\": { \"from\": <keypoint>, \"to\": <keypoint> }\n"
    "}\n"
    "- If an arm is idle: {\"from\": null, \"to\": null}\n\n"
    "VALID KEYPOINT NAMES (ONLY THESE 6):\n"
    "left_cuff, right_cuff, left_hem, right_hem, left_shoulder, right_shoulder\n\n"
    "POINTS OUTPUT:\n"
    "- \"points\" MUST include ONLY the keypoints referenced by the action in \"plan\".\n"
    "- Each entry:\n"
    "{ \"label\": \"<keypoint>\", \"bbox\": [x_min, y_min, x_max, y_max] }\n\n"
    "STANDARD SHIRT FOLDING REFERENCE (FULL PLAN FROM A COMPLETELY FLAT, UNFOLDED SHIRT):\n"
    "Step 1: left_cuff  -> right_shoulder (right arm idle)\n"
    "Step 2: right_cuff -> left_shoulder  (left arm idle)\n"
    "Step 3: left_hem -> left_shoulder AND right_hem -> right_shoulder\n"
    "IMPORTANT: Use this reference ONLY to decide what the NEXT step should be. Do NOT output all steps.\n\n"
    "Do not output anything except the final JSON array.\n"
)


# ==================== VLM Helper Functions ====================

def _get_vlm_client(base_url: str, model_name: str):
    """构造 OpenAI 兼容的 VLM client"""
    api_key = os.environ.get("VLM_API_KEY", "EMPTY")
    client = OpenAI(base_url=base_url, api_key=api_key)
    return client, model_name


def _encode_rgb_to_data_url(rgb: np.ndarray) -> str:
    """将 RGB 图像编码为 base64 data URL"""
    rgb_uint8 = rgb.astype("uint8")
    bgr = cv2.cvtColor(rgb_uint8, cv2.COLOR_RGB2BGR)
    success, buf = cv2.imencode(".png", bgr)
    if not success:
        raise RuntimeError("无法将 RGB 图像编码为 PNG")
    img_bytes = buf.tobytes()
    img_base64 = base64.b64encode(img_bytes).decode("utf-8")
    return f"data:image/png;base64,{img_base64}"


def _parse_vlm_output(raw_text: str) -> Dict[str, object]:
    """解析 VLM 的原始文本输出"""
    try:
        data = json.loads(raw_text)
    except Exception:
        match = re.search(r"\[.*\]", raw_text, re.S)
        if not match:
            raise ValueError(f"无法在 VLM 输出中找到 JSON 数组，原始输出:\n{raw_text}")
        data = json.loads(match.group(0))

    if not isinstance(data, list) or len(data) != 2:
        raise ValueError(f"VLM 输出 JSON 结构不符合预期: {data}")

    plan_obj = data[0] if isinstance(data[0], dict) else {}
    points_obj = data[1] if isinstance(data[1], dict) else {}
    plan = plan_obj.get("plan", None)
    points = points_obj.get("points", None)
    return {"plan": plan, "points": points}


def _convert_vlm_points_to_absolute(
    points: Optional[List[Dict[str, object]]],
    img_w: int,
    img_h: int
) -> List[Dict[str, object]]:
    """将 VLM 输出的 points (0~1000 相对坐标) 转换为绝对像素坐标"""
    if not points:
        return []
    
    abs_points = []
    for item in points:
        if not isinstance(item, dict):
            continue
        label = item.get("label")
        bbox = item.get("bbox")
        if bbox is None or not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
            
        x_min_rel, y_min_rel, x_max_rel, y_max_rel = [float(v) for v in bbox]
        
        x_min = int(x_min_rel / 1000.0 * img_w)
        x_max = int(x_max_rel / 1000.0 * img_w)
        y_min = int(y_min_rel / 1000.0 * img_h)
        y_max = int(y_max_rel / 1000.0 * img_h)
        
        abs_points.append({
            "label": label,
            "bbox": [x_min, y_min, x_max, y_max]
        })
    return abs_points


def _ask_vlm_plan_and_points(rgb: np.ndarray, client, model_name: str) -> Dict[str, object]:
    """调用 VLM，返回解析后的 plan 与 points"""
    image_data_url = _encode_rgb_to_data_url(rgb)
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": image_data_url},
                    },
                    {"type": "text", "text": "\n" + VLM_FOLD_PROMPT},
                ],
            }
        ],
        max_tokens=1024,
    )
    raw = response.choices[0].message.content
    result = _parse_vlm_output(raw)

    # 转换坐标为绝对像素
    points = result.get("points")
    if points:
        h, w = rgb.shape[:2]
        print(f"[DEBUG] Converting VLM points. Image size: {w}x{h}")
        print(f"[DEBUG] Points before conversion: {points}")
        result["points"] = _convert_vlm_points_to_absolute(points, w, h)
        print(f"[DEBUG] Points after conversion: {result['points']}")
        
    return result


def _debug_save_vlm_output(
    debug_dir: str,
    step_idx: int,
    vlm_result: Dict[str, object],
) -> None:
    """保存 VLM 的原始解析结果到文本文件"""
    os.makedirs(debug_dir, exist_ok=True)
    log_path = os.path.join(debug_dir, "vlm_results.txt")
    with open(log_path, "a", encoding="utf-8") as f:
        json_str = json.dumps(vlm_result, ensure_ascii=False)
        f.write(f"step {step_idx}: {json_str}\n")


def _debug_save_vlm_rgb_with_bbox(
    rgb: np.ndarray,
    vlm_points: Optional[List[Dict[str, object]]],
    save_path: str,
) -> None:
    """在 RGB 图像上可视化 VLM 输出的 label + bbox"""
    if vlm_points is None:
        vlm_points = []

    img = rgb.astype("uint8").copy()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    for item in vlm_points:
        if not isinstance(item, dict):
            continue
        label = item.get("label")
        bbox = item.get("bbox")
        if (
            label is None
            or bbox is None
            or not isinstance(bbox, (list, tuple))
            or len(bbox) != 4
        ):
            continue
        x_min, y_min, x_max, y_max = [int(v) for v in bbox]
        # 画 bbox
        cv2.rectangle(
            img_bgr,
            (x_min, y_min),
            (x_max, y_max),
            color=(0, 255, 0),
            thickness=2,
        )
        # 在框的左上角写上 label
        cv2.putText(
            img_bgr,
            str(label),
            (x_min, max(y_min - 5, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, img_bgr)


def _debug_save_vlm_rgb_raw(
    rgb: np.ndarray,
    save_path: str,
) -> None:
    """保存原始 RGB 图像（不带 bbox）"""
    img = rgb.astype("uint8").copy()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, img_bgr)


# ==================== 3D Mapping Helper Functions ====================

def _get_3d_from_bbox_direct(
    bbox: List[float],
    depth_img: np.ndarray,
    camera_intrinsics: np.ndarray,
    T_cam2base: np.ndarray,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    直接从bbox和深度图计算3D坐标（不需要点云）
    
    Args:
        bbox: [x_min, y_min, x_max, y_max] 像素坐标
        depth_img: 深度图（单位：毫米）
        camera_intrinsics: 相机内参矩阵3x3 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        T_cam2base: 相机到基座的变换矩阵4x4
        
    Returns:
        (point_base, point_cam): 基座坐标系和相机坐标系的3D点，如果失败返回(None, None)
    """
    x_min, y_min, x_max, y_max = [int(v) for v in bbox]
    
    # 从bbox区域采样深度值
    bbox_depths = []
    bbox_pixels = []
    
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            if 0 <= y < depth_img.shape[0] and 0 <= x < depth_img.shape[1]:
                depth = depth_img[y, x]
                if depth > 0:  # 有效深度
                    bbox_depths.append(depth)
                    bbox_pixels.append([x, y])
    
    if len(bbox_depths) == 0:
        print(f"[WARNING] No valid depth in bbox {bbox}")
        return None, None
    
    # 使用中位数深度（更鲁棒，抗异常值）
    median_depth_mm = np.median(bbox_depths)
    median_depth_m = median_depth_mm / 1000.0
    
    # 使用bbox中心作为2D坐标
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    
    # 从内参矩阵提取参数
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]
    
    # 反投影：从像素坐标和深度计算相机坐标系3D点
    X_cam = (center_x - cx) * median_depth_m / fx
    Y_cam = (center_y - cy) * median_depth_m / fy
    Z_cam = median_depth_m
    
    point_cam = np.array([X_cam, Y_cam, Z_cam])
    
    # 转换到基座坐标系
    point_cam_homo = np.append(point_cam, 1.0)  # 转为齐次坐标
    point_base_homo = T_cam2base @ point_cam_homo
    point_base = point_base_homo[:3]
    
    print(f"  [Direct bbox->3D] bbox={bbox}, depth={median_depth_mm:.0f}mm, "
          f"cam={point_cam}, base={point_base}")
    
    return point_base, point_cam


def _label_to_bbox_map(
    vlm_points: Optional[List[Dict[str, object]]],
) -> Dict[str, List[float]]:
    """将 VLM 输出的 points_list 转换为 label -> bbox 映射"""
    label_to_bbox: Dict[str, List[float]] = {}
    if not isinstance(vlm_points, list):
        return label_to_bbox
    for item in vlm_points:
        if not isinstance(item, dict):
            continue
        label = item.get("label")
        bbox = item.get("bbox")
        if (
            label is None
            or bbox is None
            or not isinstance(bbox, (list, tuple))
            or len(bbox) != 4
        ):
            continue
        x_min, y_min, x_max, y_max = [float(b) for b in bbox]
        label_to_bbox[str(label)] = [x_min, y_min, x_max, y_max]
    return label_to_bbox


def _get_3d_point_from_label_direct(
    label_name: Optional[str],
    label_to_bbox: Dict[str, List[float]],
    depth_img: np.ndarray,
    camera_intrinsics: np.ndarray,
    T_cam2base: np.ndarray,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    根据 label 直接从深度图获取对应的 3D 点（简化版，不需要点云）
    
    Args:
        label_name: 标签名称 (如 'left_cuff')
        label_to_bbox: label -> bbox 映射
        depth_img: 深度图（毫米）
        camera_intrinsics: 相机内参矩阵3x3
        T_cam2base: 相机到基座变换矩阵4x4
        
    Returns:
        (point_base, point_cam): 基座坐标和相机坐标，失败返回(None, None)
    """
    if label_name is None:
        return None, None
    
    bbox = label_to_bbox.get(str(label_name))
    if bbox is None:
        print(f"[WARNING] Label '{label_name}' not found in bbox map")
        return None, None
    
    return _get_3d_from_bbox_direct(bbox, depth_img, camera_intrinsics, T_cam2base)


# ==================== Motion Execution Logic ====================

def execute_vlm_motion_plan(
    robot_controller,
    plan_step: Dict[str, object],
    label_to_bbox: Dict[str, List[float]],
    depth_img: np.ndarray,
    camera_intrinsics: np.ndarray,
    T_cam2base_left: np.ndarray,
    T_cam2base_right: np.ndarray,
    lift_height: float = 0.1,
) -> bool:
    """
    根据 VLM 的 plan_step 解析意图，并调用对应的 Motion Planning 函数（简化版，直接从深度图计算）
    
    Args:
        robot_controller: RealRobotController instance
        plan_step: VLM plan 中的一个 step
        label_to_bbox: label -> bbox 映射
        depth_img: 深度图（毫米）
        camera_intrinsics: 相机内参矩阵3x3
        T_cam2base_left: 左臂的相机到基座变换矩阵
        T_cam2base_right: 右臂的相机到基座变换矩阵
        lift_height: 抬起高度
        
    Returns:
        bool: 是否执行成功
    """
    left_cfg = plan_step.get("left", {}) or {}
    right_cfg = plan_step.get("right", {}) or {}
    
    left_from_label = left_cfg.get("from")
    left_to_label = left_cfg.get("to")
    right_from_label = right_cfg.get("from")
    right_to_label = right_cfg.get("to")
    
    print(f"\n[Motion Plan] Parsing VLM output:")
    print(f"  Left:  {left_from_label} -> {left_to_label}")
    print(f"  Right: {right_from_label} -> {right_to_label}")
    
    # 检查是否是双臂动作
    if left_from_label and left_to_label and right_from_label and right_to_label:
        print(f"[Motion Plan] Detected: BIMANUAL action")
        
        # 获取左手的 3D 点（使用左臂标定）
        left_grasp_base, left_grasp_cam = _get_3d_point_from_label_direct(
            left_from_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_left
        )
        left_place_base, left_place_cam = _get_3d_point_from_label_direct(
            left_to_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_left
        )
        
        # 获取右手的 3D 点（使用右臂标定）
        right_grasp_base, right_grasp_cam = _get_3d_point_from_label_direct(
            right_from_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_right
        )
        right_place_base, right_place_cam = _get_3d_point_from_label_direct(
            right_to_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_right
        )
        
        if any(x is None for x in [left_grasp_base, left_place_base, right_grasp_base, right_place_base]):
            print("[ERROR] Missing keypoints for bimanual fold. Skipping.")
            return False
        
        print(f"  Left grasp:  {left_grasp_base}")
        print(f"  Left place:  {left_place_base}")
        print(f"  Right grasp: {right_grasp_base}")
        print(f"  Right place: {right_place_base}")
        
        # 调用双臂折叠函数
        return mp_bimanual_fold(
            robot_controller,
            left_grasp_base, right_grasp_base,
            left_place_base, right_place_base,
            lift_height
        )
    
    # 检查是否是左手单独动作
    elif left_from_label and left_to_label:
        print(f"[Motion Plan] Detected: LEFT ARM action")
        
        left_grasp_base, _ = _get_3d_point_from_label_direct(
            left_from_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_left
        )
        left_place_base, _ = _get_3d_point_from_label_direct(
            left_to_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_left
        )
        
        if left_grasp_base is None or left_place_base is None:
            print("[ERROR] Missing keypoints for left arm fold. Skipping.")
            return False
        
        print(f"  Grasp: {left_grasp_base}")
        print(f"  Place: {left_place_base}")
        
        # 调用左手折叠函数
        return mp_left_fold(
            robot_controller,
            left_grasp_base, left_place_base,
            lift_height, None
        )
    
    # 检查是否是右手单独动作
    elif right_from_label and right_to_label:
        print(f"[Motion Plan] Detected: RIGHT ARM action")
        
        right_grasp_base, _ = _get_3d_point_from_label_direct(
            right_from_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_right
        )
        right_place_base, _ = _get_3d_point_from_label_direct(
            right_to_label, label_to_bbox, depth_img, camera_intrinsics, T_cam2base_right
        )
        
        if right_grasp_base is None or right_place_base is None:
            print("[ERROR] Missing keypoints for right arm fold. Skipping.")
            return False
        
        print(f"  Grasp: {right_grasp_base}")
        print(f"  Place: {right_place_base}")
        
        # 调用右手折叠函数
        return mp_right_fold(
            robot_controller,
            right_grasp_base, right_place_base,
            lift_height, None
        )
    
    else:
        print(f"[WARNING] Unrecognized plan step: {plan_step}")
        return False


# ==================== Robot Position Management ====================

def move_to_preset(config_file, pose_name):
    """移动机器人到预设位置"""
    print(f"\n[Robot Control] Moving robot to {pose_name} position...")
    if not os.path.exists(config_file):
        print(f"  [SKIP] Config file {config_file} not found.")
        return
    
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        # Initialize temp controller
        tmp_robot = RealRobotController()
        
        if 'left' in config:
            print(f"  Moving Left Arm to {pose_name}...")
            tmp_robot.left_arm.move_joint(config['left'], speed=6, block=True)
        
        if 'right' in config:
            print(f"  Moving Right Arm to {pose_name}...")
            tmp_robot.right_arm.move_joint(config['right'], speed=6, block=True)
            
        tmp_robot.close()
        print(f"  ✓ Robot at {pose_name}")
        
    except Exception as e:
        print(f"  [WARNING] Failed to move to {pose_name}: {e}")


def robust_input_main(prompt=""):
    """Robust input function that works even after OpenCV operations"""
    print(prompt, end='', flush=True)
    try:
        with open('/dev/tty', 'r') as tty:
            return tty.readline().rstrip('\n')
    except:
        try:
            return input()
        except EOFError:
            return ""


# ==================== Main Pipeline ====================

def main():
    """主流程：VLM驱动的衣物折叠"""
    
    # ==================== Configuration ====================
    CAMERA_SN = "046322250624"
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251226-234053.npz"
    CALIB_FILE_RIGHT = "calibration_results/camera_calibration_right_arm_20251226-234855.npz"
    
    PHOTO_CONFIG_FILE = "robot_photo_config.json"
    HOME_CONFIG_FILE = "robot_home_config.json"
    
    # VLM Configuration
    VLM_BASE_URL = os.environ.get("VLM_BASE_URL", "http://localhost:8001/v1")
    VLM_MODEL_NAME = os.environ.get("VLM_MODEL_NAME", "/home/psibot/Documents/psibot_control/checkpoint-900")
    
    # Debug Configuration
    DEBUG_FLAG = True
    DEBUG_DIR = "debug_vlm_real_robot"
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    debug_session_dir = os.path.join(DEBUG_DIR, timestamp)
    debug_rgb_dir = os.path.join(debug_session_dir, "vlm_rgb")
    
    LIFT_HEIGHT = 0.08  # 抬起高度 (米) - 实际未使用
    MAX_SUBTASKS = 6    # 最大任务数
    
    print("\n" + "="*70)
    print("  VLM驱动的真实机器人衣物折叠流程")
    print("  VLM-Driven Real Robot Garment Folding Pipeline")
    print("="*70)
    
    # Load calibration files
    if not os.path.exists(CALIB_FILE_LEFT) or not os.path.exists(CALIB_FILE_RIGHT):
        print(f"[ERROR] Calibration files not found!")
        print(f"  Left:  {CALIB_FILE_LEFT}")
        print(f"  Right: {CALIB_FILE_RIGHT}")
        return
    
    calib_data_left = np.load(CALIB_FILE_LEFT)
    calib_data_right = np.load(CALIB_FILE_RIGHT)
    T_cam2base_left = calib_data_left['T_cam2base']
    T_cam2base_right = calib_data_right['T_cam2base']
    print(f"[INFO] Loaded calibration files")
    
    # Initialize VLM client
    vlm_client, vlm_model = _get_vlm_client(VLM_BASE_URL, VLM_MODEL_NAME)
    print(f"[INFO] VLM initialized: {VLM_BASE_URL} / {vlm_model}")
    
    # ==================== Task Loop ====================
    try:
        for subtask_idx in range(MAX_SUBTASKS):
            print("\n" + "="*70)
            print(f"  Subtask {subtask_idx + 1}/{MAX_SUBTASKS}: VLM Planning & Execution")
            print("="*70)
            
            # ---------------------------------------------------------
            # Step 1: 移动到拍照位置
            # ---------------------------------------------------------
            print(f"\n[Step 1/{6}] Moving to Photo position...")
            move_to_preset(PHOTO_CONFIG_FILE, "Photo")
            time.sleep(1.0)
            
            # ---------------------------------------------------------
            # Step 2: 拍照（简化版：不需要SAM分割）
            # ---------------------------------------------------------
            print(f"\n[Step 2/{6}] Capturing image...")
            
            # 初始化相机并拍照
            from robot_libs.realsense_image_module import Image as RealsenseCamera
            camera = RealsenseCamera(serial_number=CAMERA_SN)
            color_img, depth_img = camera.capture_frame()
            img_height, img_width = color_img.shape[:2]
            print(f"  ✓ Captured image: {img_width}x{img_height}")
            
            # 获取相机内参
            camera_intrinsics = camera.o3d_intrinsics.intrinsic_matrix
            
            # 关闭相机
            camera.close()
            print("  ✓ Camera closed")
            
            # ---------------------------------------------------------
            # Step 3: 移动回Home位置
            # ---------------------------------------------------------
            print(f"\n[Step 3/{6}] Moving back to Home position...")
            move_to_preset(HOME_CONFIG_FILE, "Home")
            time.sleep(1.0)
            
            # ---------------------------------------------------------
            # Step 4: VLM推理获取折叠计划和bbox
            # ---------------------------------------------------------
            print(f"\n[Step 4/{6}] Querying VLM for folding plan...")
            
            vlm_result = _ask_vlm_plan_and_points(color_img, vlm_client, vlm_model)
            plan = vlm_result.get("plan", None)
            points = vlm_result.get("points", None)
            
            print(f"  ✓ VLM Plan: {plan}")
            print(f"  ✓ VLM Points: {points}")
            
            # Save VLM output for debugging
            if DEBUG_FLAG:
                _debug_save_vlm_output(debug_session_dir, subtask_idx, vlm_result)
                _debug_save_vlm_rgb_with_bbox(
                    color_img, points,
                    os.path.join(debug_rgb_dir, f"vlm_rgb_{subtask_idx:03d}.png")
                )
                _debug_save_vlm_rgb_raw(
                    color_img,
                    os.path.join(debug_rgb_dir, f"vlm_rgb_raw_{subtask_idx:03d}.png")
                )
            
            # ---------------------------------------------------------
            # Step 5: 解析VLM输出
            # ---------------------------------------------------------
            print(f"\n[Step 5/{6}] Parsing VLM output...")
            
            # Check if finished
            if isinstance(plan, str) and "finish" in plan.lower():
                print("  ✓ VLM indicates folding is complete!")
                break
            
            # Validate plan
            if not isinstance(plan, list) or len(plan) == 0:
                print(f"  [WARNING] Invalid plan format: {plan}")
                continue
            
            current_step = plan[0]
            print(f"  ✓ Current step: {current_step}")
            
            # Build label -> bbox mapping
            label_to_bbox = _label_to_bbox_map(points)
            print(f"  ✓ Label to bbox mapping: {label_to_bbox}")
            
            # ---------------------------------------------------------
            # Step 6: 执行Motion Planning（简化版：直接从深度图计算3D坐标）
            # ---------------------------------------------------------
            print(f"\n[Step 6/{6}] Executing motion plan...")
            
            # Initialize robot controller
            print("  [INFO] Initializing robot controller...")
            robot = RealRobotController(camera_sn=CAMERA_SN)
            
            # Execute motion plan (使用简化的直接方法)
            success = execute_vlm_motion_plan(
                robot_controller=robot,
                plan_step=current_step,
                label_to_bbox=label_to_bbox,
                depth_img=depth_img,
                camera_intrinsics=camera_intrinsics,
                T_cam2base_left=T_cam2base_left,
                T_cam2base_right=T_cam2base_right,
                lift_height=LIFT_HEIGHT,
            )
            
            # Cleanup
            robot.close()
            
            if success:
                print(f"  ✓ Subtask {subtask_idx + 1} completed successfully!")
                # Wait for garment to settle before next iteration
                print("  [INFO] Waiting for garment to settle (3 seconds)...")
                time.sleep(3.0)
            else:
                print(f"  ✗ Subtask {subtask_idx + 1} failed or was cancelled")
                user_continue = robust_input_main("\n  Continue to next subtask? (y/n) [n]: ").strip().lower()
                if user_continue != 'y':
                    print("  [INFO] User chose to stop")
                    break
        
        print("\n" + "="*70)
        print("  ✓ VLM折叠流程完成！")
        print("="*70)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Process interrupted by user")
    except Exception as e:
        print(f"\n✗ Error occurred: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

