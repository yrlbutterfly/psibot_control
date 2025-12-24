#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLM Interface for Garment Folding
调用多模态大模型(VLM)生成折叠计划和关键点检测
参考 Isaac Sim 代码中的 VLM 调用逻辑
"""

import os
import json
import base64
import re
import cv2
import numpy as np
from typing import Dict, List, Optional
from openai import OpenAI


# VLM Prompt (参考 Isaac Sim 代码)
VLM_FOLD_PROMPT = """The image contains a piece of clothing. Infer the NEXT folding action.

Output a single JSON array with two objects:
[
  { "plan": <plan_output> },
  { "points": <points_output> }
]

PLAN OUTPUT:
- Either a list containing EXACTLY ONE action (the next step), OR the string "already finish folding".
- If already folded, output:
[
  { "plan": "already finish folding" },
  { "points": [] }
]

ACTION FORMAT (single step):
{
  "left":  { "from": <keypoint>, "to": <keypoint> },
  "right": { "from": <keypoint>, "to": <keypoint> }
}
- If an arm is idle: {"from": null, "to": null}

VALID KEYPOINT NAMES (ONLY THESE 6):
left_cuff, right_cuff, left_hem, right_hem, left_shoulder, right_shoulder

POINTS OUTPUT:
- "points" MUST include ONLY the keypoints referenced by the action in "plan".
- Each entry:
{ "label": "<keypoint>", "bbox": [x_min, y_min, x_max, y_max] }

STANDARD SHIRT FOLDING REFERENCE (FULL PLAN FROM A COMPLETELY FLAT, UNFOLDED SHIRT):
Step 1: left_cuff  -> right_shoulder (right arm idle)
Step 2: right_cuff -> left_shoulder  (left arm idle)
Step 3: left_hem -> left_shoulder AND right_hem -> right_shoulder
IMPORTANT: Use this reference ONLY to decide what the NEXT step should be. Do NOT output all steps.

Do not output anything except the final JSON array.
"""


class VLMInterface:
    """Interface for Vision-Language Model to generate folding plans"""
    
    def __init__(self, base_url: str, model_name: str, api_key: Optional[str] = None):
        """
        Args:
            base_url: VLM API endpoint (e.g., "http://localhost:8000/v1" for local)
            model_name: Model name (e.g., "qwen-vl-plus" or "gpt-4-vision-preview")
            api_key: API key (default: read from VLM_API_KEY env var)
        """
        if api_key is None:
            api_key = os.environ.get("VLM_API_KEY", "EMPTY")
        
        self.client = OpenAI(base_url=base_url, api_key=api_key)
        self.model_name = model_name
        print(f"[INFO] VLM initialized: {model_name} @ {base_url}")
    
    def encode_image_to_data_url(self, rgb_image: np.ndarray) -> str:
        """
        Encode RGB numpy array to base64 data URL for API
        
        Args:
            rgb_image: (H, W, 3) uint8 RGB image
        
        Returns:
            data_url: base64-encoded image string
        """
        # Convert RGB to BGR for OpenCV
        bgr = cv2.cvtColor(rgb_image.astype(np.uint8), cv2.COLOR_RGB2BGR)
        
        # Encode to PNG
        success, buffer = cv2.imencode('.png', bgr)
        if not success:
            raise RuntimeError("Failed to encode image to PNG")
        
        # Convert to base64
        img_bytes = buffer.tobytes()
        img_base64 = base64.b64encode(img_bytes).decode('utf-8')
        
        return f"data:image/png;base64,{img_base64}"
    
    def parse_vlm_response(self, raw_text: str) -> Dict[str, object]:
        """
        Parse VLM raw text output to extract plan and points
        
        Args:
            raw_text: Raw response from VLM
        
        Returns:
            {"plan": ..., "points": ...}
        """
        try:
            # Try direct JSON parsing
            data = json.loads(raw_text)
        except json.JSONDecodeError:
            # Extract JSON array using regex
            match = re.search(r'\[.*\]', raw_text, re.DOTALL)
            if not match:
                raise ValueError(f"Cannot find JSON array in VLM output:\n{raw_text}")
            data = json.loads(match.group(0))
        
        # Validate format
        if not isinstance(data, list) or len(data) != 2:
            raise ValueError(f"VLM output must be a list of length 2, got: {data}")
        
        plan_obj = data[0] if isinstance(data[0], dict) else {}
        points_obj = data[1] if isinstance(data[1], dict) else {}
        
        plan = plan_obj.get("plan", None)
        points = points_obj.get("points", None)
        
        return {"plan": plan, "points": points}
    
    def convert_bbox_to_absolute(
        self,
        points: Optional[List[Dict]],
        img_width: int,
        img_height: int
    ) -> List[Dict]:
        """
        Convert VLM bounding boxes from relative [0, 1000] to absolute pixel coordinates
        
        Args:
            points: List of {"label": str, "bbox": [x_min, y_min, x_max, y_max]} in [0, 1000]
            img_width, img_height: Image dimensions in pixels
        
        Returns:
            points_abs: Same format but with absolute pixel coordinates
        """
        if not points:
            return []
        
        points_abs = []
        
        for item in points:
            if not isinstance(item, dict):
                continue
            
            label = item.get("label")
            bbox = item.get("bbox")
            
            if bbox is None or not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                continue
            
            # Convert from [0, 1000] to pixel coordinates
            x_min_rel, y_min_rel, x_max_rel, y_max_rel = [float(v) for v in bbox]
            
            x_min = int(x_min_rel / 1000.0 * img_width)
            x_max = int(x_max_rel / 1000.0 * img_width)
            y_min = int(y_min_rel / 1000.0 * img_height)
            y_max = int(y_max_rel / 1000.0 * img_height)
            
            points_abs.append({
                "label": label,
                "bbox": [x_min, y_min, x_max, y_max]
            })
        
        return points_abs
    
    def ask_vlm_plan(self, rgb_image: np.ndarray, max_tokens: int = 1024) -> Dict[str, object]:
        """
        Query VLM for folding plan and keypoints
        
        Args:
            rgb_image: (H, W, 3) uint8 RGB image
            max_tokens: Maximum tokens for response
        
        Returns:
            {"plan": ..., "points": [...]}
        """
        # Encode image
        image_url = self.encode_image_to_data_url(rgb_image)
        
        # Call VLM API
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": image_url}
                        },
                        {
                            "type": "text",
                            "text": VLM_FOLD_PROMPT
                        }
                    ]
                }
            ],
            max_tokens=max_tokens
        )
        
        # Parse response
        raw_text = response.choices[0].message.content
        print(f"[DEBUG] VLM raw response:\n{raw_text}")
        
        result = self.parse_vlm_response(raw_text)
        
        # Convert bbox coordinates to absolute pixels
        points = result.get("points")
        if points:
            h, w = rgb_image.shape[:2]
            result["points"] = self.convert_bbox_to_absolute(points, w, h)
        
        print(f"[INFO] VLM plan: {result['plan']}")
        print(f"[INFO] VLM points: {result['points']}")
        
        return result


def test_vlm():
    """Test VLM interface with a sample image"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python vlm_interface.py <image_path>")
        return
    
    # Load test image
    img_path = sys.argv[1]
    if not os.path.exists(img_path):
        print(f"Image not found: {img_path}")
        return
    
    img = cv2.imread(img_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Initialize VLM
    vlm = VLMInterface(
        base_url="http://localhost:8000/v1",  # TODO: Update to your VLM endpoint
        model_name="qwen-vl-plus"  # TODO: Update to your model name
    )
    
    # Query VLM
    result = vlm.ask_vlm_plan(img_rgb)
    
    # Print results
    print("\n" + "="*50)
    print("VLM Output:")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("="*50)


if __name__ == "__main__":
    test_vlm()

