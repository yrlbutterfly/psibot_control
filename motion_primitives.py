#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motion Planning Primitives for Garment Folding
参考 Isaac Sim 代码中的折叠动作原语
实现基本的抓取、移动、放置等动作
"""

import numpy as np
import time
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation as R


class MotionPrimitives:
    """Motion primitives for bimanual garment manipulation"""
    
    def __init__(self, left_arm, right_arm, left_hand, right_hand):
        """
        Args:
            left_arm: Left arm controller (ArmControl instance)
            right_arm: Right arm controller (ArmControl instance)
            left_hand: Left hand controller (Hand instance)
            right_hand: Right hand controller (Hand instance)
        """
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.left_hand = left_hand
        self.right_hand = right_hand
        
        # Home positions (defined in robot base frame)
        self.home_left = np.array([-0.6, 0.0, 0.5, 3.14, 0, 0])  # [x, y, z, rx, ry, rz]
        self.home_right = np.array([0.6, 0.0, 0.5, 3.14, 0, 0])
        
        # Grasp orientations (vertical downward grasp for folding)
        self.grasp_ori_left = np.array([3.14, 0, 0])  # [rx, ry, rz] in radians
        self.grasp_ori_right = np.array([3.14, 0, 0])
        
        print("[INFO] Motion primitives initialized")
    
    def move_to_home(self, settle_time: float = 1.0):
        """Move both arms to home position"""
        print("[MP] Moving to home position...")
        
        # Move both arms simultaneously
        self.left_arm.move_pose_Cmd(self.home_left.tolist(), speed=20)
        self.right_arm.move_pose_Cmd(self.home_right.tolist(), speed=20)
        
        time.sleep(settle_time)
        print("[MP] Reached home position")
    
    def open_both_hands(self):
        """Open both hands"""
        self.left_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.right_hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(0.5)
    
    def close_hand(self, hand: str):
        """
        Close specified hand
        Args:
            hand: "left" or "right"
        """
        close_angles = [0.4, 0.4, 0.4, 0.4, 0.4, 1.0]
        if hand == "left":
            self.left_hand.set_angles(close_angles)
        elif hand == "right":
            self.right_hand.set_angles(close_angles)
        time.sleep(0.5)
    
    def open_hand(self, hand: str):
        """
        Open specified hand
        Args:
            hand: "left" or "right"
        """
        open_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        if hand == "left":
            self.left_hand.set_angles(open_angles)
        elif hand == "right":
            self.right_hand.set_angles(open_angles)
        time.sleep(0.5)
    
    def grasp_and_move(
        self,
        arm: str,
        start_pos: np.ndarray,
        target_pos: np.ndarray,
        lift_height: float = 0.15,
        approach_offset: float = 0.05
    ):
        """
        Grasp at start_pos and move to target_pos (single arm operation)
        
        Args:
            arm: "left" or "right"
            start_pos: (3,) array [x, y, z] in base frame
            target_pos: (3,) array [x, y, z] in base frame
            lift_height: Height to lift object above table
            approach_offset: Offset above grasp point for pre-grasp
        """
        print(f"[MP] {arm.upper()} arm: Grasp and move")
        
        # Select arm controller
        if arm == "left":
            arm_ctrl = self.left_arm
            grasp_ori = self.grasp_ori_left
        elif arm == "right":
            arm_ctrl = self.right_arm
            grasp_ori = self.grasp_ori_right
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        # Phase 1: Pre-grasp (approach from above)
        pre_grasp_pos = start_pos.copy()
        pre_grasp_pos[2] += approach_offset
        pre_grasp_pose = np.concatenate([pre_grasp_pos, grasp_ori])
        
        print(f"  → Phase 1: Move to pre-grasp position {pre_grasp_pos}")
        arm_ctrl.move_pose_Cmd(pre_grasp_pose.tolist(), speed=15)
        time.sleep(0.5)
        
        # Phase 2: Move down to grasp position
        grasp_pose = np.concatenate([start_pos, grasp_ori])
        print(f"  → Phase 2: Move to grasp position {start_pos}")
        arm_ctrl.move_pose_Cmd(grasp_pose.tolist(), speed=10)
        time.sleep(0.5)
        
        # Phase 3: Close gripper
        print(f"  → Phase 3: Close gripper")
        self.close_hand(arm)
        time.sleep(0.5)
        
        # Phase 4: Lift
        lift_pos = start_pos.copy()
        lift_pos[2] = lift_height
        lift_pose = np.concatenate([lift_pos, grasp_ori])
        print(f"  → Phase 4: Lift to {lift_pos}")
        arm_ctrl.move_pose_Cmd(lift_pose.tolist(), speed=10)
        time.sleep(0.5)
        
        # Phase 5: Move to target (at lift height)
        target_lift_pos = target_pos.copy()
        target_lift_pos[2] = lift_height
        target_lift_pose = np.concatenate([target_lift_pos, grasp_ori])
        print(f"  → Phase 5: Move to target {target_lift_pos}")
        arm_ctrl.move_pose_Cmd(target_lift_pose.tolist(), speed=10)
        time.sleep(0.5)
        
        # Phase 6: Move down to target height
        # For folding, we want to place the fabric on top of existing layers
        # So we use a small offset above the target position
        place_pos = target_pos.copy()
        place_pos[2] += 0.02  # 2cm above target
        place_pose = np.concatenate([place_pos, grasp_ori])
        print(f"  → Phase 6: Lower to place position {place_pos}")
        arm_ctrl.move_pose_Cmd(place_pose.tolist(), speed=8)
        time.sleep(0.5)
        
        # Phase 7: Release
        print(f"  → Phase 7: Release gripper")
        self.open_hand(arm)
        time.sleep(0.5)
        
        # Phase 8: Retract
        retract_pos = place_pos.copy()
        retract_pos[2] += approach_offset
        retract_pose = np.concatenate([retract_pos, grasp_ori])
        print(f"  → Phase 8: Retract to {retract_pos}")
        arm_ctrl.move_pose_Cmd(retract_pose.tolist(), speed=10)
        time.sleep(0.5)
        
        print(f"[MP] {arm.upper()} arm: Grasp and move complete")
    
    def bimanual_fold_bottom(
        self,
        left_start: np.ndarray,
        right_start: np.ndarray,
        left_target: np.ndarray,
        right_target: np.ndarray,
        lift_height: float = 0.15
    ):
        """
        Dual-arm bottom folding (参考 Isaac Sim 的 bottom-up fold)
        
        Args:
            left_start: Left hem position (3,) [x, y, z]
            right_start: Right hem position (3,) [x, y, z]
            left_target: Left shoulder position (3,) [x, y, z]
            right_target: Right shoulder position (3,) [x, y, z]
            lift_height: Height to lift garment
        """
        print("[MP] Bimanual bottom fold")
        
        # Phase 1: Both arms move to pre-grasp
        print("  → Phase 1: Move to pre-grasp positions")
        left_pregrasp = left_start.copy()
        left_pregrasp[2] += 0.05
        right_pregrasp = right_start.copy()
        right_pregrasp[2] += 0.05
        
        left_pregrasp_pose = np.concatenate([left_pregrasp, self.grasp_ori_left])
        right_pregrasp_pose = np.concatenate([right_pregrasp, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_pregrasp_pose.tolist(), speed=15)
        self.right_arm.move_pose_Cmd(right_pregrasp_pose.tolist(), speed=15)
        time.sleep(1.0)
        
        # Phase 2: Move to grasp positions
        print("  → Phase 2: Move to grasp positions")
        left_grasp_pose = np.concatenate([left_start, self.grasp_ori_left])
        right_grasp_pose = np.concatenate([right_start, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_grasp_pose.tolist(), speed=10)
        self.right_arm.move_pose_Cmd(right_grasp_pose.tolist(), speed=10)
        time.sleep(1.0)
        
        # Phase 3: Close both grippers
        print("  → Phase 3: Close both grippers")
        self.close_hand("left")
        self.close_hand("right")
        time.sleep(0.5)
        
        # Phase 4: Lift together
        print("  → Phase 4: Lift garment")
        left_lift = left_start.copy()
        left_lift[2] = lift_height
        right_lift = right_start.copy()
        right_lift[2] = lift_height
        
        left_lift_pose = np.concatenate([left_lift, self.grasp_ori_left])
        right_lift_pose = np.concatenate([right_lift, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_lift_pose.tolist(), speed=10)
        self.right_arm.move_pose_Cmd(right_lift_pose.tolist(), speed=10)
        time.sleep(1.0)
        
        # Phase 5: Move to target positions (fold)
        print("  → Phase 5: Fold to targets")
        left_target_lift = left_target.copy()
        left_target_lift[2] = lift_height
        right_target_lift = right_target.copy()
        right_target_lift[2] = lift_height
        
        left_target_pose = np.concatenate([left_target_lift, self.grasp_ori_left])
        right_target_pose = np.concatenate([right_target_lift, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_target_pose.tolist(), speed=8)
        self.right_arm.move_pose_Cmd(right_target_pose.tolist(), speed=8)
        time.sleep(1.5)
        
        # Phase 6: Lower and place
        print("  → Phase 6: Lower and place")
        left_place = left_target.copy()
        left_place[2] += 0.02
        right_place = right_target.copy()
        right_place[2] += 0.02
        
        left_place_pose = np.concatenate([left_place, self.grasp_ori_left])
        right_place_pose = np.concatenate([right_place, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_place_pose.tolist(), speed=8)
        self.right_arm.move_pose_Cmd(right_place_pose.tolist(), speed=8)
        time.sleep(1.0)
        
        # Phase 7: Release both
        print("  → Phase 7: Release both grippers")
        self.open_hand("left")
        self.open_hand("right")
        time.sleep(0.5)
        
        # Phase 8: Retract both
        print("  → Phase 8: Retract")
        left_retract = left_place.copy()
        left_retract[2] += 0.05
        right_retract = right_place.copy()
        right_retract[2] += 0.05
        
        left_retract_pose = np.concatenate([left_retract, self.grasp_ori_left])
        right_retract_pose = np.concatenate([right_retract, self.grasp_ori_right])
        
        self.left_arm.move_pose_Cmd(left_retract_pose.tolist(), speed=10)
        self.right_arm.move_pose_Cmd(right_retract_pose.tolist(), speed=10)
        time.sleep(1.0)
        
        print("[MP] Bimanual bottom fold complete")
    
    def execute_vlm_action(
        self,
        action: dict,
        label_to_3d_base: dict
    ):
        """
        Execute a VLM action (one step from the plan)
        
        Args:
            action: Dict with format:
                {
                    "left": {"from": "left_cuff", "to": "right_shoulder"},
                    "right": {"from": null, "to": null}
                }
            label_to_3d_base: Dict mapping keypoint labels to 3D positions in base frame
        
        Returns:
            success: bool
        """
        print(f"[MP] Executing VLM action: {action}")
        
        left_cfg = action.get("left", {}) or {}
        right_cfg = action.get("right", {}) or {}
        
        left_from = left_cfg.get("from")
        left_to = left_cfg.get("to")
        right_from = right_cfg.get("from")
        right_to = right_cfg.get("to")
        
        # Check if both arms are active (bimanual fold)
        if left_from and left_to and right_from and right_to:
            # Bimanual operation (e.g., bottom fold)
            left_start = label_to_3d_base.get(left_from)
            left_end = label_to_3d_base.get(left_to)
            right_start = label_to_3d_base.get(right_from)
            right_end = label_to_3d_base.get(right_to)
            
            if None in [left_start, left_end, right_start, right_end]:
                print(f"[ERROR] Missing keypoints for bimanual action")
                return False
            
            self.bimanual_fold_bottom(left_start, right_start, left_end, right_end)
            return True
        
        # Single arm operations
        elif left_from and left_to:
            # Left arm fold (e.g., left sleeve)
            start = label_to_3d_base.get(left_from)
            end = label_to_3d_base.get(left_to)
            
            if start is None or end is None:
                print(f"[ERROR] Missing keypoints for left arm action")
                return False
            
            self.grasp_and_move("left", start, end)
            return True
        
        elif right_from and right_to:
            # Right arm fold (e.g., right sleeve)
            start = label_to_3d_base.get(right_from)
            end = label_to_3d_base.get(right_to)
            
            if start is None or end is None:
                print(f"[ERROR] Missing keypoints for right arm action")
                return False
            
            self.grasp_and_move("right", start, end)
            return True
        
        else:
            print(f"[WARNING] Unrecognized action format: {action}")
            return False


if __name__ == "__main__":
    print("This module defines motion primitives for garment folding.")
    print("Import and use MotionPrimitives class in your main script.")

