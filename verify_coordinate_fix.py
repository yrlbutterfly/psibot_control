#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证坐标系修复是否正确
"""

import numpy as np

print("\n" + "="*70)
print("  Coordinate System Fix Verification")
print("  坐标系修复验证")
print("="*70 + "\n")

# Load calibration
calib = np.load('calibration_results/camera_calibration_left_arm_20251226-234053.npz')
T_cam2base = calib['T_cam2base']

print("[1] Calibration Matrix:")
print(f"  Camera position in base: {T_cam2base[:3, 3]}")
print()

# Example point from terminal output
print("[2] Test Case from Terminal Output:")
print("  Point cloud coordinate (after generate_pcd transform):")
point_transformed = np.array([-0.639, -0.104, -0.906])
print(f"    [{point_transformed[0]:.3f}, {point_transformed[1]:.3f}, {point_transformed[2]:.3f}] m")
print()

print("[3] Previous (WRONG) method - Direct transform:")
center_homo_wrong = np.append(point_transformed, 1)
result_wrong = (T_cam2base @ center_homo_wrong)[:3]
print(f"  Base frame: [{result_wrong[0]:.3f}, {result_wrong[1]:.3f}, {result_wrong[2]:.3f}] m")
print(f"  Z = {result_wrong[2]:.3f} m  <- TOO HIGH! (should be ~0.75m)")
print()

print("[4] New (CORRECT) method - Undo transform first:")
print("  Step 1: Undo generate_pcd transform")
point_original = point_transformed.copy()
point_original[1] = -point_original[1]  # Undo Y flip
point_original[2] = -point_original[2]  # Undo Z flip
print(f"    [{point_transformed[0]:.3f}, {point_transformed[1]:.3f}, {point_transformed[2]:.3f}] -> "
      f"[{point_original[0]:.3f}, {point_original[1]:.3f}, {point_original[2]:.3f}]")
print()

print("  Step 2: Apply T_cam2base")
center_homo_correct = np.append(point_original, 1)
result_correct = (T_cam2base @ center_homo_correct)[:3]
print(f"  Base frame: [{result_correct[0]:.3f}, {result_correct[1]:.3f}, {result_correct[2]:.3f}] m")
print(f"  Z = {result_correct[2]:.3f} m  <- Should be reasonable table height!")
print()

print("[5] Comparison:")
print(f"  Wrong method Z: {result_wrong[2]:.3f} m")
print(f"  Correct method Z: {result_correct[2]:.3f} m")
print(f"  Difference: {abs(result_wrong[2] - result_correct[2]):.3f} m")
print()

# Expected table height
table_height = 0.75
print("[6] Reality Check:")
print(f"  Expected table height: ~{table_height:.2f} m")
print(f"  Correct method Z: {result_correct[2]:.3f} m")
diff = abs(result_correct[2] - table_height)
print(f"  Difference: {diff:.3f} m")

if diff < 0.15:  # Within 15cm
    print(f"  ✓✓✓ EXCELLENT! Within 15cm of expected height")
elif diff < 0.30:  # Within 30cm
    print(f"  ✓✓ GOOD! Within 30cm of expected height")
elif diff < 0.50:  # Within 50cm
    print(f"  ✓ OK! Within 50cm - may need fine tuning")
else:
    print(f"  ✗ Still has issues - difference > 50cm")

print("\n" + "="*70)
print("  Conclusion:")
if diff < 0.30:
    print("  ✓ Coordinate system fix is working correctly!")
    print("  ✓ 坐标系修复正确!")
else:
    print("  ⚠️  May need further adjustment")
print("="*70 + "\n")

