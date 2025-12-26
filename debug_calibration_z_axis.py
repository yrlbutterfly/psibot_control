#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试标定坐标系Z轴方向
Debug calibration Z-axis direction
"""

import numpy as np

print("\n" + "="*70)
print("  Calibration Z-Axis Debug")
print("  标定坐标系Z轴调试")
print("="*70 + "\n")

# Load calibration
calib_left = np.load('calibration_results/camera_calibration_left_arm_20251226-234053.npz')
T_cam2base = calib_left['T_cam2base']

print("[1] Calibration Transform Matrix (T_cam2base):")
print(T_cam2base)
print()

# Analyze rotation matrix
R = T_cam2base[:3, :3]
t = T_cam2base[:3, 3]

print("[2] Camera position in base frame (translation):")
print(f"  X = {t[0]:.3f} m")
print(f"  Y = {t[1]:.3f} m")
print(f"  Z = {t[2]:.3f} m  <- Camera height")
print()

print("[3] Camera axis mapping to base frame:")
print(f"  Camera +X maps to base: [{R[0,0]:+.3f}, {R[1,0]:+.3f}, {R[2,0]:+.3f}]")
print(f"  Camera +Y maps to base: [{R[0,1]:+.3f}, {R[1,1]:+.3f}, {R[2,1]:+.3f}]")
print(f"  Camera +Z maps to base: [{R[0,2]:+.3f}, {R[1,2]:+.3f}, {R[2,2]:+.3f}]  <- Forward direction")
print()

# Test transformation with example point
print("[4] Test transformation:")
print("  Example: Point at camera frame Z=-0.906m (90cm forward)")
test_point_cam = np.array([-0.639, -0.104, -0.906, 1])
test_point_base = T_cam2base @ test_point_cam

print(f"  Camera frame: [{test_point_cam[0]:.3f}, {test_point_cam[1]:.3f}, {test_point_cam[2]:.3f}]")
print(f"  Base frame:   [{test_point_base[0]:.3f}, {test_point_base[1]:.3f}, {test_point_base[2]:.3f}]")
print(f"  Base frame Z: {test_point_base[2]:.3f} m  <- Height above ground")
print()

# Check if Z needs to be inverted
print("[5] Z-axis analysis:")
cam_z_to_base_z = R[2, 2]  # How camera Z maps to base Z
print(f"  Camera Z -> Base Z coefficient: {cam_z_to_base_z:.3f}")

if cam_z_to_base_z < 0:
    print("  ⚠️  Camera forward (+Z) maps to base downward (-Z)")
    print("  ⚠️  This might cause height inversion issues!")
else:
    print("  ✓ Camera forward (+Z) maps to base upward (+Z)")
print()

# Suggest fix
print("[6] Recommendations:")
print()
print("Option 1: Check if base coordinate Z-axis definition is correct")
print("  - Standard: Z+ should point UP (away from ground)")
print("  - If Z+ points DOWN, calibration needs to be redone")
print()

print("Option 2: Apply Z-axis correction in code")
print("  - If garment is on table at ~0.8m but shows as 1.5m:")
print("    * Might need to use table height as reference")
print("    * Or apply: Z_corrected = some_reference - Z_base")
print()

# Calculate what table height would need to be for correction
table_z_if_flipped = 2 * t[2] - test_point_base[2]  # Mirror around camera height
print(f"Option 3: If Z should be mirrored around camera height ({t[2]:.3f}m):")
print(f"  - Corrected Z = {table_z_if_flipped:.3f} m")
print(f"  - This would put the garment at {table_z_if_flipped:.3f}m height")
print()

# Test with typical table height
typical_table_height = 0.75  # 75cm
print(f"[7] Reality check:")
print(f"  - Typical table height: {typical_table_height:.2f} m")
print(f"  - Current calculated height: {test_point_base[2]:.3f} m")
print(f"  - Difference: {test_point_base[2] - typical_table_height:.3f} m")
print()

if abs(test_point_base[2] - typical_table_height) > 0.5:
    print("  ⚠️⚠️⚠️  WARNING: Height difference > 50cm!")
    print("  ⚠️  This suggests calibration or coordinate system issue")
    print()
    print("  Possible fixes:")
    print("  1. Re-calibrate with correct base frame Z-axis definition")
    print("  2. Apply Z-correction: Z_corrected = camera_height - (Z_base - camera_height)")
    print(f"     Z_corrected = {t[2]:.3f} - ({test_point_base[2]:.3f} - {t[2]:.3f}) = {2*t[2] - test_point_base[2]:.3f} m")
else:
    print("  ✓ Height is reasonable")

print("\n" + "="*70)

