#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证新的坐标系统（移除generate_pcd变换后）
"""

import numpy as np

print("\n" + "="*70)
print("  New Coordinate System Verification")
print("  新坐标系统验证")
print("="*70 + "\n")

# Load calibration
calib = np.load('calibration_results/camera_calibration_left_arm_20251222-224450.npz')
T_cam2base = calib['T_cam2base']

print("[1] Calibration Matrix:")
print(f"  Camera position in base: {T_cam2base[:3, 3]}")
print(f"  Camera height: {T_cam2base[:3, 3][2]:.3f} m")
print()

# Example: Point at typical garment position
# Assuming garment is ~90cm forward, slightly right, at camera level
print("[2] Test Case - Garment Point:")
print("  Scenario: Garment on table, ~90cm in front of camera")
print()

# NEW coordinate system (standard camera coords: +Z forward)
point_cam_new = np.array([-0.639, 0.104, 0.906])  # X, Y, Z (Z positive = forward)
print("  Camera frame (NEW - standard coords):")
print(f"    X = {point_cam_new[0]:+.3f} m (left/right)")
print(f"    Y = {point_cam_new[1]:+.3f} m (up/down)")
print(f"    Z = {point_cam_new[2]:+.3f} m (forward depth)")
print()

# Transform to base frame
point_homo = np.append(point_cam_new, 1)
point_base = (T_cam2base @ point_homo)[:3]

print("  Base frame:")
print(f"    X = {point_base[0]:+.3f} m")
print(f"    Y = {point_base[1]:+.3f} m")
print(f"    Z = {point_base[2]:+.3f} m  <- Height above ground")
print()

# Expected table height
table_height = 0.75
print("[3] Reality Check:")
print(f"  Expected table height: ~{table_height:.2f} m")
print(f"  Calculated height: {point_base[2]:.3f} m")
diff = abs(point_base[2] - table_height)
print(f"  Difference: {diff:.3f} m")
print()

if diff < 0.10:  # Within 10cm
    print(f"  ✓✓✓ EXCELLENT! Within 10cm of expected height")
    status = "perfect"
elif diff < 0.20:  # Within 20cm
    print(f"  ✓✓ GOOD! Within 20cm of expected height")
    status = "good"
elif diff < 0.30:  # Within 30cm
    print(f"  ✓ OK! Within 30cm - acceptable")
    status = "ok"
else:
    print(f"  ✗ Still has issues - difference > 30cm")
    status = "bad"

print()
print("[4] Expected vs Calculated:")
print(f"  Position relative to robot base:")
print(f"    X: {point_base[0]:.3f} m {'(left of base)' if point_base[0] < 0 else '(right of base)'}")
print(f"    Y: {point_base[1]:.3f} m {'(behind base)' if point_base[1] < 0 else '(in front of base)'}")
print(f"    Z: {point_base[2]:.3f} m (height)")
print()

# Check if position is reachable
print("[5] Reachability Check:")
horizontal_dist = np.sqrt(point_base[0]**2 + point_base[1]**2)
print(f"  Horizontal distance from base: {horizontal_dist:.3f} m")

if horizontal_dist < 0.8 and 0.2 < point_base[2] < 1.0:
    print(f"  ✓ Position appears reachable by robot arm")
else:
    print(f"  ⚠️  Position may be out of reach")

print("\n" + "="*70)
print("  Conclusion:")
if status in ["perfect", "good"]:
    print("  ✓✓✓ Coordinate system is working correctly!")
    print("  ✓✓✓ 坐标系统工作正常!")
    print("  Ready to test with real robot.")
elif status == "ok":
    print("  ✓ Coordinate system is acceptable")
    print("  You can proceed with testing")
else:
    print("  ✗ Coordinate system needs further adjustment")
print("="*70 + "\n")

