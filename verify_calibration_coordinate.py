#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证标定结果的坐标系是否正确
Verify if calibration coordinate system is correct
"""

import numpy as np
import os


def analyze_calibration_logic(calib_file):
    """
    Analyze calibration logic and coordinate transformation
    
    Args:
        calib_file: Path to calibration npz file
    """
    print("\n" + "="*70)
    print("  Calibration Coordinate System Verification")
    print("  标定坐标系验证")
    print("="*70)
    
    if not os.path.exists(calib_file):
        print(f"[ERROR] Calibration file not found: {calib_file}")
        return
    
    # Load calibration
    calib = np.load(calib_file)
    T_cam2base = calib['T_cam2base']
    
    R = T_cam2base[:3, :3]
    t = T_cam2base[:3, 3]
    
    print(f"\n[Loaded Calibration File]")
    print(f"  {calib_file}")
    
    print(f"\n[Transformation Matrix T_cam2base]")
    print("  (Transforms points from camera frame to base frame)")
    for i in range(4):
        print(f"  [{T_cam2base[i, 0]:7.3f}, {T_cam2base[i, 1]:7.3f}, {T_cam2base[i, 2]:7.3f}, {T_cam2base[i, 3]:7.3f}]")
    
    # Camera position in base frame
    print(f"\n[Camera Position in Base Frame]")
    print(f"  Position: ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m")
    print(f"  Height above base: {t[2]:.3f} m")
    
    # Coordinate system analysis
    print(f"\n" + "="*70)
    print("  Coordinate System Analysis")
    print("="*70)
    
    print("\n[RealSense D435i Camera Coordinate System (Original)]")
    print("  As per Intel RealSense documentation:")
    print("    +X: Right (when facing the camera)")
    print("    +Y: Down")
    print("    +Z: Forward (depth direction)")
    print("  ")
    print("  For a point at depth=0.9m in front of camera:")
    print("    Original: (0, 0, +0.9) m")
    
    print("\n[After pcd.transform in generate_pcd]")
    print("  Code applies: [[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]]")
    print("  This transform:")
    print("    +X: Right (unchanged)")
    print("    +Y: Up (flipped from Down)")
    print("    +Z: Backward (flipped from Forward)")
    print("  ")
    print("  Same point after transform:")
    print("    Transformed: (0, 0, -0.9) m")
    print("    ⚠️  Z is now NEGATIVE for points in front of camera!")
    
    print("\n[Robot Base Coordinate System (Expected)]")
    print("  Typical robot base frame:")
    print("    +X: Forward")
    print("    +Y: Left")
    print("    +Z: Up")
    print("  ")
    print("  A point on table surface should have:")
    print("    Base Z ≈ 0.0 to 0.2 m (slightly above base)")
    
    # Test transformation
    print("\n" + "="*70)
    print("  Test Point Transformation")
    print("="*70)
    
    # Test point: 0.9m in front of camera center
    print("\n[Test 1: Point at camera center, depth=0.9m]")
    p_cam_original = np.array([0.0, 0.0, 0.9, 1.0])  # Original RealSense coordinates
    print(f"  Original camera frame: ({p_cam_original[0]:.3f}, {p_cam_original[1]:.3f}, {p_cam_original[2]:.3f}) m")
    
    # After pcd.transform
    T_flip = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    p_cam_flipped = T_flip @ p_cam_original
    print(f"  After flip transform: ({p_cam_flipped[0]:.3f}, {p_cam_flipped[1]:.3f}, {p_cam_flipped[2]:.3f}) m")
    
    # Transform to base frame
    p_base = T_cam2base @ p_cam_flipped
    print(f"  Base frame: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f}) m")
    
    # Analysis
    print(f"\n[Analysis]")
    print(f"  Camera is at height: {t[2]:.3f} m")
    print(f"  Point is at height: {p_base[2]:.3f} m")
    print(f"  Difference: {p_base[2] - t[2]:.3f} m")
    
    if p_base[2] > t[2]:
        print(f"  ⚠️  Point is ABOVE camera! This seems wrong.")
        print(f"      A point in front of camera should be at table height (below camera)")
    elif abs(p_base[2]) < 0.3:
        print(f"  ✓ Point is near table height, looks reasonable")
    else:
        print(f"  ⚠️  Check if this makes sense for your setup")
    
    # Test another point
    print("\n[Test 2: Point 30cm left of camera center, depth=0.9m]")
    p_cam_original_2 = np.array([-0.3, 0.0, 0.9, 1.0])  # Left of camera
    print(f"  Original camera frame: ({p_cam_original_2[0]:.3f}, {p_cam_original_2[1]:.3f}, {p_cam_original_2[2]:.3f}) m")
    
    p_cam_flipped_2 = T_flip @ p_cam_original_2
    print(f"  After flip transform: ({p_cam_flipped_2[0]:.3f}, {p_cam_flipped_2[1]:.3f}, {p_cam_flipped_2[2]:.3f}) m")
    
    p_base_2 = T_cam2base @ p_cam_flipped_2
    print(f"  Base frame: ({p_base_2[0]:.3f}, {p_base_2[1]:.3f}, {p_base_2[2]:.3f}) m")
    
    # Recommendations
    print("\n" + "="*70)
    print("  Diagnosis & Recommendations")
    print("="*70)
    
    print("\n[Current Issue]")
    print("  Points in front of camera are being transformed to positions")
    print("  ABOVE the camera in base frame, which is physically incorrect.")
    
    print("\n[Likely Cause]")
    print("  The coordinate flip in generate_pcd() creates Z-axis pointing")
    print("  backward, and the calibration T_cam2base might have been")
    print("  computed with this flip already applied.")
    
    print("\n[Solution Options]")
    print("  Option 1: Remove the flip transform and re-calibrate")
    print("    - Comment out line 99 in realsense_image_module.py")
    print("    - Re-run camera_calibreate.py")
    print("    - Use natural RealSense coordinate system")
    
    print("\n  Option 2: Fix the flip direction")
    print("    - Change the flip to match expected orientation")
    print("    - Re-calibrate with new flip")
    
    print("\n  Option 3: Keep current setup but invert Z in motion planning")
    print("    - Apply inverse transform before sending to robot")
    print("    - This is a workaround, not recommended")
    
    print("\n[Recommended Action]")
    print("  1. Check your actual robot setup:")
    print("     - Measure camera height above table")
    print("     - Measure target object height above table")
    print("     - These should make physical sense")
    print("  ")
    print("  2. If calibration is wrong:")
    print("     - Re-run calibration WITHOUT the flip transform")
    print("     - Or adjust flip to match your coordinate convention")
    
    print("\n" + "="*70)


def main():
    """Main function"""
    CALIB_FILE_LEFT = "calibration_results/camera_calibration_left_arm_20251222-224450.npz"
    
    analyze_calibration_logic(CALIB_FILE_LEFT)
    
    print("\n[Next Steps]")
    print("  1. Physically measure your setup:")
    print("     - Camera height from table: ____ m")
    print("     - Typical target object height: ____ m")
    print("  ")
    print("  2. Compare with debug output to see if it matches reality")
    print("  ")
    print("  3. If coordinates don't match, consider re-calibration")


if __name__ == "__main__":
    main()

