#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple camera test script for RealSense cameras
"""

import cv2
import numpy as np
from robot_libs.realsense_image_module import RealSenseImage
import time

def test_camera_simple():
    """
    Test single camera without serial number (auto-detect first available camera)
    """
    print("=== Testing camera (auto-detect) ===")
    
    # Initialize camera without serial number
    camera = RealSenseImage()
    
    print("Camera initialized successfully!")
    print("Press 'q' to quit, 's' to save current frame")
    
    try:
        frame_count = 0
        while True:
            # Capture frame
            color_image, depth_image = camera.capture_frame()
            
            # Convert depth to colormap for visualization
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # Convert RGB to BGR for OpenCV display
            color_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Add frame info text
            cv2.putText(color_bgr, f"Frame: {frame_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display images
            cv2.imshow('Color Image', color_bgr)
            cv2.imshow('Depth Image', depth_colormap)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('s'):
                # Save images
                timestamp = int(time.time())
                color_filename = f"color_{timestamp}.png"
                depth_filename = f"depth_{timestamp}.png"
                
                cv2.imwrite(color_filename, color_bgr)
                cv2.imwrite(depth_filename, depth_colormap)
                print(f"Saved: {color_filename} and {depth_filename}")
            
            frame_count += 1
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("Camera closed")


def test_camera_with_serial(serial_number):
    """
    Test specific camera with serial number
    
    Args:
        serial_number: Camera serial number (e.g., "134222070573")
    """
    print(f"=== Testing camera with SN: {serial_number} ===")
    
    # Initialize camera with specific serial number
    camera = RealSenseImage(SN_number=serial_number)
    
    print("Camera initialized successfully!")
    print(f"Camera intrinsics: {camera.o3d_intrinsics.intrinsic_matrix}")
    print("Press 'q' to quit, 's' to save current frame")
    
    try:
        frame_count = 0
        while True:
            color_image, depth_image = camera.capture_frame()
            
            # Convert depth to colormap
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # Convert RGB to BGR
            color_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Add frame info text
            cv2.putText(color_bgr, f"Frame: {frame_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display
            cv2.imshow('Color Image', color_bgr)
            cv2.imshow('Depth Image', depth_colormap)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord('s'):
                # Save images
                timestamp = int(time.time())
                color_filename = f"color_{timestamp}.png"
                depth_filename = f"depth_{timestamp}.png"
                
                cv2.imwrite(color_filename, color_bgr)
                cv2.imwrite(depth_filename, depth_colormap)
                print(f"Saved: {color_filename} and {depth_filename}")
            
            frame_count += 1
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print(f"Camera closed (captured {frame_count} frames)")


if __name__ == "__main__":
    import sys
    
    print("RealSense Camera Test")
    print("=" * 50)
    
    # Check if serial number is provided
    if len(sys.argv) > 1:
        serial_number = sys.argv[1]
        test_camera_with_serial(serial_number)
    else:
        print("No serial number provided, using auto-detect mode")
        print("Usage: python test_camera.py [SERIAL_NUMBER]")
        print("Example serial numbers from your project:")
        print("  - 134222070573 (top camera)")
        print("  - 013222072349 (chest camera)")
        print("  - 230322272019 (right camera)")
        print()
        test_camera_simple()

