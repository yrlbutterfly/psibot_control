#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
List all connected RealSense cameras
"""

import pyrealsense2 as rs

def list_connected_cameras():
    """
    List all connected RealSense cameras with their serial numbers
    """
    print("Scanning for RealSense cameras...")
    print("=" * 60)
    
    # Create a context object to access connected devices
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense cameras found!")
        print("\nPossible reasons:")
        print("1. Camera not plugged in")
        print("2. USB cable issue")
        print("3. Permission issue (try: sudo usermod -aG video $USER)")
        print("4. Driver not installed")
        return []
    
    print(f"Found {len(devices)} RealSense camera(s):\n")
    
    camera_list = []
    for i, device in enumerate(devices):
        print(f"Camera {i+1}:")
        print(f"  Name: {device.get_info(rs.camera_info.name)}")
        print(f"  Serial Number: {device.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")
        print(f"  USB Type: {device.get_info(rs.camera_info.usb_type_descriptor)}")
        
        serial = device.get_info(rs.camera_info.serial_number)
        camera_list.append(serial)
        
        # List available sensors
        print("  Sensors:")
        for sensor in device.query_sensors():
            print(f"    - {sensor.get_info(rs.camera_info.name)}")
        print()
    
    print("=" * 60)
    print("\nTo test a specific camera, use:")
    print(f"  python test_camera.py {camera_list[0] if camera_list else 'SERIAL_NUMBER'}")
    print("\nOr test without serial number (auto-detect):")
    print("  python test_camera.py")
    
    return camera_list


if __name__ == "__main__":
    list_connected_cameras()









