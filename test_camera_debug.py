#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealSense Camera Debug Tool
测试相机连接和初始化
"""

import pyrealsense2 as rs
import sys

def test_camera_connection():
    """测试相机连接"""
    print("=" * 60)
    print("  RealSense Camera Diagnostic Tool")
    print("=" * 60)
    
    # Step 1: List all devices
    print("\n[1] Enumerating devices...")
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("✗ No RealSense devices found!")
        return False
    
    print(f"✓ Found {len(devices)} device(s)")
    
    for i, dev in enumerate(devices):
        print(f"\nDevice {i}:")
        print(f"  Name: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
    
    # Step 2: Test each device
    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        print(f"\n[2] Testing device {i} (SN: {serial})...")
        
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        
        try:
            # Try to start with minimal config
            print("  Attempting to start pipeline...")
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            profile = pipeline.start(config)
            print("  ✓ Pipeline started successfully")
            
            # Try to get a frame
            print("  Waiting for frames (timeout=5s)...")
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if depth_frame and color_frame:
                print(f"  ✓ Successfully captured frames!")
                print(f"    Depth: {depth_frame.get_width()}x{depth_frame.get_height()}")
                print(f"    Color: {color_frame.get_width()}x{color_frame.get_height()}")
                
                pipeline.stop()
                return True
            else:
                print("  ✗ Frames are invalid")
                pipeline.stop()
                return False
                
        except RuntimeError as e:
            print(f"  ✗ Runtime error: {e}")
            try:
                pipeline.stop()
            except:
                pass
            return False
        except Exception as e:
            print(f"  ✗ Unexpected error: {e}")
            try:
                pipeline.stop()
            except:
                pass
            return False
    
    return True

def test_specific_device(serial_number):
    """测试指定序列号的设备"""
    print(f"\n[3] Testing specific device: {serial_number}")
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    
    try:
        print("  Configuring streams...")
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        print("  Starting pipeline...")
        pipeline.start(config)
        
        print("  Warming up (capturing 10 frames)...")
        for i in range(10):
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            print(f"    Frame {i+1}/10: OK")
        
        print("  ✓ Device is working normally!")
        pipeline.stop()
        return True
        
    except Exception as e:
        print(f"  ✗ Error: {e}")
        try:
            pipeline.stop()
        except:
            pass
        return False

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  开始诊断...")
    print("=" * 60)
    
    # Test connection
    if test_camera_connection():
        print("\n" + "=" * 60)
        print("  ✓ Basic test passed!")
        print("=" * 60)
        
        # Test specific device if provided
        if len(sys.argv) > 1:
            serial = sys.argv[1]
            test_specific_device(serial)
    else:
        print("\n" + "=" * 60)
        print("  ✗ Camera test failed!")
        print("=" * 60)
        print("\n可能的解决方案：")
        print("  1. 物理重连相机USB线")
        print("  2. 检查USB端口（使用USB 3.0）")
        print("  3. 重启电脑")
        print("  4. 更新固件")













