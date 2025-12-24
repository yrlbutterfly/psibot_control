#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定前配置和检查脚本
帮助用户准备标定环境
"""

import os
import sys
import subprocess
import json

def print_header(text):
    print("\n" + "="*60)
    print(f"  {text}")
    print("="*60 + "\n")

def print_step(num, text):
    print(f"\n[步骤 {num}] {text}")
    print("-" * 50)

def check_camera():
    """检查相机连接"""
    print_step(1, "检查相机连接")
    
    try:
        # 尝试导入并列出相机
        from robot_libs.realsense_image_module import RealSenseImage
        import pyrealsense2 as rs
        
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("❌ 未检测到相机！")
            print("   请检查：")
            print("   - 相机USB线是否插好")
            print("   - 是否安装了 librealsense")
            return None
        
        print(f"✅ 检测到 {len(devices)} 个相机")
        camera_list = []
        for i, dev in enumerate(devices):
            serial = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            print(f"   {i+1}. {name} (序列号: {serial})")
            camera_list.append(serial)
        
        return camera_list
    
    except Exception as e:
        print(f"❌ 检查相机时出错: {e}")
        return None

def check_robot_connection(ip):
    """检查机器人网络连接"""
    print(f"\n检查机器人连接 ({ip})...")
    
    # Ping test
    result = subprocess.run(
        ['ping', '-c', '1', '-W', '1', ip],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    if result.returncode == 0:
        print(f"✅ 机器人 {ip} 网络连接正常")
        return True
    else:
        print(f"❌ 无法连接到机器人 {ip}")
        print("   请检查：")
        print("   - 机器人是否开机")
        print("   - 网络连接是否正常")
        print("   - IP地址是否正确")
        return False

def check_apriltag_setup():
    """检查AprilTag准备情况"""
    print_step(3, "检查 AprilTag 准备")
    
    print("请确认以下准备工作已完成：")
    print("  [ ] 已准备 AprilTag 标定板 (75mm)")
    print("  [ ] 标定板已固定在桌面上")
    print("  [ ] 标定板在机器人工作空间内")
    print("  [ ] 光照充足且均匀")
    print("  [ ] 工作区域已清理干净")
    
    response = input("\n以上准备工作是否都已完成？(y/n): ").lower()
    
    if response == 'y':
        print("✅ AprilTag 准备完成")
        return True
    else:
        print("⚠️  请先完成准备工作")
        print("\n如何获取 AprilTag：")
        print("  方法1: 打印 (快速)")
        print("    wget https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag25h9/tag25_09_00000.png")
        print("    然后打印，确保黑色外边缘是 75mm × 75mm")
        print("\n  方法2: 购买")
        print("    淘宝搜索: AprilTag 标定板 75mm tag25h9")
        return False

def update_calibration_config(camera_sn, robot_ip):
    """更新标定程序配置"""
    print_step(4, "更新标定程序配置")
    
    calib_file = "camera_calibreate.py"
    
    # Read file
    with open(calib_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Update camera SN
    content = content.replace(
        'camera_sn="027322071904"',
        f'camera_sn="{camera_sn}"'
    )
    
    # Update robot IP
    content = content.replace(
        'robot_ip="192.168.100.101"',
        f'robot_ip="{robot_ip}"'
    )
    
    # Write back
    with open(calib_file, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"✅ 配置已更新:")
    print(f"   相机序列号: {camera_sn}")
    print(f"   机器人IP: {robot_ip}")

def create_calibration_dirs():
    """创建标定结果目录"""
    os.makedirs("calibration_results", exist_ok=True)
    os.makedirs("calibration_images", exist_ok=True)
    print("✅ 创建了标定结果目录")

def main():
    print_header("手眼标定 - 配置和检查向导")
    
    print("这个脚本会帮你：")
    print("  1. 检查相机连接")
    print("  2. 检查机器人连接")
    print("  3. 确认 AprilTag 准备情况")
    print("  4. 自动配置标定程序")
    
    input("\n按回车开始检查...")
    
    # Step 1: Check camera
    camera_list = check_camera()
    if camera_list is None or len(camera_list) == 0:
        print("\n❌ 无法继续，请先解决相机问题")
        return
    
    # Select camera
    if len(camera_list) == 1:
        selected_camera = camera_list[0]
        print(f"\n已自动选择相机: {selected_camera}")
    else:
        print("\n请选择要用于标定的相机:")
        for i, sn in enumerate(camera_list):
            print(f"  {i+1}. {sn}")
        choice = input("输入编号: ")
        try:
            selected_camera = camera_list[int(choice)-1]
        except:
            print("无效选择")
            return
    
    # Step 2: Check robot
    print_step(2, "检查机器人连接")
    print("\n你有以下机器人:")
    print("  1. 左臂 (192.168.100.100)")
    print("  2. 右臂 (192.168.100.101)")
    print("  3. 其他 (手动输入)")
    
    choice = input("\n选择要标定的机器人 (1/2/3): ")
    
    if choice == '1':
        robot_ip = "192.168.100.100"
    elif choice == '2':
        robot_ip = "192.168.100.101"
    elif choice == '3':
        robot_ip = input("输入机器人IP地址: ")
    else:
        print("无效选择")
        return
    
    # Check connection
    if not check_robot_connection(robot_ip):
        print("\n⚠️  机器人连接失败，但你可以稍后修复")
        cont = input("是否继续配置？(y/n): ")
        if cont.lower() != 'y':
            return
    
    # Step 3: Check AprilTag
    if not check_apriltag_setup():
        print("\n⚠️  请先准备好 AprilTag")
        cont = input("是否仍要继续配置程序？(y/n): ")
        if cont.lower() != 'y':
            return
    
    # Step 4: Update config
    update_calibration_config(selected_camera, robot_ip)
    
    # Step 5: Create directories
    print_step(5, "准备目录结构")
    create_calibration_dirs()
    
    # Summary
    print_header("配置完成！")
    
    print("✅ 所有配置已完成！\n")
    print("下一步操作：")
    print("\n1. 确保 AprilTag 标定板已准备好并固定在桌面")
    print("   (如果还没有，请先打印或购买)")
    print("\n2. 运行标定程序：")
    print("   python camera_calibreate.py")
    print("\n3. 按照屏幕提示操作：")
    print("   - 移动机器人到不同位置")
    print("   - 每次按 'c' 保存")
    print("   - 收集 15-20 个不同位姿")
    print("\n详细操作指南请查看: 手眼标定详细教程.md")
    
    # Save config info
    config_info = {
        "camera_sn": selected_camera,
        "robot_ip": robot_ip,
        "apriltag_size": 0.075,
        "timestamp": str(subprocess.check_output(['date'], text=True).strip())
    }
    
    with open("calibration_config.json", 'w') as f:
        json.dump(config_info, f, indent=2)
    
    print("\n配置信息已保存到: calibration_config.json")
    
    # Ask if ready to start
    print("\n" + "="*60)
    ready = input("是否现在就开始标定？(y/n): ")
    if ready.lower() == 'y':
        print("\n正在启动标定程序...\n")
        subprocess.run(['python', 'camera_calibreate.py'])
    else:
        print("\n准备好后运行: python camera_calibreate.py")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

