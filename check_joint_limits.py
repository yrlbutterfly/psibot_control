#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查和修改机械臂关节限位
"""

from robot_libs.realman_arm_module import ArmControl
import numpy as np

def check_limits():
    """检查当前的关节限位设置"""
    print("\n" + "="*70)
    print("  检查机械臂关节限位设置")
    print("="*70)
    
    # 连接左臂和右臂
    print("\n[1] 连接左臂 (192.168.1.18)...")
    left_arm = ArmControl("192.168.1.18")
    
    print("[2] 连接右臂 (192.168.1.19)...")
    right_arm = ArmControl("192.168.1.19")
    
    print("\n" + "-"*70)
    print("左臂限位设置:")
    print("-"*70)
    
    try:
        min_limits = left_arm.robot.Algo_Get_Joint_Min_Limit()
        max_limits = left_arm.robot.Algo_Get_Joint_Max_Limit()
        max_speed = left_arm.robot.Algo_Get_Joint_Max_Speed()
        max_acc = left_arm.robot.Algo_Get_Joint_Max_Acc()
        
        print(f"最小关节限位 (度): {min_limits}")
        print(f"最大关节限位 (度): {max_limits}")
        print(f"最大关节速度 (度/s): {max_speed}")
        print(f"最大关节加速度 (度/s²): {max_acc}")
        
        # 获取当前关节角度
        current_joints = left_arm.get_current_joint()
        current_joints_deg = np.rad2deg(current_joints)
        print(f"\n当前关节角度 (度): {current_joints_deg}")
        
        # 检查是否接近限位
        print("\n关节状态检查:")
        for i, (cur, min_lim, max_lim) in enumerate(zip(current_joints_deg, min_limits, max_limits), 1):
            margin_min = cur - min_lim
            margin_max = max_lim - cur
            status = "✓" if margin_min > 10 and margin_max > 10 else "⚠️"
            print(f"  关节{i}: {cur:7.2f}° | 范围: [{min_lim:7.2f}, {max_lim:7.2f}] | 余量: -{margin_min:6.2f}° / +{margin_max:6.2f}° {status}")
            
    except Exception as e:
        print(f"错误: {e}")
    
    print("\n" + "-"*70)
    print("右臂限位设置:")
    print("-"*70)
    
    try:
        min_limits = right_arm.robot.Algo_Get_Joint_Min_Limit()
        max_limits = right_arm.robot.Algo_Get_Joint_Max_Limit()
        max_speed = right_arm.robot.Algo_Get_Joint_Max_Speed()
        max_acc = right_arm.robot.Algo_Get_Joint_Max_Acc()
        
        print(f"最小关节限位 (度): {min_limits}")
        print(f"最大关节限位 (度): {max_limits}")
        print(f"最大关节速度 (度/s): {max_speed}")
        print(f"最大关节加速度 (度/s²): {max_acc}")
        
        # 获取当前关节角度
        current_joints = right_arm.get_current_joint()
        current_joints_deg = np.rad2deg(current_joints)
        print(f"\n当前关节角度 (度): {current_joints_deg}")
        
        # 检查是否接近限位
        print("\n关节状态检查:")
        for i, (cur, min_lim, max_lim) in enumerate(zip(current_joints_deg, min_limits, max_limits), 1):
            margin_min = cur - min_lim
            margin_max = max_lim - cur
            status = "✓" if margin_min > 10 and margin_max > 10 else "⚠️"
            print(f"  关节{i}: {cur:7.2f}° | 范围: [{min_lim:7.2f}, {max_lim:7.2f}] | 余量: -{margin_min:6.2f}° / +{margin_max:6.2f}° {status}")
            
    except Exception as e:
        print(f"错误: {e}")
    
    # 清理
    print("\n[清理] 关闭连接...")
    left_arm.close()
    right_arm.close()
    print("✓ 完成")
    
    print("\n" + "="*70)
    print("提示:")
    print("  - 如果某个关节余量小于10度，说明接近限位")
    print("  - 可以通过修改 robot_libs/realman_arm_module.py 来调整限位")
    print("="*70)

if __name__ == "__main__":
    check_limits()

