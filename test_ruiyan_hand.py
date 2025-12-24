import time
import numpy as np
import sys
import os
from robot_libs.ruiyan_hand_module import Hand

def test_hand(port_name, hand_type):
    print(f"\n{'='*50}")
    print(f"开始测试 {hand_type} 手 (Port: {port_name})")
    print(f"{'='*50}")

    try:
        # 1. 初始化连接
        print("1. 正在初始化连接...")
        hand = Hand(port=port_name)
        print("   ✅ 连接成功！")

        # 2. 读取当前状态
        print("\n2. 读取当前角度...")
        current_angles = hand.get_angles()
        print(f"   当前角度: {['%.3f' % a for a in current_angles]}")
        print("   ✅ 读取成功！")

        # 3. 简单的动作测试
        print("\n3. 开始动作测试 (请注意手部安全，确周围无遮挡)")
        
        # 动作序列
        actions = [
            ("全张开", [1.0] * 6),
            ("半握",   [0.5] * 6),
            ("握拳",   [0.0] * 6),
            ("全张开", [1.0] * 6)
        ]

        for action_name, target_angles in actions:
            print(f"\n   -> 执行动作: {action_name}")
            hand.set_angles(target_angles)
            time.sleep(2.0) # 等待动作完成
            
            # 验证（可选）
            real_angles = hand.get_angles()
            print(f"      目标: {['%.2f' % a for a in target_angles]}")
            print(f"      实际: {['%.2f' % a for a in real_angles]}")

        # 4. 单指测试 (可选)
        print("\n4. 单指顺序测试")
        finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
        
        # 先全部张开
        hand.set_angles([1.0] * 6)
        time.sleep(1.0)
        
        base_pose = [1.0] * 6
        for i in range(6):
            print(f"   测试: {finger_names[i]}")
            # 弯曲该手指
            target = list(base_pose)
            target[i] = 0.0 
            hand.set_angles(target)
            time.sleep(1.0)
            
            # 恢复
            hand.set_angles(base_pose)
            time.sleep(0.5)

        print("\n✅ 所有测试完成！")
        
        # 结束前张开
        hand.set_angles([1.0] * 6)

    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'hand' in locals():
            hand.close()
            print("连接已关闭")

def main():
    # 检测系统中的USB设备，或者直接指定
    # 根据 control.py 中的配置：
    # Left: /dev/ttyUSB0
    # Right: /dev/ttyUSB1
    
    print("请选择要测试的手:")
    print("1. 左手 (/dev/ttyUSB0)")
    print("2. 右手 (/dev/ttyUSB1)")
    
    choice = input("请输入选项 (1/2): ").strip()
    
    if choice == '1':
        test_hand('/dev/ttyUSB0', 'Left')
    elif choice == '2':
        test_hand('/dev/ttyUSB1', 'Right')
    else:
        print("无效选项")

if __name__ == "__main__":
    main()

