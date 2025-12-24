#!/usr/bin/env python3
"""
Test basic movements for the RIGHT Ruiyan Hand
测试右手瑞依机械手的基本动作
"""
import time
import sys
from robot_libs.ruiyan_hand_module import Hand

def test_basic_movements(port='/dev/ttyACM1'):
    """Test basic hand movements"""
    print(f"连接到右手 {port}...")
    
    try:
        # Connect to hand
        hand = Hand(port=port)
        print("✅ 右手连接成功！\n")
        
        finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
        
        # Test 1: Read current position
        print("="*60)
        print("测试 1: 读取当前位置")
        print("="*60)
        current = hand.get_angles()
        for i, (name, angle) in enumerate(zip(finger_names, current)):
            print(f"  {name}: {angle:.3f}")
        print("✅ 读取成功！\n")
        time.sleep(1)
        
        # Test 2: Fully open
        print("="*60)
        print("测试 2: 完全张开")
        print("="*60)
        print("正在张开所有手指...")
        hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(2.5)
        
        actual = hand.get_angles()
        print("实际位置:")
        for i, (name, angle) in enumerate(zip(finger_names, actual)):
            target = 1.0
            error = abs(angle - target)
            status = "✅" if error < 0.15 else "⚠️"
            print(f"  {status} {name}: {angle:.3f} (目标: {target:.2f}, 误差: {error:.3f})")
        print()
        time.sleep(1)
        
        # Test 3: Fully closed (fist)
        print("="*60)
        print("测试 3: 完全握拳")
        print("="*60)
        print("正在握拳...")
        hand.set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(2.5)
        
        actual = hand.get_angles()
        print("实际位置:")
        for i, (name, angle) in enumerate(zip(finger_names, actual)):
            target = 0.0
            error = abs(angle - target)
            status = "✅" if error < 0.15 else "⚠️"
            print(f"  {status} {name}: {angle:.3f} (目标: {target:.2f}, 误差: {error:.3f})")
        print()
        time.sleep(1)
        
        # Test 4: Half grip
        print("="*60)
        print("测试 4: 半握（50%）")
        print("="*60)
        print("正在半握...")
        hand.set_angles([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        time.sleep(2.5)
        
        actual = hand.get_angles()
        print("实际位置:")
        for i, (name, angle) in enumerate(zip(finger_names, actual)):
            target = 0.5
            error = abs(angle - target)
            status = "✅" if error < 0.15 else "⚠️"
            print(f"  {status} {name}: {angle:.3f} (目标: {target:.2f}, 误差: {error:.3f})")
        print()
        time.sleep(1)
        
        # Test 5: Individual finger test
        print("="*60)
        print("测试 5: 单个手指测试")
        print("="*60)
        print("将逐个测试每个手指的独立运动\n")
        
        # Start with all open
        hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(1.5)
        
        for finger_idx in range(6):
            print(f"测试 {finger_names[finger_idx]}...")
            
            # Close only this finger
            target = [1.0] * 6
            target[finger_idx] = 0.0
            hand.set_angles(target)
            time.sleep(1.5)
            
            actual = hand.get_angles()
            closed_angle = actual[finger_idx]
            status = "✅" if closed_angle < 0.2 else "⚠️" if closed_angle < 0.4 else "❌"
            print(f"  {status} 弯曲到: {closed_angle:.3f}")
            
            # Open it back
            hand.set_angles([1.0] * 6)
            time.sleep(1.0)
            
            actual = hand.get_angles()
            open_angle = actual[finger_idx]
            status = "✅" if open_angle > 0.8 else "⚠️" if open_angle > 0.6 else "❌"
            print(f"  {status} 张开到: {open_angle:.3f}\n")
        
        # Test 6: OK gesture (thumb and index touch)
        print("="*60)
        print("测试 6: OK 手势（拇指和食指相触）")
        print("="*60)
        print("正在做 OK 手势...")
        # Close thumb and index, keep others open
        hand.set_angles([0.0, 0.0, 1.0, 1.0, 1.0, 0.3])
        time.sleep(2.5)
        
        actual = hand.get_angles()
        print("实际位置:")
        for i, (name, angle) in enumerate(zip(finger_names, actual)):
            print(f"  {name}: {angle:.3f}")
        print()
        time.sleep(1)
        
        # Test 7: Pointing gesture
        print("="*60)
        print("测试 7: 指向手势（只伸出食指）")
        print("="*60)
        print("正在做指向手势...")
        # Only index open, others closed
        hand.set_angles([0.0, 1.0, 0.0, 0.0, 0.0, 0.3])
        time.sleep(2.5)
        
        actual = hand.get_angles()
        print("实际位置:")
        for i, (name, angle) in enumerate(zip(finger_names, actual)):
            print(f"  {name}: {angle:.3f}")
        print()
        time.sleep(1)
        
        # Final: Return to open position
        print("="*60)
        print("测试完成，恢复到张开状态")
        print("="*60)
        hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(2.0)
        
        print("\n" + "="*60)
        print("✅ 所有基本动作测试完成！")
        print("="*60)
        
        # Close connection
        hand.close()
        print("连接已关闭")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 测试被用户中断")
        if 'hand' in locals():
            print("恢复到张开状态...")
            try:
                hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                time.sleep(1)
            except:
                pass
            hand.close()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if 'hand' in locals():
            try:
                hand.close()
            except:
                pass

def main():
    port = '/dev/ttyACM1'  # Right hand (default)
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("="*60)
    print("瑞依右手机械手 - 基本动作测试")
    print("Ruiyan Right Hand - Basic Movement Test")
    print("="*60)
    print(f"端口: {port}")
    print()
    print("测试项目:")
    print("  1. 读取当前位置")
    print("  2. 完全张开")
    print("  3. 完全握拳")
    print("  4. 半握")
    print("  5. 单个手指测试")
    print("  6. OK 手势")
    print("  7. 指向手势")
    print("="*60)
    print()
    
    input("准备好后按 Enter 开始测试（确保手部周围无障碍物）...")
    print()
    
    test_basic_movements(port)

if __name__ == "__main__":
    main()

