#!/usr/bin/env python3
"""
Safe basic functionality test for Ruiyan Hands - Open and Close only
安全的瑞依机械手基本功能测试 - 仅测试张开和闭合
"""
import time
import sys
from robot_libs.ruiyan_hand_module import Hand

def smooth_move(hand, target, duration=5.0, steps=15):
    """
    Smoothly move hand to target position
    
    Args:
        hand: Hand object
        target: Target angles [6 values]
        duration: Total time for movement (seconds)
        steps: Number of interpolation steps
    """
    current = hand.get_angles()
    step_time = duration / steps
    
    for step in range(steps + 1):
        progress = step / steps
        interpolated = [
            current[i] + (target[i] - current[i]) * progress
            for i in range(6)
        ]
        hand.set_angles(interpolated)
        
        # Progress bar
        bar_length = 30
        filled = int(bar_length * progress)
        bar = "█" * filled + "░" * (bar_length - filled)
        print(f"\r  进度: [{bar}] {int(progress * 100):3d}%", end='', flush=True)
        
        time.sleep(step_time)
    
    print()  # New line
    time.sleep(0.5)

def show_position(hand, hand_name=""):
    """Display current position of all fingers"""
    finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
    angles = hand.get_angles()
    
    if hand_name:
        print(f"\n{hand_name} 当前位置:")
    else:
        print("\n当前位置:")
    
    for name, angle in zip(finger_names, angles):
        # Visual bar
        bar_length = 20
        filled = int(bar_length * angle)
        bar = "█" * filled + "░" * (bar_length - filled)
        print(f"  {name:8s} [{bar}] {angle:.3f}")
    
    return angles

def test_single_hand(port, hand_name):
    """Test open and close for a single hand"""
    print(f"\n{'='*60}")
    print(f"测试 {hand_name}")
    print(f"端口: {port}")
    print(f"{'='*60}")
    
    try:
        # Connect
        print(f"\n正在连接 {hand_name}...")
        hand = Hand(port=port)
        print(f"✅ {hand_name} 连接成功！")
        
        # Show initial position
        print(f"\n{'='*60}")
        print("步骤 1: 读取初始位置")
        print(f"{'='*60}")
        show_position(hand, hand_name)
        
        input("\n按 Enter 继续...")
        
        # Test 1: Move to fully open
        print(f"\n{'='*60}")
        print("步骤 2: 完全张开")
        print(f"{'='*60}")
        print(f"正在缓慢张开 {hand_name}（5秒）...")
        smooth_move(hand, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], duration=5.0)
        
        final = show_position(hand, hand_name)
        
        # Check if opened successfully
        avg_open = sum(final) / len(final)
        if avg_open > 0.8:
            print(f"✅ {hand_name} 成功张开（平均: {avg_open:.2f}）")
        else:
            print(f"⚠️  {hand_name} 未完全张开（平均: {avg_open:.2f}）")
        
        time.sleep(1)
        
        # Test 2: Move to fully closed
        print(f"\n{'='*60}")
        print("步骤 3: 完全闭合（握拳）")
        print(f"{'='*60}")
        
        response = input(f"是否测试 {hand_name} 握拳？[Y/n] (默认Y): ").strip().lower()
        if response == 'n' or response == 'no':
            print(f"⏭️  跳过 {hand_name} 握拳测试")
        else:
            print(f"正在缓慢握拳 {hand_name}（6秒）...")
            smooth_move(hand, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=6.0)
            
            final = show_position(hand, hand_name)
            
            # Check if closed successfully
            avg_closed = sum(final) / len(final)
            if avg_closed < 0.2:
                print(f"✅ {hand_name} 成功握拳（平均: {avg_closed:.2f}）")
            else:
                print(f"⚠️  {hand_name} 未完全闭合（平均: {avg_closed:.2f}）")
            
            time.sleep(1)
            
            # Return to open
            print(f"\n正在恢复 {hand_name} 到张开状态（5秒）...")
            smooth_move(hand, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], duration=5.0)
            show_position(hand, hand_name)
        
        # Test 3: Quick open-close cycle
        print(f"\n{'='*60}")
        print("步骤 4: 快速循环测试（张开-闭合-张开）")
        print(f"{'='*60}")
        
        response = input(f"是否测试 {hand_name} 快速循环？[y/N] (默认N): ").strip().lower()
        if response == 'y' or response == 'yes':
            print(f"\n开始 {hand_name} 快速循环测试...")
            
            for i in range(3):
                print(f"\n  循环 {i+1}/3:")
                print(f"    → 闭合...")
                smooth_move(hand, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=4.0)
                time.sleep(0.5)
                
                print(f"    → 张开...")
                smooth_move(hand, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], duration=4.0)
                time.sleep(0.5)
            
            print(f"\n✅ {hand_name} 快速循环测试完成")
            show_position(hand, hand_name)
        else:
            print(f"⏭️  跳过 {hand_name} 快速循环测试")
        
        # Summary
        print(f"\n{'='*60}")
        print(f"✅ {hand_name} 测试完成")
        print(f"{'='*60}")
        
        # Ensure hand is in safe position
        print(f"\n确保 {hand_name} 处于安全状态（张开）...")
        hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        time.sleep(1)
        
        hand.close()
        print(f"✅ {hand_name} 连接已关闭")
        
        return True
        
    except KeyboardInterrupt:
        print(f"\n\n⚠️  {hand_name} 测试被用户中断")
        if 'hand' in locals():
            print(f"正在恢复 {hand_name} 到安全位置...")
            try:
                hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                time.sleep(1)
                print(f"✅ {hand_name} 已恢复到安全位置")
            except:
                pass
            hand.close()
        return False
        
    except Exception as e:
        print(f"\n❌ {hand_name} 测试错误: {e}")
        import traceback
        traceback.print_exc()
        if 'hand' in locals():
            try:
                hand.set_angles([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                time.sleep(1)
                hand.close()
            except:
                pass
        return False

def main():
    print("="*60)
    print("瑞依机械手 - 基本功能安全测试")
    print("Ruiyan Hands - Safe Basic Function Test")
    print("="*60)
    print()
    print("测试项目:")
    print("  1. 读取初始位置")
    print("  2. 完全张开")
    print("  3. 完全闭合（握拳）")
    print("  4. 快速循环（可选）")
    print()
    print("🛡️  安全特性:")
    print("  ✓ 所有动作缓慢平滑执行")
    print("  ✓ 实时显示位置和进度")
    print("  ✓ 可随时中断（Ctrl+C）")
    print("  ✓ 自动恢复到安全位置")
    print()
    print("⚠️  安全提示:")
    print("  • 确保手部周围无障碍物")
    print("  • 观察每个动作的执行情况")
    print("  • 如发现异常立即按 Ctrl+C")
    print("="*60)
    print()
    
    # Ask which hand(s) to test
    print("请选择要测试的手:")
    print("1. 右手 (/dev/ttyACM3) - WCH串口")
    print("2. 左手 (/dev/ttyACM2) - WCH串口 (如果连接)")
    print("3. 双手（依次测试）")
    print()
    
    choice = input("请输入选项 (1/2/3): ").strip()
    
    if choice == '1':
        print("\n开始测试右手...")
        test_single_hand('/dev/ttyACM3', '右手')
        
    elif choice == '2':
        print("\n开始测试左手...")
        test_single_hand('/dev/ttyACM2', '左手')
        
    elif choice == '3':
        print("\n开始测试双手...")
        
        # Test right hand first
        success_right = test_single_hand('/dev/ttyACM3', '右手')
        
        if success_right:
            print("\n" + "="*60)
            input("\n右手测试完成。按 Enter 继续测试左手...")
            
            # Test left hand
            test_single_hand('/dev/ttyACM2', '左手')
        
        print("\n" + "="*60)
        print("✅ 双手测试完成")
        print("="*60)
    else:
        print("❌ 无效选项")
        return
    
    print("\n" + "="*60)
    print("测试结束！所有机械手已恢复到安全位置（张开）")
    print("="*60)

if __name__ == "__main__":
    main()

