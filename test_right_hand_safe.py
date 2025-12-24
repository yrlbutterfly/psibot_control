#!/usr/bin/env python3
"""
SAFE version: Test RIGHT Ruiyan Hand step by step with user confirmation
超级安全版本：逐步测试右手，每步需要用户确认
"""
import time
import sys
from robot_libs.ruiyan_hand_module import Hand

def print_current_position(hand, step_name=""):
    """Print current position of all fingers"""
    if step_name:
        print(f"\n[{step_name}] 当前位置:")
    else:
        print("\n当前位置:")
    
    finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
    current = hand.get_angles()
    
    for i, (name, angle) in enumerate(zip(finger_names, current)):
        # Check for errors in position (like stuck at weird values)
        status = "✅" if 0.0 <= angle <= 1.1 else "⚠️"
        print(f"  {status} {name}: {angle:.3f}")
    
    return current

def safe_move(hand, target, description, move_time=3.0, steps=5):
    """
    Safely move hand with smooth interpolation
    
    Args:
        hand: Hand object
        target: Target angles [6 values]
        description: Description of the movement
        move_time: Total time for movement (seconds)
        steps: Number of interpolation steps
    """
    print(f"\n{'='*60}")
    print(f"动作: {description}")
    print(f"{'='*60}")
    
    # Get current position
    current = hand.get_angles()
    
    # Show target
    finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
    print("\n目标位置:")
    for i, (name, angle) in enumerate(zip(finger_names, target)):
        change = target[i] - current[i]
        direction = "→" if abs(change) < 0.05 else "↑" if change > 0 else "↓"
        print(f"  {direction} {name}: {current[i]:.3f} → {angle:.3f} (变化: {change:+.3f})")
    
    # Ask for confirmation
    response = input(f"\n是否执行此动作？[Y/n] (默认Y): ").strip().lower()
    if response == 'n' or response == 'no':
        print("⏭️  跳过此动作")
        return False
    
    # Smooth movement
    print(f"\n正在缓慢移动（{move_time:.1f}秒）...")
    step_time = move_time / steps
    
    for step in range(steps + 1):
        progress = step / steps
        interpolated = [
            current[i] + (target[i] - current[i]) * progress
            for i in range(6)
        ]
        hand.set_angles(interpolated)
        
        # Show progress bar
        bar_length = 30
        filled = int(bar_length * progress)
        bar = "█" * filled + "░" * (bar_length - filled)
        print(f"\r  进度: [{bar}] {int(progress * 100)}%", end='', flush=True)
        
        time.sleep(step_time)
    
    print()  # New line after progress bar
    
    # Verify final position
    time.sleep(0.5)
    final = hand.get_angles()
    
    print("\n实际到达位置:")
    all_good = True
    for i, (name, angle) in enumerate(zip(finger_names, final)):
        error = abs(final[i] - target[i])
        if error < 0.1:
            status = "✅"
        elif error < 0.2:
            status = "⚠️"
            all_good = False
        else:
            status = "❌"
            all_good = False
        print(f"  {status} {name}: {angle:.3f} (目标: {target[i]:.3f}, 误差: {error:.3f})")
    
    if not all_good:
        print("\n⚠️  警告：部分手指未能准确到达目标位置")
        response = input("是否继续测试？[Y/n]: ").strip().lower()
        if response == 'n' or response == 'no':
            return False
    else:
        print("\n✅ 动作完成，位置准确")
    
    return True

def test_safe_movements(port='/dev/ttyACM1'):
    """Safe step-by-step testing"""
    print(f"连接到右手 {port}...")
    
    try:
        # Connect
        hand = Hand(port=port)
        print("✅ 右手连接成功！")
        
        # Step 0: Read initial position
        print("\n" + "="*60)
        print("步骤 0: 读取初始位置")
        print("="*60)
        print_current_position(hand, "初始状态")
        
        input("\n按 Enter 继续...")
        
        # Step 1: Small test - slightly close all fingers (90% open)
        if not safe_move(
            hand,
            [0.9, 0.9, 0.9, 0.9, 0.9, 0.9],
            "微小测试 - 轻微弯曲（90% 张开）",
            move_time=3.0
        ):
            print("\n⏹️  测试终止")
            hand.close()
            return
        
        time.sleep(1)
        
        # Step 2: Return to fully open
        if not safe_move(
            hand,
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "完全张开",
            move_time=3.0
        ):
            print("\n⏹️  测试终止")
            hand.close()
            return
        
        time.sleep(1)
        
        # Step 3: Half grip (conservative test)
        if not safe_move(
            hand,
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            "半握（50%）",
            move_time=4.0
        ):
            print("\n⏹️  测试终止")
            hand.close()
            return
        
        time.sleep(1)
        
        # Step 4: Return to open
        if not safe_move(
            hand,
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "回到完全张开",
            move_time=3.0
        ):
            print("\n⏹️  测试终止")
            hand.close()
            return
        
        time.sleep(1)
        
        # Step 5: Test single finger (index)
        print("\n" + "="*60)
        print("是否继续测试单个手指？")
        print("="*60)
        response = input("继续？[Y/n]: ").strip().lower()
        
        if response != 'n' and response != 'no':
            if not safe_move(
                hand,
                [1.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                "单指测试 - 弯曲食指",
                move_time=2.5
            ):
                print("\n⏹️  测试终止")
                hand.close()
                return
            
            time.sleep(1)
            
            safe_move(
                hand,
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "伸展食指",
                move_time=2.5
            )
        
        time.sleep(1)
        
        # Step 6: Full close (if user wants)
        print("\n" + "="*60)
        print("是否测试完全握拳？（这是最大幅度动作）")
        print("="*60)
        response = input("测试握拳？[y/N] (默认N): ").strip().lower()
        
        if response == 'y' or response == 'yes':
            if not safe_move(
                hand,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "完全握拳",
                move_time=5.0
            ):
                print("\n⏹️  测试终止")
                hand.close()
                return
            
            time.sleep(2)
            
            # Return to open
            safe_move(
                hand,
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "完全张开",
                move_time=4.0
            )
        
        # Final
        print("\n" + "="*60)
        print("✅ 安全测试完成！")
        print("="*60)
        print_current_position(hand, "最终状态")
        
        print("\n右手将保持在当前状态")
        hand.close()
        print("连接已关闭")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 测试被用户中断")
        if 'hand' in locals():
            print("正在恢复到安全位置（张开）...")
            try:
                # Slow return to open position
                current = hand.get_angles()
                for step in range(10):
                    progress = step / 10.0
                    target = [current[i] + (1.0 - current[i]) * progress for i in range(6)]
                    hand.set_angles(target)
                    time.sleep(0.3)
                hand.set_angles([1.0] * 6)
                time.sleep(1)
                print("✅ 已恢复到安全位置")
            except:
                pass
            hand.close()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if 'hand' in locals():
            print("\n尝试恢复到安全位置...")
            try:
                hand.set_angles([1.0] * 6)
                time.sleep(1)
            except:
                pass
            try:
                hand.close()
            except:
                pass

def main():
    port = '/dev/ttyACM1'  # Right hand
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("="*60)
    print("瑞依右手机械手 - 超级安全测试模式")
    print("Ruiyan Right Hand - SAFE Testing Mode")
    print("="*60)
    print(f"端口: {port}\n")
    
    print("🛡️  安全特性:")
    print("  ✓ 所有动作缓慢平滑执行")
    print("  ✓ 每个动作前需要用户确认")
    print("  ✓ 实时显示进度和位置")
    print("  ✓ 检测异常并提示")
    print("  ✓ 可随时中断（Ctrl+C）")
    print("  ✓ 中断后自动恢复到安全位置")
    print()
    
    print("测试流程:")
    print("  1. 读取初始位置")
    print("  2. 微小测试（90% 张开）")
    print("  3. 完全张开")
    print("  4. 半握测试")
    print("  5. 单指测试（可选）")
    print("  6. 完全握拳（可选）")
    print()
    
    print("⚠️  安全提示:")
    print("  • 确保手部周围无障碍物")
    print("  • 观察每个动作的执行情况")
    print("  • 如发现异常立即按 Ctrl+C")
    print("  • 首次使用建议有人在旁协助")
    print("="*60)
    print()
    
    response = input("已理解安全提示，准备开始测试？[Y/n]: ").strip().lower()
    if response == 'n' or response == 'no':
        print("测试取消")
        return
    
    print()
    test_safe_movements(port)

if __name__ == "__main__":
    main()

