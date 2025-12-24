#!/usr/bin/env python3
"""
Open the Ruiyan Hand to fully extended position
将瑞依机械手张开到完全伸展状态
"""
import time
import sys
from robot_libs.ruiyan_hand_module import Hand

def open_hand(port='/dev/ttyACM0'):
    """Open the hand to fully extended position"""
    print(f"连接到 {port}...")
    
    try:
        # Connect to hand
        hand = Hand(port=port)
        print("✅ 连接成功！")
        
        # Read current position
        print("\n当前位置:")
        current = hand.get_angles()
        finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
        for i, (name, angle) in enumerate(zip(finger_names, current)):
            print(f"  {name}: {angle:.3f}")
        
        # Set to fully open position
        print("\n正在张开手指...")
        open_position = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
        # Move slowly to open position
        for step in range(5):
            progress = (step + 1) / 5.0
            target = [angle * progress + current[i] * (1 - progress) 
                     for i, angle in enumerate(open_position)]
            hand.set_angles(target)
            print(f"  进度: {int(progress * 100)}%")
            time.sleep(0.5)
        
        # Final set to ensure fully open
        hand.set_angles(open_position)
        time.sleep(1.0)
        
        # Verify final position
        print("\n最终位置:")
        final = hand.get_angles()
        for i, (name, angle) in enumerate(zip(finger_names, final)):
            status = "✅" if angle > 0.8 else "⚠️" if angle > 0.5 else "❌"
            print(f"  {status} {name}: {angle:.3f}")
        
        print("\n✅ 完成！手指已张开")
        
        # Close connection
        hand.close()
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 操作被用户中断")
        if 'hand' in locals():
            hand.close()
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        if 'hand' in locals():
            hand.close()

def main():
    port = '/dev/ttyACM0'  # Left hand
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("="*60)
    print("瑞依机械手 - 张开手指")
    print("Ruiyan Hand - Open Position")
    print("="*60)
    print(f"端口: {port}")
    print("目标: 所有手指完全张开")
    print("="*60)
    print()
    
    open_hand(port)
    
    print("\n注意：如果拇指旋转（Motor 1）无法移动，")
    print("可能是机械卡死或电机故障，需要人工检查。")

if __name__ == "__main__":
    main()

