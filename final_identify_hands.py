#!/usr/bin/env python3
"""
Final script to identify left and right hand ports
最终版本 - 识别左右手端口
"""

import serial
import time
import struct
import os

def test_port(port, baudrate=460800):
    """Test if port has Ruiyan hand and get positions"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.15,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.02)
        
        # Send batch control command
        motor_ids = [1, 2, 3, 4, 5, 6]
        instruction = 0xAA
        
        for motor_id in motor_ids:
            serial_frame = struct.pack(
                "<B B B 2B 3H 1B",
                0xA5, motor_id, 0x00, 0x08, instruction,
                0, 1500, 800, 0x00,
            )
            checksum = sum(serial_frame) & 0xFF
            serial_frame += struct.pack("<B", checksum)
            ser.write(serial_frame)
            time.sleep(0.001)
        
        ser.flush()
        time.sleep(0.03)
        
        response = ser.read(78)
        ser.close()
        
        if len(response) == 78 and response[0] == 0xA5:
            # Parse positions
            positions = []
            for i in range(6):
                motor_data = response[i*13:(i+1)*13]
                if motor_data[0] == 0xA5:
                    finger_data = motor_data[4:12]
                    data_uint64 = struct.unpack("<Q", finger_data)[0]
                    position = (data_uint64 >> 16) & 0xFFF
                    positions.append(position)
                else:
                    positions.append(None)
            return True, positions
        
        return False, None
        
    except Exception as e:
        return False, None

def move_hand(port, target_positions):
    """Send movement command to hand"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=460800,
            timeout=0.1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.01)
        
        motor_ids = [1, 2, 3, 4, 5, 6]
        instruction = 0xAA
        
        for motor_id, target_pos in zip(motor_ids, target_positions):
            serial_frame = struct.pack(
                "<B B B 2B 3H 1B",
                0xA5, motor_id, 0x00, 0x08, instruction,
                target_pos, 1500, 800, 0x00,
            )
            checksum = sum(serial_frame) & 0xFF
            serial_frame += struct.pack("<B", checksum)
            ser.write(serial_frame)
            time.sleep(0.001)
        
        ser.flush()
        ser.close()
        return True
        
    except Exception as e:
        return False

def main():
    print("="*70)
    print("瑞依机械手最终识别工具")
    print("Final Ruiyan Hands Identification Tool")
    print("="*70)
    
    # Step 1: Scan all ports
    print("\n步骤 1: 扫描所有端口")
    print("-" * 70)
    
    all_ports = []
    for i in range(10):
        for prefix in ['/dev/ttyACM', '/dev/ttyUSB']:
            port = f'{prefix}{i}'
            if os.path.exists(port):
                all_ports.append(port)
    
    print(f"找到 {len(all_ports)} 个串口设备: {', '.join(all_ports)}")
    
    found_hands = []
    
    for port in all_ports:
        print(f"  测试 {port}...", end=' ')
        is_hand, positions = test_port(port)
        
        if is_hand:
            print(f"✅ 瑞依机械手")
            found_hands.append({
                'port': port,
                'positions': positions
            })
        else:
            print("❌")
    
    print(f"\n{'='*70}")
    print(f"扫描结果: 找到 {len(found_hands)} 个机械手")
    print(f"{'='*70}")
    
    if len(found_hands) == 0:
        print("\n❌ 未找到任何机械手")
        print("\n请检查:")
        print("  1. 机械手是否已上电")
        print("  2. USB连接是否正常")
        print("  3. 电源指示灯是否亮起")
        return
    
    # Display found hands
    for i, hand in enumerate(found_hands):
        print(f"\n机械手 #{i+1}:")
        print(f"  端口: {hand['port']}")
        motor_names = ["拇指旋转", "拇指", "食指", "中指", "无名指", "小指"]
        print(f"  当前位置:")
        for name, pos in zip(motor_names, hand['positions']):
            if pos is not None:
                normalized = 1.0 - (pos / 4096.0)
                bar_length = 15
                filled = int(bar_length * normalized)
                bar = "█" * filled + "░" * (bar_length - filled)
                print(f"    {name:8s}: [{bar}] {normalized:.2f}")
    
    if len(found_hands) == 1:
        print(f"\n只有 1 个机械手，建议设置为右手")
        print(f"\n配置:")
        print(f"  右手: {found_hands[0]['port']}")
        return
    
    # Step 2: Identify left and right by testing
    print(f"\n{'='*70}")
    print(f"步骤 2: 区分左右手")
    print(f"{'='*70}")
    print(f"\n将依次让每个手的 **食指** 单独弯曲")
    print(f"请观察哪个手在动，然后告诉我它是左手还是右手")
    
    hands_mapping = []
    
    for i, hand in enumerate(found_hands):
        print(f"\n{'='*70}")
        print(f"测试机械手 #{i+1} - 端口: {hand['port']}")
        print(f"{'='*70}")
        
        input(f"\n按 Enter 开始测试 (食指将弯曲 2 次)...")
        
        # Blink index finger twice
        # Motor order: [ThumbRot(1), Thumb(2), Index(3), Middle(4), Ring(5), Pinky(6)]
        # Index is motor 3 (index 2 in array)
        
        for blink in range(2):
            print(f"  第 {blink+1} 次: 食指弯曲...", end='', flush=True)
            # Close index finger (4096 = closed)
            move_hand(hand['port'], [0, 0, 4096, 0, 0, 0])
            time.sleep(1.5)
            
            print(" 张开...", end='', flush=True)
            # Open all fingers
            move_hand(hand['port'], [0, 0, 0, 0, 0, 0])
            time.sleep(1)
            print(" 完成")
        
        # Ask user
        while True:
            identity = input(f"\n这是左手(L)还是右手(R)? [L/R]: ").strip().upper()
            if identity in ['L', 'R']:
                hand_type = "左手" if identity == 'L' else "右手"
                hands_mapping.append({
                    'port': hand['port'],
                    'type': hand_type
                })
                print(f"✅ 已标记为 {hand_type}")
                break
            else:
                print("❌ 请输入 L 或 R")
    
    # Step 3: Show results and code changes
    print(f"\n{'='*70}")
    print(f"识别结果")
    print(f"{'='*70}")
    
    left_port = None
    right_port = None
    
    for mapping in sorted(hands_mapping, key=lambda x: x['type'], reverse=True):
        print(f"  {mapping['type']:4s}: {mapping['port']}")
        if mapping['type'] == "左手":
            left_port = mapping['port']
        else:
            right_port = mapping['port']
    
    # Step 4: Code modification instructions
    print(f"\n{'='*70}")
    print(f"需要修改的代码")
    print(f"{'='*70}")
    
    print(f"\n文件: test_hands_basic_safe.py")
    print(f"  第220行: print(\"1. 右手 ({right_port}) - WCH串口\")")
    print(f"  第221行: print(\"2. 左手 ({left_port}) - WCH串口 (如果连接)\")")
    print(f"  第227行: test_single_hand('{right_port}', '右手')")
    print(f"  第231行: test_single_hand('{left_port}', '左手')")
    print(f"  第236行: success_right = test_single_hand('{right_port}', '右手')")
    print(f"  第242行: test_single_hand('{left_port}', '左手')")
    
    print(f"\n文件: robot_libs/ruiyan_hand_module.py")
    print(f"  第20行: def __init__(self, port='{right_port}', node_id=2):")
    
    # Step 5: Auto apply changes?
    print(f"\n{'='*70}")
    apply = input("\n是否自动应用这些修改? [Y/n]: ").strip().lower()
    
    if apply == 'n' or apply == 'no':
        print("跳过自动修改")
        return
    
    print("\n正在应用修改...")
    
    # Apply changes
    try:
        # Update test_hands_basic_safe.py
        with open('test_hands_basic_safe.py', 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace port references
        content = content.replace(
            'print("1. 右手 (/dev/ttyACM3) - WCH串口")',
            f'print("1. 右手 ({right_port})")'
        )
        content = content.replace(
            'print("2. 左手 (/dev/ttyACM2) - WCH串口 (如果连接)")',
            f'print("2. 左手 ({left_port})")'
        )
        content = content.replace(
            "test_single_hand('/dev/ttyACM3', '右手')",
            f"test_single_hand('{right_port}', '右手')"
        )
        content = content.replace(
            "test_single_hand('/dev/ttyACM2', '左手')",
            f"test_single_hand('{left_port}', '左手')"
        )
        
        with open('test_hands_basic_safe.py', 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("  ✅ test_hands_basic_safe.py 已更新")
        
        # Update ruiyan_hand_module.py
        with open('robot_libs/ruiyan_hand_module.py', 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace default port
        content = content.replace(
            "def __init__(self, port='/dev/ttyACM3', node_id=2):",
            f"def __init__(self, port='{right_port}', node_id=2):"
        )
        
        with open('robot_libs/ruiyan_hand_module.py', 'w', encoding='utf-8') as f:
            f.write(content)
        
        print("  ✅ robot_libs/ruiyan_hand_module.py 已更新")
        
        print(f"\n{'='*70}")
        print("✅ 所有修改已完成!")
        print(f"{'='*70}")
        print(f"\n现在可以运行:")
        print(f"  python test_hands_basic_safe.py")
        
    except Exception as e:
        print(f"\n❌ 自动修改失败: {e}")
        print("请手动修改上述文件")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  被用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()






