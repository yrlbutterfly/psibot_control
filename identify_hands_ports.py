#!/usr/bin/env python3
"""
Identify left and right hand ports
识别左手和右手的端口
"""

import serial
import time
import struct

def test_ruiyan_hand_connection(port):
    """Test if port has a working Ruiyan hand and get position data"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=460800,
            timeout=0.1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.01)
        
        # Send control commands to all motors
        motor_ids = [1, 2, 3, 4, 5, 6]
        instruction = 0xAA  # CTRL_MOTOR_POSITION_VELOCITY_CURRENT
        
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
        time.sleep(0.02)
        
        # Read batch response
        response = ser.read(78)
        
        ser.close()
        
        if len(response) == 78:
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
        return False, str(e)

def make_hand_move(port, target_positions):
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
        print(f"错误: {e}")
        return False

def main():
    print("="*60)
    print("瑞依机械手端口识别工具")
    print("Identify Left and Right Hand Ports")
    print("="*60)
    
    # Scan WCH USB Quad Serial ports
    ports_to_scan = [
        '/dev/ttyACM2',
        '/dev/ttyACM3',
        '/dev/ttyACM4',
        '/dev/ttyACM5'
    ]
    
    found_hands = []
    
    print("\n正在扫描端口...")
    print("-" * 60)
    
    for port in ports_to_scan:
        print(f"\n测试: {port}")
        is_hand, result = test_ruiyan_hand_connection(port)
        
        if is_hand:
            print(f"  ✅ 发现瑞依机械手!")
            print(f"  当前位置 (原始值 0-4096):")
            
            motor_names = ["拇指旋转", "拇指", "食指", "中指", "无名指", "小指"]
            for name, pos in zip(motor_names, result):
                if pos is not None:
                    # Convert to 0-1 range (inverted: 0=open, 4096=closed)
                    normalized = 1.0 - (pos / 4096.0)
                    bar_length = 20
                    filled = int(bar_length * normalized)
                    bar = "█" * filled + "░" * (bar_length - filled)
                    print(f"    {name:8s}: {pos:4d} [{bar}] {normalized:.3f}")
            
            found_hands.append({
                'port': port,
                'positions': result
            })
        else:
            print(f"  ❌ 无响应或通信失败")
    
    print("\n" + "="*60)
    print(f"扫描完成，找到 {len(found_hands)} 个机械手")
    print("="*60)
    
    if len(found_hands) == 0:
        print("\n❌ 未找到任何瑞依机械手")
        print("\n可能原因:")
        print("  1. 机械手未连接或未上电")
        print("  2. USB连接问题")
        print("  3. 端口被其他程序占用")
        return
    
    elif len(found_hands) == 1:
        print(f"\n✅ 找到 1 个机械手:")
        print(f"   端口: {found_hands[0]['port']}")
        print(f"\n💡 如果只有一个手,建议将其设置为右手")
        
    elif len(found_hands) == 2:
        print(f"\n✅ 找到 2 个机械手，现在进行区分测试...")
        print("\n将依次让每个手的 **食指** 弯曲，以便识别左右手")
        print("观察哪个手的食指在动，来确定是左手还是右手")
        
        hands_mapping = []
        
        for i, hand in enumerate(found_hands):
            print(f"\n{'='*60}")
            print(f"测试手 #{i+1} - 端口: {hand['port']}")
            print(f"{'='*60}")
            
            input(f"\n按 Enter 开始测试手 #{i+1} (食指将弯曲)...")
            
            # Motor mapping: [ThumbRot, Thumb, Index, Middle, Ring, Pinky]
            # Index finger is motor 3 (index 2 in the list)
            # Close only index finger (4096 = fully closed)
            test_positions = [0, 0, 4096, 0, 0, 0]
            
            print(f"正在让食指弯曲...")
            make_hand_move(hand['port'], test_positions)
            time.sleep(2)
            
            print(f"正在恢复张开...")
            make_hand_move(hand['port'], [0, 0, 0, 0, 0, 0])
            time.sleep(1)
            
            # Ask user to identify
            while True:
                identity = input(f"\n这是左手(L)还是右手(R)? [L/R]: ").strip().upper()
                if identity in ['L', 'R']:
                    hand_type = "左手" if identity == 'L' else "右手"
                    hands_mapping.append({
                        'port': hand['port'],
                        'type': hand_type
                    })
                    print(f"✅ 已标记 {hand['port']} 为 {hand_type}")
                    break
                else:
                    print("❌ 请输入 L 或 R")
        
        # Show final mapping
        print(f"\n{'='*60}")
        print("识别结果")
        print(f"{'='*60}")
        
        for mapping in hands_mapping:
            print(f"  {mapping['type']:4s}: {mapping['port']}")
        
        # Generate code suggestions
        print(f"\n{'='*60}")
        print("代码修改建议")
        print(f"{'='*60}")
        
        left_port = None
        right_port = None
        
        for mapping in hands_mapping:
            if mapping['type'] == "左手":
                left_port = mapping['port']
            else:
                right_port = mapping['port']
        
        if left_port and right_port:
            print(f"\n修改 test_hands_basic_safe.py:")
            print(f"  第220行: print(\"1. 右手 ({right_port}) - WCH串口\")")
            print(f"  第221行: print(\"2. 左手 ({left_port}) - WCH串口 (如果连接)\")")
            print(f"  第227行: test_single_hand('{right_port}', '右手')")
            print(f"  第231行: test_single_hand('{left_port}', '左手')")
            print(f"  第236行: success_right = test_single_hand('{right_port}', '右手')")
            print(f"  第242行: test_single_hand('{left_port}', '左手')")
    
    else:
        print(f"\n⚠️  找到 {len(found_hands)} 个机械手 (超过预期的2个)")
        print("\n端口列表:")
        for i, hand in enumerate(found_hands):
            print(f"  {i+1}. {hand['port']}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  被用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()











