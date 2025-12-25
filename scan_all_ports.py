#!/usr/bin/env python3
"""
Scan all serial ports for Ruiyan hands
扫描所有串口查找瑞依机械手
"""

import serial
import time
import struct
import os

def test_port(port, baudrate=460800):
    """Test if port has Ruiyan hand"""
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
            return True, positions
        
        return False, None
        
    except Exception as e:
        return False, None

def main():
    print("="*70)
    print("全端口扫描 - 查找所有瑞依机械手")
    print("Scanning All Ports for Ruiyan Hands")
    print("="*70)
    
    # Get all ttyACM and ttyUSB devices
    all_ports = []
    for i in range(10):
        for prefix in ['/dev/ttyACM', '/dev/ttyUSB']:
            port = f'{prefix}{i}'
            if os.path.exists(port):
                all_ports.append(port)
    
    if not all_ports:
        print("\n❌ 未找到任何串口设备")
        return
    
    print(f"\n找到 {len(all_ports)} 个串口设备:")
    for port in all_ports:
        print(f"  - {port}")
    
    print(f"\n开始扫描...")
    print("-" * 70)
    
    found_hands = []
    
    for port in all_ports:
        print(f"\n测试: {port} ", end='')
        
        # Test with default baudrate
        is_hand, positions = test_port(port, 460800)
        
        if is_hand:
            print(f"✅ 瑞依机械手!")
            motor_names = ["拇指旋转", "拇指", "食指", "中指", "无名指", "小指"]
            
            print(f"  位置:")
            for name, pos in zip(motor_names, positions):
                normalized = 1.0 - (pos / 4096.0)
                print(f"    {name}: {pos:4d} ({normalized:.2f})")
            
            found_hands.append({
                'port': port,
                'baudrate': 460800,
                'positions': positions
            })
        else:
            print(f"❌")
    
    print(f"\n{'='*70}")
    print(f"扫描结果: 找到 {len(found_hands)} 个机械手")
    print(f"{'='*70}")
    
    if len(found_hands) == 0:
        print("\n❌ 未找到任何瑞依机械手")
        print("\n可能原因:")
        print("  1. 机械手未上电")
        print("  2. USB连接松动")
        print("  3. 波特率不匹配")
        
    elif len(found_hands) == 1:
        print(f"\n✅ 找到 1 个机械手:")
        print(f"   端口: {found_hands[0]['port']}")
        print(f"\n如果你有两个手但只检测到一个，请检查:")
        print(f"  1. 另一个手是否已上电")
        print(f"  2. USB线缆是否连接牢固")
        print(f"  3. 检查机械手是否有错误指示灯")
        
    elif len(found_hands) == 2:
        print(f"\n✅ 找到 2 个机械手:")
        for i, hand in enumerate(found_hands):
            print(f"   {i+1}. {hand['port']}")
        
        print(f"\n💡 使用 identify_hands_ports.py 来区分左右手")
        
    else:
        print(f"\n找到的机械手:")
        for i, hand in enumerate(found_hands):
            print(f"   {i+1}. {hand['port']}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  被用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

