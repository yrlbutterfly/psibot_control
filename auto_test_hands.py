#!/usr/bin/env python3
"""
Auto test to identify which port is which hand
自动测试识别左右手端口
"""

import serial
import time
import struct

def move_finger(port, motor_id, closed=True):
    """Move a specific finger"""
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
        
        # Set all motors to open (0) except the target motor
        for mid in motor_ids:
            if mid == motor_id and closed:
                target_pos = 4096  # Close
            else:
                target_pos = 0  # Open
            
            serial_frame = struct.pack(
                "<B B B 2B 3H 1B",
                0xA5, mid, 0x00, 0x08, instruction,
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
    print("="*70)
    print("机械手端口识别 - 自动测试")
    print("="*70)
    
    port1 = '/dev/ttyACM2'
    port2 = '/dev/ttyACM3'
    
    print(f"\n找到2个机械手:")
    print(f"  端口A: {port1}")
    print(f"  端口B: {port2}")
    
    print(f"\n现在将依次测试每个端口")
    print(f"观察哪个手的食指在动，来确定左右手")
    print(f"\n" + "="*70)
    
    # Test Port 1
    print(f"\n测试端口A: {port1}")
    print(f"正在让食指弯曲 3 次...")
    
    for i in range(3):
        print(f"  第 {i+1} 次: 弯曲...", end='', flush=True)
        move_finger(port1, 3, closed=True)  # Motor 3 is index finger
        time.sleep(1.2)
        print(" 张开...", end='', flush=True)
        move_finger(port1, 3, closed=False)
        time.sleep(0.8)
        print(" ✓")
    
    time.sleep(1)
    
    # Test Port 2
    print(f"\n测试端口B: {port2}")
    print(f"正在让食指弯曲 3 次...")
    
    for i in range(3):
        print(f"  第 {i+1} 次: 弯曲...", end='', flush=True)
        move_finger(port2, 3, closed=True)
        time.sleep(1.2)
        print(" 张开...", end='', flush=True)
        move_finger(port2, 3, closed=False)
        time.sleep(0.8)
        print(" ✓")
    
    print(f"\n{'='*70}")
    print("测试完成!")
    print(f"{'='*70}")
    
    print(f"\n请根据观察到的情况，告诉我:")
    print(f"  端口A ({port1}) 是左手还是右手?")
    print(f"  端口B ({port2}) 是左手还是右手?")
    
    print(f"\n提示:")
    print(f"  - 第一次测试时动的是端口A")
    print(f"  - 第二次测试时动的是端口B")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  被用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()













