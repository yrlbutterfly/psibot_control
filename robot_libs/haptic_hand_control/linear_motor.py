import time
import struct
import argparse

from collections import deque
import numpy as np
import serial

class LinearMotorController:
    """
    线性马达控制器类
    用于控制6个手指的触觉反馈振动
    通过手套串口发送控制指令
    """
    def __init__(self, serial_client, hand_name):
        """
        初始化线性马达控制器
        
        参数:
            serial_client: 已连接的串口对象（来自手套读取器）
            hand_name: 手的名称（用于日志显示）
        """
        self.serial_client = serial_client
        self.hand_name = hand_name
        
        # 手指映射：[拇指, 食指, 中指, 无名指, 小指, 掌心]
        # self.finger_names = ["拇指", "食指", "中指", "无名指", "小指", "掌心"]
        # using english names
        self.finger_names = ["thumb", "index", "middle", "ring", "pinky", "palm"]
        
        print(f"✅ {hand_name}线性马达控制器初始化成功（使用手套串口）")
    
    def _calculate_crc16(self, data):
        """
        计算Modbus CRC16校验码
        
        参数:
            data: 要计算CRC的数据
            
        返回:
            crc: 16位CRC校验码
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def _generate_motor_command(self, finger_states):
        """
        生成线性马达控制指令
        
        参数:
            finger_states: 6个手指的状态列表，True表示振动，False表示关闭
            
        返回:
            command: 完整的Modbus指令字节数组
        """
        # Modbus指令头部
        command = [0x01, 0x10, 0x00, 0xB0, 0x00, 0x06, 0x0C]
        
        # 添加6个手指的控制数据（每个手指2字节）
        for i, state in enumerate(finger_states):
            if state:  # 振动
                command.extend([0x01, 0x36])
            else:  # 关闭 - 所有位置都使用0xFF 0xFF
                command.extend([0xFF, 0xFF])
        
        # 计算并添加CRC校验码
        crc = self._calculate_crc16(command)
        command.append(crc & 0xFF)
        command.append((crc >> 8) & 0xFF)
        
        return bytearray(command)
    
    def set_multiple_vibration(self, finger_states):
        """
        同时设置多个手指的振动状态
        
        参数:
            finger_states: 6个布尔值的列表，对应6个手指的振动状态
        """
        if len(finger_states) != 6:
            print(f"❌ {self.hand_name}finger_states长度错误，应该为6，实际为{len(finger_states)}")
            return
        
        if not self.serial_client:
            print(f"❌ {self.hand_name}线性马达串口未连接")
            return
        
        try:
            command = self._generate_motor_command(finger_states)
            
            # 通过手套串口发送马达控制命令
            self.serial_client.write(command)
            self.serial_client.flush()  # 确保数据发送
            
            # 打印振动状态
            active_fingers = [self.finger_names[i] for i, state in enumerate(finger_states) if state]
            if active_fingers:
                print(f"🔄 {self.hand_name}振动手指: {', '.join(active_fingers)}")
            
        except Exception as e:
            print(f"❌ {self.hand_name}设置多手指振动失败: {e}")
    
    def close(self):
        """
        关闭线性马达控制器
        注意：不关闭串口，因为串口属于手套读取器
        """
        print(f"🔚 {self.hand_name}线性马达控制器已关闭")

def detect_ports():
    """检测可用串口"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        usb_ports = [port for port in ports if "USB" in port.description]
        if usb_ports:
            print("🔍 检测到的USB串口:")
            for i, port in enumerate(usb_ports):
                print(f"  {i+1}. {port.device}:{port.description}")
        else:
            print("❌ 未检测到USB串口")
        return [port.device for port in ports]
    except ImportError:
        print("⚠️ 需要安装 pyserial: pip install pyserial")
        return []

def main():
    # For only testing the linear motor
    devices = detect_ports()
    # ==================== 设备端口配置 ====================
    # 左手设备配置
    LEFT_GLOVE_PORT = "/dev/ttyUSB2"        # 左手套串口（同时控制线性马达）

    # 右手设备配置  
    RIGHT_GLOVE_PORT = "/dev/ttyUSB0"       # 右手套串口（同时控制线性马达）

    input_port = input("选择左手套还是右手套？(1:左手套, 2:右手套)")
    if input_port == "1":
        port = LEFT_GLOVE_PORT
        hand_name = "左手"
    elif input_port == "2":
        port = RIGHT_GLOVE_PORT
        hand_name = "右手"
    else:
        print("输入错误")
        return

    # parser = argparse.ArgumentParser(description='线性马达控制器')
    # parser.add_argument('--port', type=str, default=RIGHT_GLOVE_PORT, help='串口端口')
    # args = parser.parse_args()

    ser = serial.Serial(
                port=port,
                baudrate=500000,
                timeout=0.02,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

    motor_controller = LinearMotorController(serial_client=ser, hand_name=hand_name)
    motor_controller.set_multiple_vibration([True, True, True, True, True, True])
    _sec = 5
    print(f"🔄 振动{_sec}秒")
    time.sleep(_sec)
    motor_controller.set_multiple_vibration([False, False, False, False, False, False])
    # motor_controller.set_multiple_vibration([False, False, False, False, False, False])
    motor_controller.close()

if __name__ == "__main__":
    main()

    # TODO: how to improve the vibration frequency?