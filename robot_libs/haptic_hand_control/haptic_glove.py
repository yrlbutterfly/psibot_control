#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PSI手套控制类 - 带触觉反馈版
支持线性马达振动反馈
"""


import time
import numpy as np
import threading
import serial
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from glove_reader import SimpleGloveReader
from linear_motor import LinearMotorController

# 全局通信锁和排队系统
MODBUS_LOCK = threading.Lock()
COMMUNICATION_QUEUE = threading.Semaphore(1)  # 确保同时只有一个设备通信

# 通用配置
GLOVE_BAUDRATE = 500000                 # 手套波特率


ANGLE_INDICES = {
    "left": [0+2, 5+1, 9+1, 13+1, 17+1, 1],
    "right": [0, 5, 9, 13, 17, 18]
}
GLOVE_ANGLE_MIN = {
    "left": [-280.9, 159.2, 170.1, 159.9, 187.2, -210.8],
    "right": [146.50, 76.39, 72.81, 159.99, 115.12, 208.64]
}
GLOVE_ANGLE_MAX = {
    "left": [-216.1, 321.2, 326.3, 325.4, 337.6, -170.1],
    "right": [91.09, 36.54, 20.79, 105.36, 10.26, 235.38]
}
ANGLE_DIRECT = {
    "left": [-1, 1, 1, 1, 1, -1],
    "right": [1, 1, 1, 1, 1, 1]
}

# 关节名称
JOINT_NAMES = ["thumb_bend", "ndex_bend", "middle_bend", "ring_bend", "pinky_bend", "thumb_rotate"]



class HapticGloveController:
    """带触觉反馈的手套控制器"""
    def __init__(self, hand_name, glove_port, angle_indices, angle_min, angle_max, 
                 enable_smooth=False, current_threshold=50, current_check_interval=10):
        """
        初始化带触觉反馈的手套控制器
        
        参数:
            hand_name: 手的名称（"左手"或"右手"）
            glove_port: 手套串口
            angle_indices: 角度索引列表
            angle_min: 最小角度列表
            angle_max: 最大角度列表
            enable_smooth: 是否启用第6个手指平滑处理
            current_threshold: 电流阈值（mA）
            current_check_interval: 电流检查间隔
        """
        self.hand_name = hand_name
        self.enable_smooth = enable_smooth
        
        # 初始化手套读取器
        try:
            self.glove_reader = SimpleGloveReader(
                port=glove_port, 
                baudrate=GLOVE_BAUDRATE
            )
            print(f"✅ {hand_name}手套设备连接成功: {glove_port}")
        except Exception as e:
            print(f"❌ {hand_name}手套设备连接失败: {e}")
            raise
        
        # 控制状态
        self.running = False
        self.control_thread = None
        
        # # 第6个手指平滑处理
        # if self.enabgle_smooth:
        #     self.finer6_history = []
        #     self.finger6_history_size = 8  # 增大历史缓存
        #     self.finger6_last_value = None
        #     self.finger6_change_threshold = 0.08  # 增大变化阈值
        
        # 统计信息
        self.update_count = 0
        self.last_successful_angles = [0.] * 6  # 记录上次成功的角度
        
        print(f"🎮 {hand_name}手套控制器初始化完成")

        if self.enable_smooth:
            print(f"🔧 {hand_name}第6个手指平滑配置: 历史缓存=8, 变化阈值=0.08")
        self._print_limit_config()
    
    def _print_limit_config(self):
        """打印限位配置信息"""
        print(f"🔒 {self.hand_name}限位配置:")
        finger_names = ["拇指弯曲", "食指弯曲", "中指弯曲", "无名指弯曲", "小指弯曲", "拇指旋转"]
        for i, name in enumerate(finger_names):
            if i < 5:
                print(f"  {name}: 范围[{self.glove_angle_max[i]:.1f}° - {self.glove_angle_min[i]:.1f}°] (反向)")
            else:
                print(f"  {name}: 范围[{self.glove_angle_min[i]:.1f}° - {self.glove_angle_max[i]:.1f}°] (正向)")
        print()
    
    # def smooth_finger6(self, value):
    #     """对第6个手指进行平滑处理"""
    #     if not self.enable_smooth:
    #         return value
            
    #     if self.finger6_last_value is None:
    #         self.finger6_last_value = value
    #         self.finger6_history.append(value)
    #         return value
        
    #     change = abs(value - self.finger6_last_value)
    #     if change < self.finger6_change_threshold:
    #         return self.finger6_last_value
        
    #     self.finger6_history.append(value)
    #     if len(self.finger6_history) > self.finger6_history_size:
    #         self.finger6_history.pop(0)
        
    #     smoothed_value = sum(self.finger6_history) / len(self.finger6_history)
        
    #     # 更保守的变化率限制
    #     max_change = 0.05  # 降低最大变化量
    #     if abs(smoothed_value - self.finger6_last_value) > max_change:
    #         if smoothed_value > self.finger6_last_value:
    #             smoothed_value = self.finger6_last_value + max_change
    #         else:
    #             smoothed_value = self.finger6_last_value - max_change
        
    #     self.finger6_last_value = smoothed_value
    #     return smoothed_value
    
    def process_angles(self, glove_angles):
        """将手套角度映射到控制范围"""
        if len(glove_angles) != 6:
            raise ValueError(f"期望6个角度值，得到{len(glove_angles)}个")
        
        angles_array = np.array(glove_angles)
        mapped_angles = np.clip(
            (angles_array - self.glove_angle_min) / (self.glove_angle_max - self.glove_angle_min), 
            0, 1
        )
        
        return mapped_angles.tolist()
    
    def extract_target_angles(self, all_angles):
        """从21个角度值中提取目标角度并添加限位"""
        if len(all_angles) != 21:
            return None
        
        target_angles = [all_angles[i] for i in self.angle_indices]
        
        # 特殊处理：对前5个手指都检查是否超过400度
        finger_names = ["拇指", "食指", "中指", "无名指", "小指"]
        for i in range(5):
            if target_angles[i] > 400:
                target_angles[i] = self.glove_angle_max[i]
        
        # 添加限位处理
        limited_angles = []
        for i, angle in enumerate(target_angles):
            if i < 5:
                min_limit = self.glove_angle_max[i]
                max_limit = self.glove_angle_min[i]
                limited_angle = np.clip(angle, min_limit, max_limit)
            else:
                min_limit = self.glove_angle_min[i]
                max_limit = self.glove_angle_max[i]
                limited_angle = np.clip(angle, min_limit, max_limit)
            
            limited_angles.append(limited_angle)
        
        return limited_angles
    
    def read_action_angles(self):
        """读取手套角度数据"""
        try:
            all_angles = self.glove_reader.read_angles()
            if all_angles and len(all_angles) == 21:
                target_angles = self.extract_target_angles(all_angles)
                if target_angles:
                    mapped_angles = self.process_angles(target_angles)
                    self.update_count += 1
                    return mapped_angles
        except Exception as e:
            print(f"❌ {self.hand_name}读取手套角度错误: {e}")
        
        return None
    
    def close(self):
        """关闭控制器并清理资源"""
        # self.stop_control()
        
        # 关闭所有振动后再关闭控制器
        if self.motor_controller:
            try:
                self.motor_controller.set_multiple_vibration([False] * 6)
                print(f"🔄 {self.hand_name}退出时已关闭所有振动")
                time.sleep(0.1)  # 确保指令发送完成
            except Exception as e:
                print(f"⚠️ {self.hand_name}关闭振动时出错: {e}")
            self.motor_controller.close()
        
        try:
            self.glove_reader.close()
        except:
            pass
        print(f"🔚 {self.hand_name}手套控制器已关闭")

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

if __name__ == "__main__":
    # 测试代码
    detect_ports() 