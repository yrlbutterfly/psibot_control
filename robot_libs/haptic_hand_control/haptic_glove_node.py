#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PSI手套控制器ROS2节点
读取手套数据并发布到机械手控制话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import numpy as np
import os
import sys
from rclpy.duration import Duration
import matplotlib.pyplot as plt
from collections import deque
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from haptic_glove import detect_ports
from glove_reader import SimpleGloveReader
from rclpy.callback_groups import ReentrantCallbackGroup
from kalman_filter_angles import KalmanFilterAngles

class LowPassFilter:
    """低通滤波器类"""
    def __init__(self, delta=0.1, num_joints=6):
        """
        初始化低通滤波器
        alpha: 滤波系数 (0-1)，越小滤波越强
        num_joints: 关节数量
        """
        self.delta = delta
        self.num_joints = num_joints
        self.filtered_values = None
    
    def filter(self, values):
        """
        对输入值进行低通滤波
        values: 输入的角度数组
        """
        if self.filtered_values is None:
            self.filtered_values = np.array(values)
        else:
            _delta = np.array(values) - self.filtered_values
            # cutoff the delta that is larger than delta
            # TODO：频率低时需检查这里是否过度clip导致延迟高
            # 这个是用在手套的读数上，这个倒是没必要很低
            _delta = np.clip(_delta, -self.delta, self.delta)
            self.filtered_values = self.filtered_values + _delta
        return self.filtered_values.tolist()
    
    def reset(self):
        """重置滤波器状态"""
        self.filtered_values = None



ANGLE_INDICES = {
    "left": [4, 2, 7, 11, 15, 19] ,
    "right": [4, 2, 7, 11, 15, 19] 
}

# 关节名称
JOINT_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend"]

GLOVE_ANGLE_MIN = {
    "left":  [305, 260, 235, 235, 235, 270],
    "right": [50 , 257, 240, 245, 240, 260]
}
GLOVE_ANGLE_MAX = {
    "left": [295, 210, 130, 125, 125, 150],
    "right": [66 , 220, 155, 130, 130, 150]
}

ANGLE_DIRECT = {
    "left": [1, 1, 1, 1, 1, 1],
    "right": [1, 1, 1, 1, 1, 1],
}

CLIP_MIN = {    
    "left": [0, 0, 0, 0, 0, 0],
    "right": [0, 0, 0, 0, 0, 0]
}
CLIP_MAX = {
    "left": [0.6, 1, 1, 1, 1, 1],
    "right": [0.6, 1, 1, 1, 1, 1]
}


def precise_wait_until(time_end, dt=0.001):
    """精确等待直到指定时间"""
    while True:
        if time.time() >= time_end:
            break
        time.sleep(dt)

class HapticGloveROS2Node(Node):
    """PSI手套控制器ROS2节点"""
    
    def __init__(self):
        super().__init__('haptic_glove_controller')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('glove_port', "/dev/ttyUSB0"),
                ('hand_name', "left"),
                ('publish_rate', 10.0),  # 发布频率 (Hz)
                ('enable_linear_motor', False),
                ('enable_smooth', True),
                ('glove_read_rate', 30),
                ('low_pass_delta', 0.1),  # 低通滤波系数
            ]
        )
        
        # 获取参数
        self.glove_port = self.get_parameter('glove_port').value
        self.hand_name = self.get_parameter('hand_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_linear_motor = self.get_parameter('enable_linear_motor').value
        self.enable_smooth = self.get_parameter('enable_smooth').value
        self.glove_read_rate = self.get_parameter('glove_read_rate').value
        self.data_expired_duration = 0.5 # 数据过期时间，单位秒
        self.smooth_window_sec = 0.1
        self.low_pass_delta = self.get_parameter('low_pass_delta').value
        self.kalman_filter = KalmanFilterAngles(num_joints=6, dt=1.0/self.glove_read_rate)
        self.low_pass_filter = LowPassFilter(delta=self.low_pass_delta, num_joints=6)

        assert self.hand_name in ["left", "right"], "💡 hand_name must be left or right"
        
        # Initialize matplotlib visualization
        # self.init_matplotlib_visualization()
        
        # 初始化手套控制器
        self.glove_controller = None
        self.controllers_initialized = False
        
        # 角度数据缓存
        self.action_angles = None
        self.action_angles_stamp = None
        self.action_angles_queue = deque(maxlen=int(self.smooth_window_sec*self.glove_read_rate))
        
        # 创建发布者
        self.action_angles_pub = self.create_publisher(
            JointState, 
            '/haptic_glove/action_angles',  # /haptic_glove/action_angles 
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # 创建定时器
        self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 初始化手套控制器
        self.init_glove_controllers()
        
        self.get_logger().info(f"🎮 PSI手套控制器ROS节点已启动")
        self.get_logger().info(f"📊 发布频率: {self.publish_rate} Hz")
        self.get_logger().info(f"🔧 线性马达: {'启用' if self.enable_linear_motor else '禁用'}")
        self.get_logger().info(f"🔧 平滑处理: {'启用' if self.enable_smooth else '禁用'}")
        self.get_logger().info(f"🔧 低通滤波系数: {self.low_pass_delta}")
    
    def init_matplotlib_visualization(self):
        """Initialize matplotlib for real-time visualization"""
        # Set up matplotlib for interactive mode
        plt.ion()
        
        # Create figure and subplots
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 10))
        self.fig.suptitle(f'{self.hand_name.capitalize()} Hand Action Angles Visualization', fontsize=16)
        
        # Flatten axes for easier indexing
        self.axes = self.axes.flatten()
        
        # Initialize data storage for plotting
        self.max_points = 100  # Number of points to keep in history
        self.time_data = deque(maxlen=self.max_points)
        self.angle_data = [deque(maxlen=self.max_points) for _ in range(6)]
        
        # Initialize plots
        self.lines = []
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
        
        for i, (joint_name, color) in enumerate(zip(JOINT_NAMES, colors)):
            line, = self.axes[i].plot([], [], color=color, linewidth=2, label=joint_name)
            self.lines.append(line)
            self.axes[i].set_title(joint_name, fontsize=12)
            self.axes[i].set_ylim(0, 1)
            self.axes[i].set_xlim(0, 10)  # 10 seconds window
            self.axes[i].grid(True, alpha=0.3)
            self.axes[i].set_ylabel('Normalized Angle')
            if i >= 3:  # Bottom row
                self.axes[i].set_xlabel('Time (s)')
        
        plt.tight_layout()
        plt.show(block=False)
        
        self.get_logger().info("📊 Matplotlib visualization initialized")
    
    def init_glove_controllers(self):
        """初始化手套控制器"""
        self.get_logger().info("🔌 初始化手套控制器...")
        
        try:
            # 初始化左手控制器
            self.get_logger().info(f"📱 连接{self.hand_name}: {self.glove_port}")
            self.glove_controller = SimpleGloveReader(
                port=self.glove_port,
                baudrate=500000,
                timeout=0.02
            )
            
            # 启动数据读取线程
            self.start_glove_reading()
            
            self.controllers_initialized = True
            self.get_logger().info("✅ 手套控制器初始化完成")
            
        except Exception as e:
            self.get_logger().error(f"❌ 手套控制器初始化失败: {e}")
            self.get_logger().info("💡 提示: 检查手套连接和串口权限")
    
    def start_glove_reading(self):
        """启动手套数据读取线程"""
        # 手套数据读取线程
        reading_hz = self.glove_read_rate
        dt = 1.0 / reading_hz
        def glove_reader():
            while rclpy.ok():
                t0 = time.time()
                if self.glove_controller:
                    _action_angles = self.glove_controller.read_angles()
                    if _action_angles:
                        self.action_angles = self.process_action_angles(_action_angles)
                        self.action_angles_stamp = self.get_clock().now()
                        self.get_logger().info(f"✅ 成功读取{self.hand_name}手套角度: {self.action_angles}, glove_reader正常", once=True)
                    else:
                        self.get_logger().warn(f"⚠️ 本次{self.hand_name}手套读取为空")
                        pass
                precise_wait_until(t0 + dt)
        
        # 启动线程
        if self.glove_controller:
            glove_thread = threading.Thread(target=glove_reader, daemon=True)
            glove_thread.start()
            self.get_logger().info(f"✅ {self.hand_name}数据读取线程已启动")
    
    def process_action_angles(self, action_angles):
        """处理手套角度数据"""
        action_angles = np.array(action_angles)[ANGLE_INDICES[self.hand_name]].tolist()
        
        # 检查并处理大于360的读数，设置为0
        for i, angle in enumerate(action_angles):
            if abs(angle) > 600:
                self.get_logger().warn(f"⚠️ 检测到异常角度值 {angle}，设置为0")
                action_angles[i] = 0.0
        
        # 使用正确的方向
        action_angles = [angle * ANGLE_DIRECT[self.hand_name][i] for i, angle in enumerate(action_angles)]
        # 将角度归一化到0-1
        action_angles = [(angle - GLOVE_ANGLE_MIN[self.hand_name][i]) / (GLOVE_ANGLE_MAX[self.hand_name][i] - GLOVE_ANGLE_MIN[self.hand_name][i]) for i, angle in enumerate(action_angles)]
        # clip to 0 - 1
        action_angles = np.clip(action_angles, 0.0, 1.0).tolist()

        action_angles = np.clip(action_angles, CLIP_MIN[self.hand_name], CLIP_MAX[self.hand_name]).tolist()

        if self.enable_smooth:
            # 先进行低通滤波
            action_angles = self.low_pass_filter.filter(action_angles)

            action_angles = self.action_angles_queue.append(action_angles)
            action_angles = np.mean(self.action_angles_queue, axis=0)
            # 再进行卡尔曼滤波
            # action_angles = self.kalman_filter.filter_angles(action_angles)
        # 限制在0-1之间
        action_angles = np.clip(action_angles, 0.0, 1.0).tolist()
        return action_angles

    def visualize_action_angles(self):
        """Visualize action_angles using matplotlib"""
        if self.action_angles is None:
            return
            
        current_time = time.time()
        
        # Add current time and angles to data storage
        self.time_data.append(current_time)
        for i, angle in enumerate(self.action_angles):
            self.angle_data[i].append(angle)
        
        # Update plots
        if len(self.time_data) > 1:
            # Convert time to relative time (seconds from start)
            start_time = self.time_data[0]
            relative_times = [(t - start_time) for t in self.time_data]
            
            # Update each line
            for i, line in enumerate(self.lines):
                line.set_data(relative_times, list(self.angle_data[i]))
                
                # Auto-scale x-axis to show last 10 seconds
                if len(relative_times) > 0:
                    max_time = max(relative_times)
                    min_time = max(0, max_time - 10)  # Show last 10 seconds
                    self.axes[i].set_xlim(min_time, max_time)
            
            # Refresh the plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        

    def timer_callback(self):
        """定时器回调函数 - 发布角度数据"""
        if not self.controllers_initialized:
            return
        
        current_time = self.get_clock().now()
        
        # 发布左手角度
        if self.glove_controller and self.action_angles is not None:

            if current_time - self.action_angles_stamp > Duration(seconds=self.data_expired_duration):
                self.get_logger().warn(f"❌ {self.hand_name}手套角度发布时间超过{self.data_expired_duration}秒， 数据过期")
                return

            # 发布Float64MultiArray消息
            glove_msg = JointState()
            glove_msg.header.stamp = self.action_angles_stamp.to_msg()
            glove_msg.name = JOINT_NAMES
            if isinstance(self.action_angles, np.ndarray):
                glove_msg.position = self.action_angles.tolist()
            else:
                glove_msg.position = self.action_angles
            self.action_angles_pub.publish(glove_msg)
            
            # Visualize action_angles using Rerun
            # self.visualize_action_angles()
            
            self.get_logger().info(f"✅ 成功发布{self.hand_name}手套消息, 角度: {self.action_angles}, timer_callback正常", once=True)
                
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info("🛑 正在关闭手套控制器...")
        self.kalman_filter.reset()
        self.low_pass_filter.reset()
        
        # 关闭手套控制器
        if self.glove_controller:
            try:
                self.glove_controller.close()
                self.get_logger().info(f"✅ {self.hand_name}手套控制器已关闭")
            except Exception as e:
                self.get_logger().warn(f"❌ {self.hand_name}手套控制器关闭错误: {e}")
        
        # 关闭matplotlib窗口
        try:
            plt.close(self.fig)
            self.get_logger().info("✅ Matplotlib窗口已关闭")
        except Exception as e:
            self.get_logger().warn(f"❌ Matplotlib窗口关闭错误: {e}")
        
        super().destroy_node()

def main(args=None):
    detect_ports()
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = HapticGloveROS2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 节点错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 

# TODO: 按照100hz的频率读取传感器数据， 进行平滑, 但是按照制定hz进行发布