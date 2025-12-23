#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple client for testing RyHand ROS2 Node
Provides interactive commands and automatic test sequences
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import numpy as np
import threading
import sys


class RyHandClient(Node):
    """Simple client for testing RyHand ROS2 Node"""
    
    def __init__(self):
        super().__init__('ry_hand_client')

        self.hand_name = "right"
        
        # 创建发布者
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'/ry_hand/{self.hand_name}/set_angles',
            10
        )
        
        # 创建订阅者来监听关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/ry_hand/{self.hand_name}/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 关节名称
        self.joint_names = ["thumb_rotation", "thumb_bend", "index", "middle", "ring", "pinky"]
        
        # 当前关节状态
        self.current_joint_states = [0.0] * 6
        
        # 测试序列
        self.test_sequences = {
            'demo': [
                [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 中间位置
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 完全伸直
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # 完全弯曲
                [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # 不同角度
                [0.8, 0.7, 0.6, 0.5, 0.4, 0.3],  # 反向角度
                [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 回到中间
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 

                [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 中间位置
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 完全伸直
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # 完全弯曲
                [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # 不同角度
                [0.8, 0.7, 0.6, 0.5, 0.4, 0.3],  # 反向角度
                [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 回到中间
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
            ],
            'wave': [
                [0.5, 0.5, 0.0, 0.0, 0.0, 0.5],  # 只伸食指和中指
                [0.5, 0.5, 1.0, 1.0, 0.0, 0.5],  # 弯曲食指和中指
                [0.5, 0.5, 0.0, 0.0, 0.0, 0.5],  # 再伸直
                [0.5, 0.5, 1.0, 1.0, 0.0, 0.5],  # 再弯曲
            ],
            'fist': [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 完全伸直
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # 握拳
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 再伸直
                [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # 再握拳
            ]
        }
        
        self.get_logger().info("RyHand Client started")
        self.get_logger().info("Available commands:")
        self.get_logger().info("  demo - Run demo sequence")
        self.get_logger().info("  wave - Run wave sequence")
        self.get_logger().info("  fist - Run fist sequence")
        self.get_logger().info("  manual <joint> <angle> - Set specific joint")
        self.get_logger().info("  status - Show current status")
        self.get_logger().info("  quit - Exit client")

    def joint_state_callback(self, msg):
        """处理关节状态回调"""
        self.get_logger().info(f"joint_state_callback position:{msg.position}")
        if len(msg.position) == 6:
            self.current_joint_states = list(msg.position)
            self.get_logger().info(f"joint_state_callback_position:{self.current_joint_states}")

    def send_angles(self, angles):
        """发送角度命令"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = angles
        
        self.joint_state_pub.publish(joint_msg)
        self.get_logger().info(f"Sent angles: {[f'{a:.3f}' for a in angles]}")

    def run_sequence(self, sequence_name):
        """运行测试序列"""
        if sequence_name not in self.test_sequences:
            self.get_logger().error(f"Unknown sequence: {sequence_name}")
            return
            
        sequence = self.test_sequences[sequence_name]
        self.get_logger().info(f"Running {sequence_name} sequence...")
        
        for i, angles in enumerate(sequence):
            self.send_angles(angles)
            time.sleep(1)  # 等待2秒
            
        self.get_logger().info(f"{sequence_name} sequence completed")

    def set_joint(self, joint_index, angle):
        """设置单个关节角度"""
        if joint_index < 0 or joint_index >= 6:
            self.get_logger().error(f"Invalid joint index: {joint_index}. Must be 0-5")
            return
            
        if angle < 0.0 or angle > 1.0:
            self.get_logger().error(f"Invalid angle: {angle}. Must be 0.0-1.0")
            return
            
        angles = self.current_joint_states.copy()
        angles[joint_index] = angle
        self.send_angles(angles)

    def show_status(self):
        """显示当前状态"""
        joint_names = ["拇指旋转", "拇指弯曲", "食指", "中指", "无名指", "小指"]
        self.get_logger().info("Current joint states:")
        for i, (name, angle) in enumerate(zip(joint_names, self.current_joint_states)):
            self.get_logger().info(f"  {name}: {angle:.3f}")

    def interactive_mode(self):
        """交互模式"""
        while rclpy.ok():
            try:
                command = input("RyHand> ").strip().lower()
                
                if command == 'quit' or command == 'exit':
                    break
                elif command == 'demo':
                    self.run_sequence('demo')
                elif command == 'wave':
                    self.run_sequence('wave')
                elif command == 'fist':
                    self.run_sequence('fist')
                elif command == 'status':
                    self.show_status()
                elif command.startswith('manual '):
                    parts = command.split()
                    if len(parts) == 3:
                        try:
                            joint = int(parts[1])
                            angle = float(parts[2])
                            self.set_joint(joint, angle)
                        except ValueError:
                            self.get_logger().error("Invalid manual command. Use: manual <joint> <angle>")
                    else:
                        self.get_logger().error("Invalid manual command. Use: manual <joint> <angle>")
                elif command == 'help':
                    self.get_logger().info("Available commands:")
                    self.get_logger().info("  demo - Run demo sequence")
                    self.get_logger().info("  wave - Run wave sequence")
                    self.get_logger().info("  fist - Run fist sequence")
                    self.get_logger().info("  manual <joint> <angle> - Set specific joint (0-5, 0.0-1.0)")
                    self.get_logger().info("  status - Show current status")
                    self.get_logger().info("  quit - Exit client")
                else:
                    self.get_logger().info("Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break

    def auto_test_mode(self):
        """自动测试模式"""
        self.get_logger().info("Starting auto test mode...")
        
        # 运行所有测试序列
        for sequence_name in self.test_sequences.keys():
            self.run_sequence(sequence_name)
            time.sleep(1.0)  # 序列间等待1秒
            
        self.get_logger().info("Auto test completed")


def main(args=None):
    rclpy.init(args=args)
    
    # 创建客户端节点
    client = RyHandClient()
    
    # 检查命令行参数
    if len(sys.argv) > 1 and sys.argv[1] == '--auto':
        # 自动测试模式
        client.auto_test_mode()
    else:
        # 交互模式
        client.interactive_mode()
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 