#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
import rclpy.time
from sensor_msgs.msg import JointState
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from aoyi_hand_module_modbus_non_block import Hand

class AoyiHandNode(Node):
    def __init__(self):
        super().__init__('aoyi_hand_node')
        
        # 创建回调组，允许并发执行  
        self.callback_group = ReentrantCallbackGroup()
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('node_id', 2)
        self.declare_parameter('publish_rate', 100.0)
        
        # 获取参数
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 线程锁，用于保护共享数据
        self.lock = threading.Lock()
        
        # 目标角度，初始化为中间位置
        self.target_angles = [0.5] * 6
        self.current_angles = [0.5] * 6
        self.finger_currents = [0.0] * 6
        
        # 连接状态
        self.hand_connected = False
        self.hand = None
        
        # 初始化aoyi hand
        self.init_hand()
        
        # 创建ROS发布者和订阅者
        self.setup_ros_interface()
        
        # 启动定时器（100Hz发布频率）
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.timer_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info("Aoyi Hand Node initialized successfully")
        self.get_logger().info(f"Publishing rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Port: {self.port}, Node ID: {self.node_id}")

        self.timer_stmp = None
        self.cb_stmp = None

    def init_hand(self):
        """初始化aoyi hand连接"""
        try:
            self.get_logger().info(f"Connecting to Aoyi Hand on port {self.port}...")
            self.hand = Hand(port=self.port, node_id=self.node_id)
            self.hand_connected = True
            self.get_logger().info("Aoyi Hand connected successfully")
            
            # 读取初始角度和电流
            with self.lock:
                try:
                    # 读取初始角度和电流, 从硬件读取, 会阻塞
                    initial_angles = self.hand.get_angles()
                    initial_currents = self.hand.get_finger_currents()
                    self.current_angles = initial_angles
                    self.target_angles = initial_angles
                    self.finger_currents = initial_currents
                        
                except Exception as e:
                    self.get_logger().warn(f"Failed to read initial values: {e}")
                    # 使用默认值
                    self.current_angles = [0.5] * 6
                    self.target_angles = [0.5] * 6
                    self.finger_currents = [0.0] * 6
                    
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Aoyi Hand: {e}")
            self.hand_connected = False

    def setup_ros_interface(self):
        """设置ROS发布者和订阅者"""
        # 订阅角度设置命令
        self.angle_sub = self.create_subscription(
            JointState,
            '/aoyi_hand/set_angles',
            self.angle_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # 发布手指电流
        self.current_pub = self.create_publisher(
            JointState,
            '/aoyi_hand/finger_currents',
            10
        )
        
        # 发布当前角度
        self.angle_pub = self.create_publisher(
            JointState,
            '/aoyi_hand/finger_angles',
            10
        )

    def angle_callback(self, msg):
        """处理角度设置命令的回调函数"""
        if not self.hand_connected:
            self.get_logger().warn("Hand not connected, ignoring angle command")
            return
            
        if len(msg.data) != 6:
            self.get_logger().warn(f"Invalid angle array length: {len(msg.data)}, expected 6")
            return
        
        # 检查角度范围
        angles = list(msg.data)
        for i, angle in enumerate(angles):
            if not (0.0 <= angle <= 1.0):
                self.get_logger().warn(f"Angle {i} out of range [0,1]: {angle}")
                angles[i] = max(0.0, min(1.0, angle))  # 限制在有效范围内
        
        # 更新目标角度（线程安全）
        with self.lock:
            self.target_angles = angles
            
        # 异步设置角度，避免阻塞订阅者
        threading.Thread(target=self.set_angles_async, args=(angles,), daemon=True).start()

    def set_angles_async(self, angles):
        """异步设置角度，避免阻塞主线程"""
        try:
            if self.hand_connected and self.hand:
                self.hand.set_angles_non_block(angles)
                if self.cb_stmp is None:
                    self.cb_stmp = time.time()
                else:
                    if time.time() - self.cb_stmp > 0.1*1.5:
                        self.get_logger().warn(f"Time interval for set_angles_async is too long: {time.time() - self.cb_stmp}")
                    self.cb_stmp = time.time()
                self.get_logger().debug(f"Set angles: {[f'{a:.3f}' for a in angles]}")
        except Exception as e:
            self.get_logger().warn(f"Failed to set angles: {e}")

    def timer_callback(self):
        """定时器回调函数，以100Hz频率发布数据"""
        if not self.hand_connected:
            return
            
        try:          
            t0 = time.time() 
            current_angles = self.hand.get_angles_non_block()
            t1 = time.time()
            # print(f"self.hand.get_angles_non_block():{t1-t0:.6f}")
            finger_currents = self.hand.get_finger_currents_non_block()    
            t2 = time.time()
            # print(f"self.hand.get_angles_non_block():{t2-t1:.6f}")
            # 发布数据
            self.publish_data(current_angles, finger_currents)

            if self.timer_stmp is None:
                self.timer_stmp = time.time()
            else:
                if time.time() - self.timer_stmp > 0.1:
                    self.get_logger().warn(f"Time interval for timer_callback is too long: {time.time() - self.timer_stmp}")
                self.timer_stmp = time.time()
                
        except Exception as e:
            self.get_logger().warn(f"Timer callback error: {e}")

    def publish_data(self, current_angles, finger_currents):
        """发布所有数据

        # JointState
        std_msgs/Header header
        float64[] data

        finger_names = {
			0: "拇指",
			1: "食指",
			2: "中指",
			3: "无名指",
			4: "小指",
			5: "拇指旋转"
		}
        """     
        _stamp = self.get_clock().now().to_msg()

        # 发布手指电流
        current_msg = JointState()
        current_msg.name = ["thumb", "index", "middle", "ring", "pinky", "thumb_rotation"]
        current_msg.position = finger_currents
        current_msg.header.stamp = _stamp
        self.current_pub.publish(current_msg)
        
        # 发布当前角度
        angle_msg = JointState()
        angle_msg.name = ["thumb", "index", "middle", "ring", "pinky", "thumb_rotation"]
        angle_msg.position = current_angles
        angle_msg.header.stamp = _stamp
        self.angle_pub.publish(angle_msg)
            
    def shutdown(self):
        """节点关闭时的清理工作"""
        self.get_logger().info("Shutting down Aoyi Hand Node...")
        
        if hasattr(self, 'timer'):
            self.timer.cancel()
            
        if self.hand_connected and self.hand:
            try:
                # 设置手指到安全位置（中间位置）
                safe_angles = [0.5] * 6
                self.hand.set_angles(safe_angles)
                self.get_logger().info("Set hand to safe position")
                
                # 关闭连接
                self.hand.close()
                self.get_logger().info("Hand connection closed")
            except Exception as e:
                self.get_logger().warn(f"Error during shutdown: {e}")

def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = AoyiHandNode()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    

    node.get_logger().info("Aoyi Hand Node is running...")
    executor.spin()

    # 清理工作
    node.shutdown()
    node.destroy_node()
            

if __name__ == '__main__':
    main() 

