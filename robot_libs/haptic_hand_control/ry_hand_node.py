import time
import numpy as np
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
import os

try:
    from Ruckig_Interpolator import SmoothJointInterpolator
except:
    sys.path.append(os.path.dirname(os.path.realpath(__file__)))
    from Ruckig_Interpolator import SmoothJointInterpolator
    
class RyHandROS2Node(Node):
    """ROS2 Node for controlling RyHand robotic hands"""
    
    def __init__(self):
        super().__init__('ry_hand_node')
        
        # 创建回调组，允许并发执行
        self.callback_group = ReentrantCallbackGroup() 
        
        # 声明参数
        self.declare_parameter('hand_type', 'left')  # 'left', 'right'
        self.declare_parameter('control_frequency', 100.0)
        
        # 获取参数
        self.hand_type = self.get_parameter('hand_type').get_parameter_value().string_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.dof = 6
        self.hand_init_angle = [0.] * self.dof
        
        # 初始化手控制器
        self.init_hands()
        self.last_cmd = None
        self.last_cmd_time = None

        self.dt = 1.0 / self.control_frequency
        
        # 设置ROS接口
        self.setup_ros_interface()
        
        # 启动定时器
        self.timer = self.create_timer(
            self.dt,
            self.timer_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self._init_interpolator()
        self.set_interpolator_input_param(self.hand_init_angle)
        time.sleep(0.5)
        self.reset_hand()
        time.sleep(2)
        
        self.get_logger().info(f"RyHand ROS2 Node initialized successfully")
        self.get_logger().info(f"Hand type: {self.hand_type}")
        self.get_logger().info(f"Control frequency: {self.control_frequency} Hz")
    

    def _init_interpolator(self):
        """Initialize smooth joint interpolator"""
        # Get control frequency from parameters
        # self.dt = 1.0 / self.control_frequency
        dt = self.dt
        
        self.interpolator = SmoothJointInterpolator(
            dof=6,
            step=dt,
            alpha=0.5
        )
        
        max_velocity = [200,]*6
        max_acceleration = [400,]*6
        max_jerk = [400,]*6
        
        self.interpolator.set_kinematic_limits(
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            max_jerk=max_jerk
        )
        
        self.get_logger().info(f"Smooth joint interpolator initialized for {self.dof} DOF with {self.control_frequency} Hz control frequency")

    def set_interpolator_input_param(self, current_position: np.ndarray):
        """
        Set the input parameters for the interpolator
        """
        current_velocity = np.zeros(self.dof)
        current_acceleration = np.zeros(self.dof)
        self.get_logger().info(f"Set interpolator input param: {current_position}")
        self.interpolator.set_input_param(
            current_position=current_position,
            current_velocity=current_velocity,
            current_acceleration=current_acceleration
        )

    def init_hands(self):
        """初始化手控制器"""
        if self.hand_type == 'left': 
            from ry_hand_left import RyLeftHand as RyHand
        elif self.hand_type == 'right':
            from ry_hand_right import RyRightHand as RyHand
        else:
             raise ValueError(f"Invalid hand_type: {self.hand_type}. Must be 'left', 'right'")
        self.hand = RyHand()
        self.hand.init()

    def setup_ros_interface(self):
        """设置ROS发布者和订阅者"""
        # 订阅角度设置命令
        self.create_subscription(
            JointState,
            f'/ry_hand/set_angles',
            self.joint_command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # 发布当前角度状态
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/ry_hand/joint_states',
            10
        )

    def joint_command_callback(self, msg):
        """处理JointState消息的回调函数"""
        if len(msg.position) != 6:
            self.get_logger().warn(f"Invalid angle array length: {len(msg.position)}, expected 6")
            return
        # self.get_logger().info(f"joint_command_callback position:{msg.position}")
        # 检查角度范围
        angles = list(msg.position)
        for i, angle in enumerate(angles):
            if not (0.0 <= angle <= 1.0):
                self.get_logger().warn(f"Angle {i} out of range [0,1]: {angle}")
                angles[i] = max(0.0, min(1.0, angle))  # 限制在有效范围内

        self.last_cmd = angles
        self.last_cmd_time = time.time()
       
    def timer_callback(self):
        """定时器回调函数，发布关节状态"""

        current_angles = np.asarray(self.hand.get_angles()).tolist()

        # 发布当前角度状态
        joint_names = ["thumb_rotation", "thumb_bend", "index", "middle", "ring", "pinky"]
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = joint_names
        joint_msg.position = np.asarray(current_angles).tolist()
        self.joint_state_pub.publish(joint_msg)

        if self.last_cmd is None:
            return
        
        if self.last_cmd_time - time.time() > 0.5:
            return

        smoothed_angle, _, _, _ = self.interpolator.update(target_pos=self.last_cmd)
        if isinstance(smoothed_angle, np.ndarray):
            smoothed_angle = smoothed_angle.tolist() 
        self.hand.set_angles(smoothed_angle)
        self.get_logger().debug(f"Set {self.hand_type} hand angles: {[f'{a:.3f}' for a in smoothed_angle]}")

    def shutdown(self):
        """节点关闭时的清理工作"""
        self.get_logger().info("Shutting down RyHand ROS2 Node...")
        
        if hasattr(self, 'timer'):
            self.timer.cancel()
        
        if hasattr(self, 'interpolator'):
            self.interpolator.close()
            
        # 设置手到安全位置
        safe_angles = [0.] * 6
        
        if self.hand:
            try:
                self.hand.set_angles(safe_angles)
                self.get_logger().info("Set left hand to safe position")
            except Exception as e:
                self.get_logger().warn(f"Error setting left hand safe position: {e}")

    def reset_hand(self):
        """重置手到init位置"""
        self.last_cmd = self.hand_init_angle
        self.last_cmd_time = time.time()
        self.hand.set_angles(self.hand_init_angle)
        self.get_logger().info("Set hand to init position")


def main(args=None):
    rclpy.init(args=args)

    # 创建节点
    node = RyHandROS2Node()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    node.get_logger().info("RyHand ROS2 Node is running...")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Received keyboard interrupt, shutting down...")
    finally:
        # 清理工作
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()