import time
import numpy as np
import sys
import os

# 确保能导入 robot_libs 包
current_dir = os.path.dirname(os.path.abspath(__file__))
robot_libs_parent = os.path.dirname(current_dir)  # Get parent directory of robot_libs
if robot_libs_parent not in sys.path:
    sys.path.insert(0, robot_libs_parent)

try:
    from robot_libs.haptic_hand_control.ry_hand_controller import RuiyanHandController, RuiyanInstructionType
    from robot_libs.haptic_hand_control.ry_hand_interface import SerialInterface
except ImportError as e:
    print(f"Error: Could not import RyHand Serial modules: {e}")
    raise RuntimeError(f"Failed to import required RyHand modules. Please check your installation. Error: {e}")

class Hand:
    def __init__(self, port='/dev/ttyUSB1', node_id=2):
        """
        初始化 Ruiyan 机械手 (RS485/串口版本)
        兼容 Aoyi Hand 接口

        Args:
            port: 串口号 (如 '/dev/ttyUSB0')
            node_id: 设备ID (保留兼容，实际使用内部定义的 motor_ids)
        """
        self.port = port
        self.node_id = node_id
        
        # 瑞依手默认波特率通常较高，这里参考 ry_hand_485_node.py 使用 460800
        # 如果您的硬件配置不同，可能需要调整
        self.baudrate = 460800 
        
        # 定义电机ID列表 (通常瑞依手是 1-6)
        self.motor_ids = [1, 2, 3, 4, 5, 6]
        
        print(f"Initializing Ruiyan Hand (Serial/485) on {port}...")
        
        # 1. 初始化串口接口
        self.interface = SerialInterface(
            port=self.port,
            baudrate=self.baudrate,
            auto_connect=True
        )
        
        # 2. 初始化控制器
        # 使用位置、速度、电流混合控制模式 (0xAA)
        self.controller = RuiyanHandController(
            communication_interface=self.interface,
            motors_id=self.motor_ids,
            instruction=RuiyanInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT
        )
        
        # 初始化内部状态
        # 默认速度和电流限制 (参考 node 代码)
        # Reduced speed for safer, slower movement (original: 3000)
        self.controller.velocity_list = [1500] * 6
        # Motor 1 (Thumb Rotation) may need lower current to avoid overcurrent protection
        # Adjust if needed: [motor1, motor2, motor3, motor4, motor5, motor6]
        self.controller.current_list = [800, 1000, 1000, 1000, 1000, 1000]  # Reduced current for motor 1
        self.controller.position_list = [0] * 6 # 初始位置 0
        
        self.last_read_angles = [0.0] * 6

        # 尝试连接
        if not self.interface.is_connected():
             raise RuntimeError(f"Failed to open serial port {port}")
             
        print(f"Ruiyan Hand connected on {port}")

    def _init_hand(self):
        """保留接口兼容"""
        pass

    def set_angles(self, angles):
        """
        同时设置多个手指的角度

        Args:
            angles: 长度为6的列表，包含所有手指的目标角度 (0-100的百分比, 0.0-1.0)
                Aoyi 顺序: [拇指, 食指, 中指, 无名指, 小指, 拇指旋转]
        """
        if len(angles) != 6:
            raise ValueError("angles必须是长度为6的列表")
        
        try:
            # 确保角度在 0-1 范围内
            aoyi_angles = np.clip(angles, 0, 1).tolist()
            
            # 映射 Aoyi 顺序到 Ruiyan 顺序
            # Aoyi:   [Thumb(0), Index(1), Middle(2), Ring(3), Pinky(4), ThumbRot(5)]
            # Ruiyan: [ThumbRot(1), Thumb(2), Index(3), Middle(4), Ring(5), Pinky(6)]
            # 注意: Ruiyan controller list index 0 corresponds to motor_id 1
            
            ruiyan_angles = [0.0] * 6
            ruiyan_angles[0] = aoyi_angles[5] # Motor 1: ThumbRot (Index 5 in Aoyi)
            ruiyan_angles[1] = aoyi_angles[0] # Motor 2: Thumb    (Index 0 in Aoyi)
            ruiyan_angles[2] = aoyi_angles[1] # Motor 3: Index    (Index 1 in Aoyi)
            ruiyan_angles[3] = aoyi_angles[2] # Motor 4: Middle   (Index 2 in Aoyi)
            ruiyan_angles[4] = aoyi_angles[3] # Motor 5: Ring     (Index 3 in Aoyi)
            ruiyan_angles[5] = aoyi_angles[4] # Motor 6: Pinky    (Index 4 in Aoyi)
            
            # 将 0-1 映射到 0-4096 (瑞依手的位置范围)
            # Ruiyan Hand hardware: 0=open, 4096=closed
            # User input: 0=closed, 1=open
            # So we need to invert: user_input=1.0 -> hardware=0, user_input=0.0 -> hardware=4096
            position_cmds = [int((1.0 - angle) * 4096) for angle in ruiyan_angles]
            
            # 更新控制器目标
            self.controller.position_list = position_cmds
            
            # 执行控制循环 (发送指令并读取状态)
            status_list = self.controller.loop()
            
            # 更新缓存的读取角度 (如果通信成功)
            if status_list:
                self._update_read_angles(status_list)
                
        except Exception as e:
            # 记录错误但不一定抛出，以免阻塞主控
            print(f"设置角度失败: {str(e)}")
            # raise RuntimeError(f"设置角度失败: {str(e)}")

    def _update_read_angles(self, status_list):
        """从状态列表更新当前角度缓存"""
        # status_list 是 RuiyanFingerStatusMessage 对象列表
        # 需要映射回 Aoyi 顺序
        
        # 创建临时字典方便按 ID 查找
        status_map = {msg.motor_id: msg for msg in status_list if msg}
        
        # Ruiyan Motor IDs: 1:Rot, 2:Thumb, 3:Index, 4:Middle, 5:Ring, 6:Pinky
        
        def get_norm_pos(motor_id):
            if motor_id in status_map and status_map[motor_id].position is not None:
                # Hardware: 0=open, 4096=closed
                # User expects: 0=closed, 1=open
                # So invert the reading
                return 1.0 - (float(status_map[motor_id].position) / 4096.0)
            return 0.0

        # Aoyi Order: [Thumb, Index, Middle, Ring, Pinky, ThumbRot]
        self.last_read_angles[0] = get_norm_pos(2) # Thumb
        self.last_read_angles[1] = get_norm_pos(3) # Index
        self.last_read_angles[2] = get_norm_pos(4) # Middle
        self.last_read_angles[3] = get_norm_pos(5) # Ring
        self.last_read_angles[4] = get_norm_pos(6) # Pinky
        self.last_read_angles[5] = get_norm_pos(1) # ThumbRot

    def get_angles(self):
        """
        同时读取所有手指的当前角度
        返回 Aoyi 顺序: [拇指, 食指, 中指, 无名指, 小指, 拇指旋转]
        """
        # 在 set_angles 中调用 loop() 时已经更新了状态
        # 如果需要单独读取，也可以调用一次 controller.loop()
        # 这里为了保证实时性，主动调用一次 loop (但这会重发上一次的位置指令)
        
        try:
            status_list = self.controller.loop()
            if status_list:
                self._update_read_angles(status_list)
        except Exception as e:
            print(f"读取角度通信错误: {e}")
            
        return self.last_read_angles

    def close(self):
        """关闭连接"""
        if hasattr(self, 'interface') and self.interface:
            self.interface.disconnect()

    def __del__(self):
        """析构函数"""
        self.close()

if __name__ == "__main__":
    try:
        # 测试代码
        print("注意：请确保您有权限访问串口 (sudo chmod 666 /dev/ttyUSB*)")
        hand = Hand(port='/dev/ttyUSB0') # 请根据实际情况修改端口

        print("测试所有手指...")
        
        # 伸展
        print("Command: Open")
        hand.set_angles([1.0] * 6)
        time.sleep(1)
        print(f"Read Angles: {hand.get_angles()}")
        
        # 握拳
        print("Command: Close")
        hand.set_angles([0.0] * 6)
        time.sleep(1)
        print(f"Read Angles: {hand.get_angles()}")

    except Exception as e:
        print(f"错误: {str(e)}")
