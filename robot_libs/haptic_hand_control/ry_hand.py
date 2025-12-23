import time
import numpy as np
import RyHandLibSocketCan_6_left as ry_left_hand
import RyHandLibSocketCan_6_right as ry_right_hand
import atexit
import signal
import sys


class RyRightHand:
    def __init__(self):
        # Reset the contents of servo_bus
        ry_right_hand.ctypes.memset(ry_right_hand.ctypes.byref(ry_right_hand.servo_bus), 0,
                              ry_right_hand.ctypes.sizeof(ry_right_hand.RyCanServoBus))

        # 指定最大支持的Hook数，由用户根据实际应用情况来指定个数，建议给定值大于2 ，（一个总线至少需一个Hook，用户可以不给定，库内部最少会申请一个Hook）
        ry_right_hand.servo_bus.usHookNum = 5
        ry_right_hand.servo_bus.usListenNum = 30 + 1

        # 初始化库, 内部会使用 malloc 需保证有足够的栈空间，请查看栈空间设置
        ry_right_hand.lib.RyCanServoBusInit(ry_right_hand.ctypes.byref(ry_right_hand.servo_bus),
                                      ry_right_hand.c_bus_write,
                                      ry_right_hand.ctypes.byref(ry_right_hand.ticks_var),
                                      1000)
        
        # Register cleanup function to be called on exit
        atexit.register(self.cleanup)
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources and properly shut down the CAN bus"""
        try:
            if hasattr(ry_right_hand, 'bus') and ry_right_hand.bus is not None:
                ry_right_hand.bus.shutdown()
                print("CAN bus properly shut down")
        except Exception as e:
            print(f"Error during cleanup: {e}")

    def check_can_bus_status(self):
        """Check if the CAN bus is properly configured and running"""
        try:
            import subprocess
            # Check if can0 interface exists and is up
            result = subprocess.run(['ip', 'link', 'show', 'can0'], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print("CAN bus 'can0' is not configured. Please run the following commands:")
                print("sudo modprobe can")
                print("sudo modprobe can-raw")
                print("sudo modprobe can-gw")
                print("sudo ip link add can0 type can bitrate 1000000")
                print("sudo ip link set can0 up")
                return False
            
            # Check if interface is UP
            if "UP" not in result.stdout:
                print("CAN bus 'can0' is not UP. Please run:")
                print("sudo ip link set can0 up")
                return False
                
            print("CAN bus 'can0' is properly configured and running")
            return True
            
        except Exception as e:
            print(f"Error checking CAN bus status: {e}")
            return False

    def set_angles(self, angles):
        """
        同时设置多个手指的角度

        Args:
                [拇指旋转, 拇指弯曲, 食指, 中指, 无名指, 小指]
        """
        # Check CAN bus status before sending commands
        if not self.check_can_bus_status():
            print("Warning: CAN bus not ready, commands may fail")
            
        cmds = [0] * 6
        cmds[0] = self.map_value_to_angle_deg(angles[0], 0, 110)
        cmds[1] = self.map_value_to_angle_deg(angles[1], 0, 30)
        cmds[2] = self.map_value_to_angle_deg(angles[2], 0, 84)
        cmds[3] = self.map_value_to_angle_deg(angles[3], 0, 90)
        cmds[4] = self.map_value_to_angle_deg(angles[4], 0, 90)
        cmds[5] = self.map_value_to_angle_deg(angles[5], 0, 88)
        ry_right_hand.update_motor_positions(cmds)

    def get_angles(self):
        motor_angles = ry_right_hand.get_motor_angles(0)
        print(f"motor_angles: {motor_angles}")
        # states = [0] * 6
        # states[0] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(0), 0, 110)
        # states[1] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(1), 0, 30)
        # states[2] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(2), 0, 84)
        # states[3] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(3), 0, 90)
        # states[4] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(4), 0, 90)
        # states[5] = self.map_angle_deg_to_value(ry_right_hand.get_motor_angles(5), 0, 88)
        # return states

    def map_angle_deg_to_value(self, angle, min_deg, max_deg):
        """将角度归一化到0~1之间"""
        angle = np.clip(angle, min_deg, max_deg)
        return (angle - min_deg) / (max_deg -
                                    min_deg) if max_deg > min_deg else 0.0

    def map_value_to_angle_deg(self, value, min_deg, max_deg):
        """将0~1的值反归一化为角度"""
        value = np.clip(value, 0, 1)
        return value * (max_deg - min_deg) + min_deg


class RyLeftHand:
    def __init__(self):
        # Reset the contents of servo_bus
        ry_left_hand.ctypes.memset(ry_left_hand.ctypes.byref(ry_left_hand.servo_bus), 0,
                              ry_left_hand.ctypes.sizeof(ry_left_hand.RyCanServoBus))

        # 指定最大支持的Hook数，由用户根据实际应用情况来指定个数，建议给定值大于2 ，（一个总线至少需一个Hook，用户可以不给定，库内部最少会申请一个Hook）
        ry_left_hand.servo_bus.usHookNum = 5
        ry_left_hand.servo_bus.usListenNum = 30 + 1

        # 初始化库, 内部会使用 malloc 需保证有足够的栈空间，请查看栈空间设置
        ry_left_hand.lib.RyCanServoBusInit(ry_left_hand.ctypes.byref(ry_left_hand.servo_bus),
                                      ry_left_hand.c_bus_write,
                                      ry_left_hand.ctypes.byref(ry_left_hand.ticks_var),
                                      1000)
        
        # Register cleanup function to be called on exit
        atexit.register(self.cleanup)
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources and properly shut down the CAN bus"""
        try:
            if hasattr(ry_left_hand, 'bus') and ry_left_hand.bus is not None:
                ry_left_hand.bus.shutdown()
                print("CAN bus properly shut down")
        except Exception as e:
            print(f"Error during cleanup: {e}")

    def check_can_bus_status(self):
        """Check if the CAN bus is properly configured and running"""
        try:
            import subprocess
            # Check if can0 interface exists and is up
            result = subprocess.run(['ip', 'link', 'show', 'can0'], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print("CAN bus 'can0' is not configured. Please run the following commands:")
                print("sudo modprobe can")
                print("sudo modprobe can-raw")
                print("sudo modprobe can-gw")
                print("sudo ip link add can0 type can bitrate 1000000")
                print("sudo ip link set can0 up")
                return False
            
            # Check if interface is UP
            if "UP" not in result.stdout:
                print("CAN bus 'can0' is not UP. Please run:")
                print("sudo ip link set can0 up")
                return False
                
            print("CAN bus 'can0' is properly configured and running")
            return True
            
        except Exception as e:
            print(f"Error checking CAN bus status: {e}")
            return False

    def set_angles(self, angles):
        """
        同时设置多个手指的角度

        Args:192.168.100
                [拇指旋转, 拇指弯曲, 食指, 中指, 无名指, 小指]
        """
        # Check CAN bus status before sending commands
        if not self.check_can_bus_status():
            print("Warning: CAN bus not ready, commands may fail")
            
        cmds = [0] * 6
        cmds[0] = self.map_value_to_angle_deg(angles[0], 0, 110)
        cmds[1] = self.map_value_to_angle_deg(angles[1], 0, 30)
        cmds[2] = self.map_value_to_angle_deg(angles[2], 0, 84)
        cmds[3] = self.map_value_to_angle_deg(angles[3], 0, 90)
        cmds[4] = self.map_value_to_angle_deg(angles[4], 0, 90)
        cmds[5] = self.map_value_to_angle_deg(angles[5], 0, 88)
        ry_left_hand.update_motor_positions(cmds)

        

    def get_angles(self):
        states = [0] * 6
        states[0] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(0), 0, 110)
        states[1] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(1), 0, 30)
        states[2] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(2), 0, 84)
        states[3] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(3), 0, 90)
        states[4] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(4), 0, 90)
        states[5] = self.map_angle_deg_to_value(ry_left_hand.get_motor_angles(5), 0, 88)
        return states

    def map_angle_deg_to_value(self, angle, min_deg, max_deg):
        """将角度归一化到0~1之间"""
        angle = np.clip(angle, min_deg, max_deg)
        return (angle - min_deg) / (max_deg -
                                    min_deg) if max_deg > min_deg else 0.0

    def map_value_to_angle_deg(self, value, min_deg, max_deg):
        """将0~1的值反归一化为角度"""
        value = np.clip(value, 0, 1)
        return value * (max_deg - min_deg) + min_deg


if __name__ == "__main__":
    right_hand = RyRightHand()
    left_hand = RyLeftHand()

    angles = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    # angles = [0.]*6
    
    # Check CAN bus status first
    if right_hand.check_can_bus_status():
        right_hand.set_angles(angles)
    else:
        print("Cannot proceed without proper CAN bus configuration")

    angles = right_hand.get_angles()
    print(f"angles: {angles}")