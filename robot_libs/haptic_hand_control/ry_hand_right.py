import time
import numpy as np
import atexit
import signal
import sys

API_VERISON = 1.0
if API_VERISON == 1.0:
    from RyHandLibSocketCan_6_right import *
elif API_VERISON == 2.0:
    from RyHandLibSocketCan_6x_right import *
else:
    raise ValueError(f"Invalid API_VERISON: {API_VERISON}. Must be 1.0 or 2.0")

class RyRightHand:
    def __init__(self):
        self.states_cache = [0] * 6
        self.last_cmd = [0] * 6

    def init(self):
        preticks = 0

        servos_info = (Servos_t * 15)()

        timer_thread = TimerThread(0.001, update_ticks)
        timer_thread.start()

        # Create and start the bus read thread
        BusRead_thread = ReadThread(None, bus_read_callback)
        BusRead_thread.start()

        # Reset the contents of servo_bus
        ctypes.memset(ctypes.byref(servo_bus), 0, ctypes.sizeof(RyCanServoBus))

        # 指定最大支持的Hook数，由用户根据实际应用情况来指定个数，建议给定值大于2 ，（一个总线至少需一个Hook，用户可以不给定，库内部最少会申请一个Hook）
        servo_bus.usHookNum = 5;                                                 	 		 
        servo_bus.usListenNum = 30+1;                                  
        
        # 初始化库, 内部会使用 malloc 需保证有足够的栈空间，请查看栈空间设置
        response_code = lib.RyCanServoBusInit(ctypes.byref(servo_bus), c_bus_write, ctypes.byref(ticks_var) , 1000) 

        # 添加监听，这里添加了30个监听，分别监听15个舵机的反馈数据，监听对象是 0xa0 和 0xaa 指令数据
        if response_code == 0:
            stuListenMsg = (CanMsg * 30)()
            for i in range(15):
                stuListenMsg[i].ulId = SERVO_BACK_ID(i + 1)
                stuListenMsg[i].pucDat[0] = 0xA0
                ret = lib.AddListen(ctypes.byref(servo_bus), ctypes.byref(stuListenMsg[i]), c_callback1 )
            for i in range(15,30):
                stuListenMsg[i].ulId = SERVO_BACK_ID(i-15 + 1)
                stuListenMsg[i].pucDat[0] = 0xAA
                ret = lib.AddListen(ctypes.byref(servo_bus), ctypes.byref(stuListenMsg[i]), c_callback1 )

    def set_angles(self, angles, limit_thumb_rot=True):
        """
        同时设置多个手指的角度

        Args:
                [拇指旋转, 拇指弯曲, 食指, 中指, 无名指, 小指] 0-1 归一化
        """
        cmds = [0] * 6
        cmds[0] = self.map_value_to_angle_deg(angles[0], 0, 110)
        cmds[1] = self.map_value_to_angle_deg(angles[1], 0, 30)
        cmds[2] = self.map_value_to_angle_deg(angles[2], 0, 84)
        cmds[3] = self.map_value_to_angle_deg(angles[3], 0, 90)
        cmds[4] = self.map_value_to_angle_deg(angles[4], 0, 90)
        cmds[5] = self.map_value_to_angle_deg(angles[5], 0, 88)
        if API_VERISON == 2.0:
            for n in range(6):
                set_speed_current_control_mode(
                    motor_id=n+1, 
                    speed=2000, # 速度值2000 (单位: 0.001行程/s)
                    current=800, # current = 800 电流值80 (单位: 0.001A)
                    target_angle=cmds[n]
                )
        else:
            update_motor_positions(cmds)
        self.last_cmd = angles

    def get_state_safe(self, id):
        try:
            for _ in range(100):
                value = get_motor_angles(id)
                if value is not None:
                    self.states_cache[id-1] = value
                    return value
                time.sleep(0.0001)
        except Exception as e:
            print(f"get_state_safe failed for id: {id}, error: {e}")
            return self.states_cache[id-1]


    def get_angles(self):
        if API_VERISON == 1.0:
            return self.last_cmd
        elif API_VERISON == 2.0:
            states = [0] * 6
            states[0] = float(self.map_angle_deg_to_value(self.get_state_safe(1), 0, 110))
            states[1] = float(self.map_angle_deg_to_value(self.get_state_safe(2), 0, 30))
            states[2] = float(self.map_angle_deg_to_value(self.get_state_safe(3), 0, 84))
            states[3] = float(self.map_angle_deg_to_value(self.get_state_safe(4), 0, 90))
            states[4] = float(self.map_angle_deg_to_value(self.get_state_safe(5), 0, 90))
            states[5] = float(self.map_angle_deg_to_value(self.get_state_safe(6), 0, 88))
            return states
        else:
            raise ValueError(f"Invalid API_VERISON: {API_VERISON}. Must be 1.0 or 2.0")

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
    # try:
    right_hand = RyRightHand()
    right_hand.init() 
    # left_hand = RyLeftHand()
    # N = 100
    # for n in range(N):
    #     _angle = n/(N-1)
    #     angles = [_angle,0.0,0,0,1.0,1.0]
    #     right_hand.set_angles(angles)        
    #     angles = right_hand.get_angles()
    #     print(f"angles: {angles}")
    #     time.sleep(0.01)
    
    # for n in range(N):
    #     _angle = n/(N-1)
    #     angles = [1-_angle,0.0,0,0,1.0,1.0]
    #     right_hand.set_angles(angles)        
    #     angles = right_hand.get_angles()
    #     print(f"angles: {angles}")
    #     time.sleep(0.01)
    
    # rxinfo = ServoData()
    # for i in range(6):
    #     # lib.RyParam_ClearFault(ctypes.byref(servo_bus), i + 1, 0)
    #     lib.RyFunc_GetServoInfo(ctypes.byref(servo_bus), i + 1, ctypes.byref(rxinfo), 0)
    #     sleep(0.0005)
    

    # angles = [0.0, 0.5, 0.0, 0.5, 0.0, 0.0]
    # right_hand.set_angles(angles)
    # time.sleep(0.5)
    # angles = right_hand.get_angles()
    # print(f"angles: {angles}")

    # angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # right_hand.set_angles(angles)
    # time.sleep(0.5)
    # angles = right_hand.get_angles()
    # print(f"angles: {angles}")
    
    for n in range(500):
        rotation_angle = np.sin(n/99*np.pi*2)
        angles = [rotation_angle, 0.5, 0.5, 0.5, 0.5, 0.5]
        right_hand.set_angles(angles.copy())
        time.sleep(0.01)
        # angles = right_hand.get_angles()
        print(f"angles_1: {angles}")

        # angles = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        # right_hand.set_angles(angles.copy())
        # time.sleep(1)
        # # angles = right_hand.get_angles()
        # print(f"angles_1: {angles}")


        # angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # right_hand.set_angles(angles.copy())
        # time.sleep(1)
        # # angles = right_hand.get_angles()
        # print(f"angles_2: {angles}")
