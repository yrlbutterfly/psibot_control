import numpy as np
from robot_libs.lib.robotic_arm import Arm, RM75
import numpy as np

def degrees_to_radians(degrees_list):
    """
    使用numpy批量转换角度到弧度
    
    Parameters:
    degrees_list (list/array): 角度值列表或数组
    
    Returns:
    ndarray: 弧度值数组
    """
    return np.deg2rad(degrees_list)

def radians_to_degrees(radians_list):
    """
    使用numpy批量转换弧度到角度
    
    Parameters:
    radians_list (list/array): 弧度值列表或数组
    
    Returns:
    ndarray: 角度值数组
    """
    return np.rad2deg(radians_list)

class ArmControl:
    def __init__(self, ip):
        self.robot = Arm(RM75, ip)

        # self.robot.Algo_Set_Joint_Min_Limit([-360.0, -360.0, -360.0, -360.0, -360.0, -360.0, -360.0])
        # self.robot.Algo_Set_Joint_Max_Limit([360.0, 360.0, 360.0, 360.0, 360.0, 360.0, 360.0])
        # print(self.robot.Algo_Get_Joint_Min_Limit())
        # print(self.robot.Algo_Get_Joint_Max_Limit())

        # self.robot.Algo_Set_Joint_Max_Speed([120, 120.0, 150.0, 150.0, 150.0, 150.0, 150.0])
        # self.robot.Algo_Set_Joint_Max_Acc([300.0, 300.0, 300.0, 300.0, 300.0, 300.0])
        # print(self.robot.Algo_Get_Joint_Max_Speed())
        # print(self.robot.Algo_Get_Joint_Max_Acc())

    def move_joint(self, joint_angles, speed=10, block=True):
        ret = self.robot.Movej_Cmd(radians_to_degrees(joint_angles), speed, 0, block=block)
        return ret
    
    def move_joint_CANFD(self, joint_angles):
        self.robot.Movej_CANFD(radians_to_degrees(joint_angles), False)

    def move_pose_Cmd(self, xyzrpy, speed=5, block=True):
        if isinstance(xyzrpy, np.ndarray):
            xyzrpy = xyzrpy.tolist()
        self.robot.Movej_P_Cmd(xyzrpy, speed, block=block)

    def move_pose(self, xyzrpy, speed=5, block=True):
        return self.move_pose_Cmd(xyzrpy, speed, block=block)

    def move_pose_CANFD(self, xyzrpy):
        self.robot.Movep_CANFD(xyzrpy, False)

    def get_current_joint(self):
        _, arm_joint, _, _, _ = self.robot.Get_Current_Arm_State()
        return degrees_to_radians(arm_joint)

    def get_current_pose(self):
        _, _, arm_pose, _, _ = self.robot.Get_Current_Arm_State()
        return arm_pose

    def close(self):
        self.robot.RM_API_UnInit()
        self.robot.Arm_Socket_Close()
