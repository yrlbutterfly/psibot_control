import numpy as np
from robot_libs.aoyi_hand_module_modbus import Hand
from robot_libs.realman_arm_module import ArmControl
from robot_libs.realsense_image_module import RealSenseImage, visualize_rgbd, generate_pcd
import time
import h5py
import threading
import cv2
import hydra
import dill
import os
from PIL import Image

save_paths = [
    '/home/zhangjiayuan/Project/DexKnotVLA/DexKnotVLA/data_replay/replay_hdf5/bag1'
]
robot_angles_path = 'robot_angles.txt'


hands = {
    "left": {
        "port": "/dev/ttyUSB0",
        "default_open":  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "default_close": [0.5, 0.5, 0.5, 0.5, 0.5, 1.0],
        "default_half":  [0.0, 0.7, 0.7, 0.0, 0.0, 0.0],
        "default_poke":  [0.9, 1.0, 1.0, 0.0, 0.0, 0.9],
        "default_hook":  [0.9, 0.0, 0.0, 0.0, 0.0, 0.9]
    },
    "right": {
        "port": "/dev/ttyUSB1",
        "default_open":  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "default_close": [0.5, 0.5, 0.5, 0.5, 0.5, 1.0],
        "default_half":  [0.0, 0.7, 0.7, 0.0, 0.0, 0.0],
        "default_poke":  [0.9, 1.0, 1.0, 0.0, 0.0, 0.9],
        "default_hook":  [0.9, 0.0, 0.0, 0.0, 0.0, 0.9]
    }
}




class RoboticsSystem:
    def __init__(self):
        # 初始化机械臂限位
        self.dof_lower_limits = [-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28]
        self.dof_upper_limits = [3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28]
        
        # 初始化手和机械臂
        self.left_hand = Hand(port=hands['left']['port'])
        self.right_hand = Hand(port=hands['right']['port'])
        self.left_arm = ArmControl(ip='192.168.100.100')
        self.right_arm = ArmControl(ip='192.168.100.101')
        
        left = self.left_arm.get_current_joint()
        right = self.right_arm.get_current_joint()
        
        print(f"[{left[0]:.6f}, {left[1]:.6f}, {left[2]:.6f}, {left[3]:.6f}, {left[4]:.6f}, {left[5]:.6f}, {left[6]:.6f}]")
        print(f"[{right[0]:.6f}, {right[1]:.6f}, {right[2]:.6f}, {right[3]:.6f}, {right[4]:.6f}, {right[5]:.6f}, {right[6]:.6f}]")
        

    def low_level_hand_control(self):
        while True:
            # self.left_hand.set_angles(self.target_left_hand_joint)
            self.right_hand.set_angles(self.target_right_hand_joint)
            time.sleep(0.1)
            
    def left_hand_move(self, qpos):
        # print("left hand")
        self.left_arm.robot.Clear_System_Err()
        left_init_qpos = np.clip(qpos, self.dof_lower_limits, self.dof_upper_limits)
        self.left_arm.move_joint(left_init_qpos, speed=10)

    def right_hand_move(self, qpos):
        # print("right hand")
        self.right_arm.robot.Clear_System_Err()
        right_init_qpos = np.clip(qpos, self.dof_lower_limits, self.dof_upper_limits)
        self.right_arm.move_joint(right_init_qpos, speed=10)
        
    def move_both_arm(self, left_qpos, right_qpos):
        t_left = threading.Thread(target=self.left_hand_move, args=(left_qpos,))
        t_right = threading.Thread(target=self.right_hand_move, args=(right_qpos,))
        t_left.start();t_right.start()
        t_left.join();t_right.join()
        print("two arms move finish!")
        
    def move_both_hand(self, left_hand_pos, right_hand_pos):
        t_left = threading.Thread(target=self.left_hand.set_angles, args=(left_hand_pos,))
        t_right = threading.Thread(target=self.right_hand.set_angles, args=(right_hand_pos,))
        t_left.start();t_right.start()
        t_left.join();t_right.join()
        print("two hands move finish!")
        
        

    def close(self):
        # self.target_left_hand_joint = self.left_hand_init_qpos
        # self.target_right_hand_joint = self.right_hand_init_qpos
        # self.left_hand.set_angles(self.target_left_hand_joint)
        # self.right_hand.set_angles(self.target_right_hand_joint)
        # self.right_arm.robot.Stop_Drag_Teach()
        self.left_hand.close()
        self.right_hand.close()
        self.left_arm.close()
        self.right_arm.close()

if __name__ == "__main__":
    system = RoboticsSystem()
    # system.collect_and_replay()
    system.close()
