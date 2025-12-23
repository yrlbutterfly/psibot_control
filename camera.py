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
import open3d as o3d

save_paths = [
    '/home/zhangjiayuan/Project/DexKnotVLA/DexKnotVLA/data_replay/replay_hdf5/bag1'
]
robot_angles_path = 'robot_angles.txt'


hands = {
    "left": {
        "port": "/dev/ttyUSB0",
        "default_open":  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "default_close": [0.5, 0.5, 0.5, 0.5, 0.5, 1.0],
        "default_half":  [0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        "default_poke":  [0.9, 1.0, 1.0, 0.0, 0.0, 0.9],
        "default_hook":  [0.9, 0.0, 0.0, 0.0, 0.0, 0.9],
        "half": [0.75, 0.75, 0.75, 0.75, 0.75, 1.0],
        "close": [0.4, 0.4, 0.4, 0.4, 0.4, 1.0]
    },
    "right": {
        "port": "/dev/ttyUSB1",
        "default_open":  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "default_close": [0.5, 0.5, 0.5, 0.5, 0.5, 1.0],
        "default_half":  [0.0, 0.7, 0.7, 0.0, 0.0, 0.0],
        "default_poke":  [0.9, 1.0, 1.0, 0.0, 0.0, 0.9],
        "default_hook":  [0.9, 0.0, 0.0, 0.0, 0.0, 0.9],
        "half": [0.75, 0.75, 0.75, 0.75, 0.75, 1.0],
        "close": [0.4, 0.4, 0.4, 0.4, 0.4, 1.0]
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

        # 初始化相机
        # self.chest_camera = RealSenseImage("013222072349")
        # self.right_camera = RealSenseImage("230322272019")
        self.top_camera = RealSenseImage("134222070573")
        
        
        # exit()
        
        color_image, depth_image = self.top_camera.capture_frame()
        
        # print(color_image.shape)
        # print(depth_image.shape)

        print(depth_image.dtype)
        
        image = Image.fromarray(color_image)
        image.save("image.png")

        cv2.imwrite("depth.png", depth_image)
        print("深度图已保存为 depth.png")
        
        # visualize_rgbd(color_image=color_image, depth_image=depth_image)

        intrinsics_file = "camera_intrinsics.json"
        o3d.io.write_pinhole_camera_intrinsic("camera_intrinsics.json", self.top_camera.o3d_intrinsics)
        print(f"相机内参已保存到 {intrinsics_file}")
        
        # pcd = generate_pcd(color_image, depth_image, self.top_camera.o3d_intrinsics, visualize_flag=True)  

        
        

    def collect_and_replay(self):

        while True:
            try:
                self.left_arm.robot.Clear_System_Err()
                self.right_arm.robot.Clear_System_Err()

                self.left_arm.move_joint(self.left_init_qpos, speed=10)
                self.right_arm.move_joint(self.right_init_qpos, speed=10)

                self.target_left_hand_joint = self.left_hand_init_qpos
                self.target_right_hand_joint = self.right_hand_init_qpos


                # 显示路径选项
                print("\n请选择数据保存路径：")
                for i, path in enumerate(save_paths):
                    print(f"{i}. {path}")
                
                path_choice = input("\n请输入路径编号 (直接回车使用默认路径0): ")
                base_path = save_paths[int(path_choice) if path_choice else 0]
                print(f"选择的保存路径: {base_path}")

                if not os.path.exists(base_path):
                    os.makedirs(base_path)

                # 1. 示教阶段
                time.sleep(1)
                print("Starting teach mode for 200 steps...")
                trajectory_file = open(robot_angles_path, 'w')
                trajectory_file.write('step,'
                                    'left_arm_j1,left_arm_j2,left_arm_j3,left_arm_j4,left_arm_j5,left_arm_j6,left_arm_j7,'
                                    'right_arm_j1,right_arm_j2,right_arm_j3,right_arm_j4,right_arm_j5,right_arm_j6,right_arm_j7,'
                                    'left_hand_j1,left_hand_j2,left_hand_j3,left_hand_j4,left_hand_j5,left_hand_j6,'
                                    'right_hand_j1,right_hand_j2,right_hand_j3,right_hand_j4,right_hand_j5,right_hand_j6\n')
        
                self.right_arm.robot.Clear_System_Err()
                # self.target_right_hand_joint = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
                # self.right_arm.move_joint(self.right_init_qpos, speed=10)
                self.right_arm.robot.Start_Multi_Drag_Teach(0, 0)
                

                trajectory_data = []

                prev_time = time.time()
                step = 0
                
                # 固定75步，20Hz采样
                nsteps = 95
                while step < nsteps:
                    
                    if time.time() - prev_time >= 0.05:  # 50ms = 20Hz
                        print(step)
                        trajectory_data.append(f'{step},' + 
                                    ','.join(map(str, self.left_arm.get_current_joint())) + ',' +
                                    ','.join(map(str, self.right_arm.get_current_joint())) + ',' +
                                    ','.join(map(str, self.left_hand.get_angles())) + ',' +
                                    ','.join(map(str, self.right_hand.get_angles())) + '\n')

                        step += 1
                        prev_time = time.time()


                    if step < 80:
                        pass
                    else:
                        # self.target_right_hand_joint = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 1.0])
                        self.target_right_hand_joint = hands['right']['default_poke']

                self.right_arm.robot.Stop_Drag_Teach()
                trajectory_file.writelines(trajectory_data)
                trajectory_file.close()

                # 2. replay
                
                print("\nStarting replay and data collection...")
                

                self.right_arm.robot.Clear_System_Err()
                self.right_arm.move_joint(self.right_init_qpos, speed=10)

                self.target_right_hand_joint = self.right_hand_init_qpos

                time.sleep(3)

                # prepare hdf5 file
                existing_numbers = []
                for f in os.listdir(base_path):
                    if f.endswith('.hdf5'):
                        try:
                            num = int(f.split('.')[0])
                            existing_numbers.append(num)
                        except (IndexError, ValueError):
                            continue
                
                next_number = max(existing_numbers + [-1]) + 1  # 如果列表为空，使用-1
                save_path = os.path.join(base_path, f'{next_number}.hdf5')
                print(f"数据将保存到: {save_path}")
                # chest_image_dir = os.path.join(base_path, f'{next_number}')
                # if not os.path.exists(chest_image_dir):
                #     os.mkdir(chest_image_dir)

                h5_file = h5py.File(save_path, 'w')

                # 创建数据集
                # for cam in ['right_camera', 'top_camera']:
                for cam in [ 'top_camera']:
                    h5_file.create_dataset(cam, shape=(nsteps, 480, 640, 3), dtype='uint8')
                for cam in ['top_camera_depth']:
                    h5_file.create_dataset(cam, shape=(nsteps, 480, 640), dtype='uint16')

                # 读取轨迹数据
                data = np.loadtxt(robot_angles_path, delimiter=',', skiprows=1)

                h5_file.create_dataset('teach_joints', shape=(len(data), 13), dtype='float32')  # 重放的关节角度
                h5_file.create_dataset('replay_joints', shape=(len(data), 13), dtype='float32')  # 重放的关节角度

                target_dt = 0.05  # 20Hz = 0.05s
                prev_time = time.time()

                # replay
                user_input = input("Press Enter to start replay")
                print('replay')
                for i in range(len(data)):
                    cycle_start = time.time()

                    right_arm = np.clip(data[i, 8:15], self.dof_lower_limits, self.dof_upper_limits)

                    teach_angles = np.concatenate([
                        right_arm,
                        data[i, 21:27]
                    ])

                    h5_file['teach_joints'][i] = teach_angles

                    # 移动机械臂
                    self.right_arm.move_joint_CANFD(right_arm)
                    # 设置手的角度
                    self.target_right_hand_joint = data[i, 21:27]

                    actual_angles = np.concatenate([
                        self.right_arm.get_current_joint(),
                        self.right_hand.get_angles()
                    ])

                    h5_file['replay_joints'][i] = actual_angles

                    # 采集图像
                    # # Capture from right camera
                    # color_image, depth_image = self.right_camera.capture_frame()
                    # h5_file['right_camera'][i] = color_image

                    # Capture from top camera
                    color_image, depth_image = self.top_camera.capture_frame()

                    h5_file['top_camera'][i] = color_image
                    h5_file['top_camera_depth'][i] = depth_image
                     

                    # if i >= 50:
                    #     chest_image_path = os.path.join(chest_image_dir, f'{i}.jpg')
                    #     cv2.imwrite(chest_image_path, self.chest_camera.capture_rgb_frame())

                    # 精确控制频率
                    elapsed = time.time() - cycle_start
                    if elapsed < target_dt:
                        while time.time() - cycle_start < target_dt:
                            pass  # 主动等待到达目标时间间隔
                    else:
                        print(f"Warning: Cycle {i} took {elapsed:.4f}s, longer than target {target_dt}s")

                h5_file.attrs['actual_frames'] = len(data)
                h5_file.close()


                print(f"\nCompleted cycle {next_number}")
                user_input = input("Press Enter to start next teaching cycle... Press Q to stop...")
                if user_input.upper() == 'Q':
                    break

            except Exception as e:
                print(f"Error: {e}")
                break

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
