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

# === 配置区域 ===
# 数据保存路径：用于存储示教和重放过程中的数据（HDF5格式）
save_paths = [
    '/home/zhangjiayuan/Project/DexKnotVLA/DexKnotVLA/data_replay/replay_hdf5/bag1'
]
# 机器人关节角度临时存储文件
robot_angles_path = 'robot_angles.txt'

# 灵巧手配置：定义左右手的串口端口和不同姿态的预设关节角度
# 每个手有6个自由度（或类似控制维度）
hands = {
    "left": {
        "port": "/dev/ttyUSB0", # 左手串口地址
        "default_open":  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], # 全张开
        "default_close": [0.5, 0.5, 0.5, 0.5, 0.5, 1.0], # 闭合
        "default_half":  [0.0, 1.0, 1.0, 0.0, 0.0, 0.0], # 半握
        "default_poke":  [0.9, 1.0, 1.0, 0.0, 0.0, 0.9], # 戳/点
        "default_hook":  [0.9, 0.0, 0.0, 0.0, 0.0, 0.9], # 钩
        "half": [0.75, 0.75, 0.75, 0.75, 0.75, 1.0],
        "close": [0.4, 0.4, 0.4, 0.4, 0.4, 1.0]
    },
    "right": {
        "port": "/dev/ttyUSB1", # 右手串口地址
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
    """
    机器人控制系统主类
    负责初始化机械臂、灵巧手，并执行示教（Teach）、重放（Replay）和数据采集
    """
    def __init__(self):
        # 初始化机械臂各关节的运动限位 (弧度制)
        self.dof_lower_limits = [-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28]
        self.dof_upper_limits = [3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28]
        
        # === 硬件初始化 ===
        # 初始化左右灵巧手
        self.left_hand = Hand(port=hands['left']['port'])
        self.right_hand = Hand(port=hands['right']['port'])
        # 初始化左右机械臂 (通过IP连接)
        self.left_arm = ArmControl(ip='192.168.100.100')
        self.right_arm = ArmControl(ip='192.168.100.101')

        # 初始化相机 (注释掉的部分表明可能连接了RealSense相机)
        # self.chest_camera = RealSenseImage("013222072349")
        # self.right_camera = RealSenseImage("230322272019")
        # self.top_camera = RealSenseImage("050122072371")
        
        
        # exit()
        
        # 设置手部为半握状态
        self.move_both_hand(hands['left']['half'], hands['right']['half'])
  
        # === 设置机械臂初始位置 ===
        # qpos (Joint Positions) 是一组7个关节角度
        self.left_init_qpos = [0.262044, -1.214400, -0.514802, -1.596121, -1.199373, -1.059380, 2.984618]
        self.right_init_qpos = [-0.010699, -1.099767, 0.276198, -1.602980, -1.640941, 1.259412, -0.131388]
        # 同时移动两臂到初始位置
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
        # 下面的代码是各种预设动作的调试代码 (catch, lift, forward, push等)
        # 实际运行时，最后生效的是后面的代码块
        # self.left_hand_move(self.left_init_qpos)
        
        # self.left_hand.set_angles(hands['left']['close'])
        
        # self.left_init_qpos = [-1.505765, -1.655183, 0.270020, -0.903487, -1.742903, -0.668775, 2.574570]
        # self.left_hand_move(self.left_init_qpos)
        
        # self.left_hand.set_angles(hands['left']['half'])

        # catch
        self.left_init_qpos = [-0.158738, -2.008350, -0.499583, -0.517036, -1.283463, -0.869523, 3.052110]
        self.right_init_qpos = [0.195878, -1.879667, 0.217869, -0.842837, -1.782889, 0.886819, -0.125175]
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
        self.move_both_hand(hands['left']['close'], hands['right']['close'])
        
        # lift
        self.left_init_qpos = [0.632490, 0.401775, 0.111369, -1.331756, -2.264320, -2.027793, 1.154884]
        self.right_init_qpos = [0.151041, 0.525641, -0.104423, -1.299345, -1.378688, 1.790324, 1.915289]
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

        self.left_init_qpos = [-0.746949, -0.378597, -0.098594, -0.398371, -1.122474, -0.655057, 1.454522]
        self.right_init_qpos = [0.954678, -0.102905, 0.527945, -0.909910, -2.618465, 0.322868, 2.072875]
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)


        self.left_init_qpos = [-1.151237, -1.209984, -0.802310, -0.784630, -0.410100, -0.355873, 1.524475]
        self.right_init_qpos = [1.541649, -1.035940, 0.618370, -1.167137, -2.503064, -0.181305, 1.861167]
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

        self.left_init_qpos = [-0.959495, -0.933629, -0.633589, -1.262414, -0.410711, -0.007278, 1.158480]
        self.right_init_qpos = [2.038824, -1.268854, 0.560041, -0.492427, -2.626808, 0.277333, 1.932045]
        self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

        self.right_init_qpos = [1.984649, -0.547457, 0.634218, -1.670106, -2.321724, 0.456089, 1.913527]
        self.right_hand_move(self.right_init_qpos)

        self.right_init_qpos = [1.157398, 0.047106, -0.072763, -1.663910, -2.068285, 1.766413, 1.386524]
        self.right_hand_move(self.right_init_qpos)

        self.right_init_qpos = [1.479533, -0.823813, 0.062692, -1.268732, -1.825475, 0.384740, 1.154116]
        self.right_hand_move(self.right_init_qpos)

        self.move_both_hand(hands['left']['half'], hands['right']['half'])

        # forward
        # self.left_init_qpos = [-0.835123, -0.767247, -0.896453, -0.869087, -3.070872, 0.765344, 4.043858]
        # self.right_init_qpos = [1.217419, -1.186283, -1.979448, 0.144478, -0.253701, 1.148374, 2.150874]
        # self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
        # # move
        # self.left_init_qpos = [-0.955969, -0.810967, -0.759497, -0.590794, -3.057066, 1.145739, 4.084629]
        # self.right_init_qpos = [2.084412, -1.247718, -1.985329, 0.457241, -0.734295, 0.465863, 2.482050]
        # self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

        # self.right_init_qpos = [1.147729, -0.949494, -1.464995, 1.636560, -0.278380, 0.769708, 2.497304]
        # self.right_hand_move(self.right_init_qpos)

        # self.right_init_qpos = [-0.299149, -0.933507, -0.797423, 1.745399, -0.255097, 1.016654, 2.489468]
        # self.right_hand_move(self.right_init_qpos)

        # self.right_init_qpos = [0.722357, -0.965638, -1.785105, 1.353398, -0.748799, 0.283459, 2.490079]
        # self.right_hand_move(self.right_init_qpos)

        # # # push
        # self.left_init_qpos = [-1.172652, -1.026341, -0.878895, -1.079539, -0.716842, -0.096011, 1.883123]
        # self.right_init_qpos = [1.275225, -1.102228, 0.814615, -0.972201, -2.230025, 0.118682, 1.076554]
        # self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
        # self.move_both_hand(hands['left']['half'], hands['right']['half'])
        
        # # move away
        # self.left_init_qpos = [-0.475934, -0.831580, -1.414083, -1.116539, -1.584166, 0.243683, 2.981755]
        # self.right_init_qpos = [0.763128, -0.835175, 1.343275, -0.833499, -1.322209, -0.458812, -0.315870]
        # self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
        # # initial
        # self.left_init_qpos = [0.262044, -1.214400, -0.514802, -1.596121, -1.199373, -1.059380, 2.984618]
        # self.right_init_qpos = [-0.010699, -1.099767, 0.276198, -1.602980, -1.640941, 1.259412, -0.131388]
        # self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
# #         self.left_init_qpos = [-0.25942575, -1.64560119, -0.38126716, -1.03499768, -0.24373523, -1.27997208,
# #  -0.36543703]        
# #         self.right_init_qpos = [0.57630773, -1.73106989,  0.16043067, -0.66228262, -2.65993414,  1.56447824,
# #   3.48737722]
# #         self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
#         self.move_both_hand(hands['left']['default_close'], hands['right']['default_close'])
        
#         self.left_init_qpos = [-0.61421629, -0.88289223, -0.6077062,  -1.85203865, -0.48274062, -1.2182922,
#  -0.5411568]        
#         self.right_init_qpos = [0.85008009, -0.74303905,  0.21275563, -1.90579483, -2.43283201,  1.35891337,
#   3.5415872]
#         self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

#         self.left_init_qpos = [-1.29894391, -0.99872975,  0.85885907, -1.07505302, -1.30950307, -1.30388314,
#  -0.28705431]        
#         self.right_init_qpos = [1.37650621, -1.49920297,  0.06736971, -0.66118307, -2.45168133,  0.93584553,
#   3.98748399]
#         self.move_both_arm(self.left_init_qpos, self.right_init_qpos)

        # self.move_both_hand(hands['left']['half'], hands['right']['half'])
        
#         self.left_init_qpos = [0.23202407, -1.52615083, -1.22461029, -1.2310855,  -0.2589894,  -0.79480551,
#   1.10266414]        
#         self.right_init_qpos = [0.30047588, -1.15284236,  0.32201326, -1.16469316, -1.73398459,  1.03967519,
#   1.50550361]
        
#         self.move_both_arm(self.left_init_qpos, self.right_init_qpos)
        
#         self.left_init_qpos = [0.00420624, -1.5621569,  -1.10800484, -0.9151983,  -0.32840115, -0.83121306,
#   1.15153328]        
#         self.right_init_qpos = [0.32086134, -1.40366366,  0.33704055, -0.82203261, -1.73550299,  0.91416853,
#   1.50496245]
        
#         self.move_both_arm(self.left_init_qpos, self.right_init_qpos)


        
        # color_image, depth_image = self.top_camera.capture_frame()
        
        # print(color_image.shape)
        # print(depth_image.shape)
        
        # image = Image.fromarray(color_image)
        # image.save("image.png")
        
        # visualize_rgbd(color_image=color_image, depth_image=depth_image)
        
        # pcd = generate_pcd(color_image, depth_image, self.top_camera.o3d_intrinsics, visualize_flag=True)  

        
        

    def collect_and_replay(self):
        """
        核心流程：数据采集与重放
        1. 示教 (Teach): 允许用户手动拖动机械臂，记录轨迹。
        2. 重放 (Replay): 机械臂自动复现记录的轨迹，并同时采集图像数据。
        3. 保存: 将关节数据和图像数据保存为HDF5文件，用于训练。
        """

        while True:
            try:
                # 清除之前的错误状态
                self.left_arm.robot.Clear_System_Err()
                self.right_arm.robot.Clear_System_Err()

                # 复位到初始位置
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

                # 1. 示教阶段 (Teach Phase)
                time.sleep(1)
                print("Starting teach mode for 200 steps...")
                # 打开文件准备写入示教轨迹
                trajectory_file = open(robot_angles_path, 'w')
                trajectory_file.write('step,'
                                    'left_arm_j1,left_arm_j2,left_arm_j3,left_arm_j4,left_arm_j5,left_arm_j6,left_arm_j7,'
                                    'right_arm_j1,right_arm_j2,right_arm_j3,right_arm_j4,right_arm_j5,right_arm_j6,right_arm_j7,'
                                    'left_hand_j1,left_hand_j2,left_hand_j3,left_hand_j4,left_hand_j5,left_hand_j6,'
                                    'right_hand_j1,right_hand_j2,right_hand_j3,right_hand_j4,right_hand_j5,right_hand_j6\n')
        
                self.right_arm.robot.Clear_System_Err()
                # 开启“零力拖动”模式，允许用户手动移动机械臂
                self.right_arm.robot.Start_Multi_Drag_Teach(0, 0)
                

                trajectory_data = []

                prev_time = time.time()
                step = 0
                
                # 录制循环：固定95步，采样率约20Hz
                nsteps = 95
                while step < nsteps:
                    
                    if time.time() - prev_time >= 0.05:  # 50ms = 20Hz
                        print(step)
                        # 记录当前时刻的左臂、右臂、左手、右手的状态
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
                        # 在最后阶段自动执行一个动作 (例如 poke)
                        # self.target_right_hand_joint = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 1.0])
                        self.target_right_hand_joint = hands['right']['default_poke']

                # 停止拖动示教模式
                self.right_arm.robot.Stop_Drag_Teach()
                # 保存轨迹到文本文件
                trajectory_file.writelines(trajectory_data)
                trajectory_file.close()

                # 2. 重放与采集阶段 (Replay & Collect Phase)
                
                print("\nStarting replay and data collection...")
                
                # 重置机械臂状态
                self.right_arm.robot.Clear_System_Err()
                self.right_arm.move_joint(self.right_init_qpos, speed=10)

                self.target_right_hand_joint = self.right_hand_init_qpos

                time.sleep(3)

                # 准备HDF5文件，自动递增文件名 (0.hdf5, 1.hdf5, ...)
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

                # 创建HDF5数据集
                # for cam in ['right_camera', 'top_camera']:
                for cam in [ 'top_camera']:
                    h5_file.create_dataset(cam, shape=(nsteps, 480, 640, 3), dtype='uint8')
                for cam in ['top_camera_depth']:
                    h5_file.create_dataset(cam, shape=(nsteps, 480, 640), dtype='uint16')

                # 从刚才录制的txt文件读取轨迹数据
                data = np.loadtxt(robot_angles_path, delimiter=',', skiprows=1)

                h5_file.create_dataset('teach_joints', shape=(len(data), 13), dtype='float32')  # 示教的关节角度 (预期)
                h5_file.create_dataset('replay_joints', shape=(len(data), 13), dtype='float32')  # 重放时的实际关节角度

                target_dt = 0.05  # 20Hz = 0.05s
                prev_time = time.time()

                # 开始重放
                user_input = input("Press Enter to start replay")
                print('replay')
                for i in range(len(data)):
                    cycle_start = time.time()

                    # 获取第i步的右臂关节数据 (注意索引 8:15 对应 txt 中的 right_arm columns)
                    right_arm = np.clip(data[i, 8:15], self.dof_lower_limits, self.dof_upper_limits)

                    teach_angles = np.concatenate([
                        right_arm,
                        data[i, 21:27]
                    ])

                    h5_file['teach_joints'][i] = teach_angles

                    # 执行移动：机械臂和手
                    # 使用 CANFD 总线通信，更实时
                    self.right_arm.move_joint_CANFD(right_arm)
                    # 设置手的角度
                    self.target_right_hand_joint = data[i, 21:27]

                    # 记录实际执行的角度
                    actual_angles = np.concatenate([
                        self.right_arm.get_current_joint(),
                        self.right_hand.get_angles()
                    ])

                    h5_file['replay_joints'][i] = actual_angles

                    # 采集图像 (注意：这里只有 top_camera 是启用的)
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

                    # 精确控制频率，确保回放节奏与示教一致
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
