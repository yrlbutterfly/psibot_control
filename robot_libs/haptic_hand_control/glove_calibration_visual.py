#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PSI手套角度rerun可视化器
使用rerun对每个关节角度进行实时可视化
"""

import time
import argparse
import numpy as np
import rerun as rr

from glove_reader import SimpleGloveReader

# 所有关节角度索引和名称
ALL_ANGLES = list(range(20))  # 0-19, 总共20个关节
ALL_ANGLES_NAMES = [f"joint_{i}" for i in range(20)]

# ================================ 右手配置 =======================================================================
# 1. 默认选择的关节角度索引和名称（右手）
# SELECTED_ANGLES = [18, 19, 14, 10, 4, 2]  # for right hand
# SELECTED_ANGLES_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend"]

# # 2. 检查关节ID是否正确
# IF_USE_SELECTED_INDEX = True

# # 3. 检查方向归一化参数
# IF_NORMALIZE_DIRECTION = True
# SELECTED_DIRECT = [-1, -1, -1, -1, -1, -1]  # for left hand

# # 4. 检查数值归一化参数
# IF_NORMALIZE_BOUND = True
# ANGLES_MIN = [-240, -253, -235, -223, -228, -252]
# ANGLES_MAX = [-210, -222, -216, -203, -203, -216]

# # ==================================  左手配置 ============================================================
# # 1. 默认选择的关节角度索引和名称
# SELECTED_ANGLES = [1, 2, 3, 11, 15, 19]  # for left hand
# SELECTED_ANGLES_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend"]

# # 2. 检查关节ID是否正确
# IF_USE_SELECTED_INDEX = False

# # 3. 检查方向归一化参数
# IF_NORMALIZE_DIRECTION = False
# SELECTED_DIRECT = [1, -1, -1, -1, -1, -1]  # for left hand

# # 4. 检查数值归一化参数
# IF_NORMALIZE_BOUND = False
# ANGLES_MIN = [170, -250, -225, -220, -230, -252]
# ANGLES_MAX = [190, -225, -205, -197, -210, -219]


# # ==================================  左手配置(new) ============================================================
# # 1. 默认选择的关节角度索引和名称
# SELECTED_ANGLES = [3, 2, 7, 11, 15, 19]  # for left hand
# SELECTED_ANGLES_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend"]

# # 2. 检查关节ID是否正确
# IF_USE_SELECTED_INDEX = False

# # 3. 检查方向归一化参数
# IF_NORMALIZE_DIRECTION = False
# SELECTED_DIRECT = [-1, -1, -1, -1, -1, -1]  # for left hand

# # 4. 检查数值归一化参数
# IF_NORMALIZE_BOUND = False
# ANGLES_MIN = [-185, -270, -230, -225, -230, -236]
# ANGLES_MAX = [-170, -242, -215, -210, -215, -215]


### 0829 
# ==================================  左手配置(2025.8.25) ============================================================
# 1. 默认选择的关节角度索引和名称
SELECTED_ANGLES = [3, 0, 5, 9, 13, 17]  # for left hand
SELECTED_ANGLES_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend"]

# 2. 检查关节ID是否正确
IF_USE_SELECTED_INDEX =     True

# 3. 检查方向归一化参数
IF_NORMALIZE_DIRECTION = True
SELECTED_DIRECT = [1, 1, 1, 1, 1, 1]  # for left hand

# 4. 检查数值归一化参数
IF_NORMALIZE_BOUND = True
ANGLES_MIN = [35, 65, 65, 65, 65, 65]
ANGLES_MAX = [45, 80, 80, 80, 80, 80]



class GloveAngleVisualizer:
    """PSI手套角度rerun可视化器"""
    
    def __init__(self, port="/dev/USB0"
    "", update_rate=100.0):
        self.port = port
        self.update_rate = update_rate
        self.reader = None
        
        # 确定要可视化的关节
        if IF_USE_SELECTED_INDEX:
            # 可视化默认选择的关节
            self.angles_to_visualize = SELECTED_ANGLES
            self.angle_names = SELECTED_ANGLES_NAMES
        else:
            # 可视化所有关节
            self.angles_to_visualize = ALL_ANGLES
            self.angle_names = ALL_ANGLES_NAMES
        
        # 统计信息
        self.read_count = 0
        self.success_count = 0
        self.last_stats_time = time.time()
        
        # 初始化rerun
        self.init_rerun()
        
        # 初始化手套读取器
        self.init_glove_reader()
        
    def init_rerun(self):
        """初始化rerun可视化"""
        # 启动rerun服务器
        rr.init("PSI手套角度可视化", spawn=True)
        print("🎨 已启动rerun可视化服务器")
        
        # 设置时间轴
        rr.set_time_seconds("sim_time", time.time())
        
        # 设置默认的scales配置
        rr.log("angles", rr.Scalar(0.0), ext={"scales": {
            "angles": {
                "kind": "linear", 
                "range": [-400, 400],
                "label": "Angle (degrees)"
            }
        }})
        
    def init_glove_reader(self):
        """初始化手套读取器"""
        try:
            self.reader = SimpleGloveReader(port=self.port)
            print(f'✅ 成功连接到手套设备: {self.port}')
            
            # 等待设备连接
            while True:
                if self.reader.read_angles() is not None:
                    break
                print(f'⏳ 等待手套设备连接...')
                time.sleep(0.1)
                
        except Exception as e:
            print(f'❌ 初始化手套读取器失败: {e}')
            raise
    
    def visualize_angles(self):
        """可视化角度数据"""
        try:
            # 读取角度数据
            all_angles = self.reader.read_angles()
            if all_angles is None:
                print('⚠️ 读取角度数据失败')
                return
            
            # 提取要可视化的角度
            if IF_USE_SELECTED_INDEX:
                angles = np.asarray(all_angles)[SELECTED_ANGLES]
            else:
                angles = np.asarray(all_angles)
                
                        
                
            # 应用方向修正（仅对默认选择的关节）
            if IF_NORMALIZE_DIRECTION and IF_USE_SELECTED_INDEX:
                angles = [angle * SELECTED_DIRECT[i] for i, angle in enumerate(angles)]
            
            # 归一化处理（仅对默认选择的关节）
            if IF_NORMALIZE_BOUND and IF_USE_SELECTED_INDEX:
                angles = [(angle - ANGLES_MIN[i]) / (ANGLES_MAX[i] - ANGLES_MIN[i]) for i, angle in enumerate(angles)]
                angles = [max(0, min(1, angle)) for angle in angles]

            
            
            # 更新rerun时间轴
            current_time = time.time()
            rr.set_time_seconds("/sim_time", current_time)
            
            if len(angles) > 1:
                angles = np.clip(angles, -500, 500)
                # for angle_name, angle in zip(self.angle_names, angles):
                #     rr.log(f"angles/{angle_name}", rr.Scalar(angle)) 
                rr.log("angles", rr.BarChart(angles))                               
            
            # 更新统计信息
            self.read_count += 1
            self.success_count += 1
            
            # 定期输出统计信息
            if current_time - self.last_stats_time >= 5.0:
                elapsed_time = current_time - self.last_stats_time
                avg_frequency = self.read_count / elapsed_time
                success_rate = (self.success_count / self.read_count * 100) if self.read_count > 0 else 0
                
                print(f'📊 统计信息 - 频率: {avg_frequency:.1f}Hz, 成功率: {success_rate:.1f}%, '
                      f'角度范围: [{min(angles):.2f}, {max(angles):.2f}]')
                
                
                # 重置计数器
                self.last_stats_time = current_time
                self.read_count = 0
                self.success_count = 0
                
        except Exception as e:
            print(f'❌ 可视化角度数据时出错: {e}')
    
    def run_visualization(self):
        """运行可视化循环"""
        print(f"🎯 使用串口: {self.port}")
        print(f"📊 更新频率: {self.update_rate}Hz")
        print(f"🔧 归一化: {'启用' if IF_NORMALIZE_BOUND else '禁用'}")
        print(f"🔧 方向归一化: {'启用' if IF_NORMALIZE_DIRECTION else '禁用'}")
        
        # 显示可视化模式
        mode_descriptions = {
            'all': '所有关节 (0-19)',
            'selected': '默认选择关节 (6个主要关节)',
            'default': '默认模式 (所有关节)'
        }
        print(f"📡 可视化模式: {mode_descriptions['selected'] if IF_USE_SELECTED_INDEX else mode_descriptions['all']}")
        print("🎨 可视化内容:")
        for angle_name in self.angle_names:
            print(f"  - {angle_name}: 时间序列图")
        print("  - comparison/all_angles: 所有关节对比图")
        print("  - stats/*: 统计信息")
        print("\n按Ctrl+C停止")
        
        try:
            while True:
                self.visualize_angles()
                time.sleep(1.0 / self.update_rate)
                
        except KeyboardInterrupt:
            print("\n程序停止")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        if self.reader:
            self.reader.close()
            print('✅ 已关闭手套读取器')


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='PSI手套角度rerun可视化器')
    parser.add_argument('--port', action='store', default="/dev/ttyACM0",
                       help='串口端口')
    parser.add_argument('--rate', type=float, default=100.0,
                       help='更新频率 (Hz)')
    parser.add_argument('--mode', choices=['all', 'selected', 'default'], default='default',
                       help='可视化模式: all(所有关节), selected(默认选择关节), default(默认模式)')
    args = parser.parse_args()
    

    # 创建可视化器
    visualizer = GloveAngleVisualizer(
        port=args.port, 
        update_rate=args.rate,
    )
    
    # 运行可视化
    visualizer.run_visualization()


if __name__ == "__main__":
    main()
