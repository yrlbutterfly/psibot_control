#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PSI手套角度可视化测试器
提供实时角度数据的可视化功能
"""

import time
import argparse
import numpy as np
from collections import deque

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    ENABLE_PLOT = True
except ImportError:
    ENABLE_PLOT = False
    print("警告: matplotlib未安装，无法启用绘图功能")

from glove_reader import SimpleGloveReader

# all_angles = [n for n in range(20)]


IF_NORMALIZE = False
# 选择的关节角度索引和名称
# SELECTED_ANGLES = [1, 0+2, 3, 9+1, 13+1, 17+1]  # for left hand
# SELECTED_DIRECT = [-1, -1, -1, 1, 1, 1]  # for left hand
# ANGLES_MIN = [-210.8, -275.9, -245, 170.1, 159.9, 187.2]
# ANGLES_MAX = [-160.1, -230.1, -220, 326.3, 325.4, 337.6]

SELECTED_ANGLES = [4, 2, 7, 11, 15, 19]   # for right hand
SELECTED_DIRECT = [1, 1, 1, 1, 1, 1]  # for right hand
ANGLES_MIN = [-45, 65, 65, 65, 65, 65]
ANGLES_MAX = [-35, 80, 80, 80, 80, 80]

# SELECTED_ANGLES = [0, 5, 9, 13, 17, 18]  # for right hand
# SELECTED_ANGLES = all_angles
SELECTED_ANGLES_NAMES = ["thumb_rotate", "thumb_bend", "index_bend", "middle_bend", "ring_bend", "pinky_bend", ]
# SELECTED_ANGLES_NAMES = [f"joint_{i}" for i in range(len(all_angles))]


class SingleWindowAnglePlotter:
    """实时角度折线图绘制器 - 单窗口模式（所有关节在一个图表中）"""
    
    def __init__(self, max_points=100):
        if not ENABLE_PLOT:
            raise ImportError("matplotlib未安装，无法使用绘图功能")
            
        self.max_points = max_points
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.lines = []
        self.data_queues = []
        
        # 添加动态缩放相关属性
        self.y_min = float('inf')
        self.y_max = float('-inf')
        self.y_margin = 0.1  # y轴边距比例
        
        # 初始化选定关节的数据队列
        joint_num = len(SELECTED_ANGLES)
        for i in range(joint_num):
            self.data_queues.append(deque(maxlen=max_points))
        
        # 设置图表
        self.setup_plot()
        
    def setup_plot(self):
        """设置图表样式"""
        self.ax.set_xlabel('Time (data points)')
        self.ax.set_ylabel('Angle (degrees)')
        self.ax.set_title('PSI手套实时角度数据')
        self.ax.grid(True, alpha=0.3)
        joint_num = len(SELECTED_ANGLES)
        
        # 创建joint_num条线，使用不同颜色
        colors = plt.cm.tab20(np.linspace(0, 1, joint_num))
        for i in range(joint_num):
            line, = self.ax.plot([], [], color=colors[i], linewidth=1.5, 
                               label=f'{SELECTED_ANGLES_NAMES[i]}', alpha=0.8)
            self.lines.append(line)
        
        # 添加图例
        handles = [self.lines[i] for i in range(joint_num)]
        labels = [f'{SELECTED_ANGLES_NAMES[i]}' for i in range(joint_num)]
        self.ax.legend(handles, labels, loc='upper right', ncol=2, fontsize=8)
        
        # 设置初始y轴范围
        self.ax.set_ylim(-100, 500)
        
        # 启用交互模式
        plt.ion()
        plt.show()
        
    def update_y_range(self, angles):
        """更新y轴范围以适应数据"""
        if angles is None:
            return
            
        # 更新全局最小值和最大值
        current_min = min(angles)
        current_max = max(angles)
        
        if current_min < self.y_min:
            self.y_min = current_min
        if current_max > self.y_max:
            self.y_max = current_max
        
        # 计算边距
        y_range = self.y_max - self.y_min
        if y_range > 0:
            margin = y_range * self.y_margin
            new_y_min = self.y_min - margin
            new_y_max = self.y_max + margin
        else:
            # 如果所有值相同，设置一个固定范围
            new_y_min = self.y_min - 10
            new_y_max = self.y_max + 10
        
        # 更新y轴范围
        self.ax.set_ylim(new_y_min, new_y_max)
        
    def reset_y_range(self):
        """重置y轴范围"""
        self.y_min = float('inf')
        self.y_max = float('-inf')
        self.ax.set_ylim(-100, 500)  # 重置为默认范围
        
    def update_plot(self, angles):
        """更新图表数据"""
        if angles is None:
            return
        
        if len(angles) != len(SELECTED_ANGLES):
            print(f"❌ 读取角度数量不正确: {len(angles)} != {len(SELECTED_ANGLES)}")
            return
            
        # 更新y轴范围
        self.update_y_range(angles)
            
        # 添加新数据点
        for i, angle in enumerate(angles):
            self.data_queues[i].append(angle)
        
        # 更新所有线条
        for i, line in enumerate(self.lines):
            if len(self.data_queues[i]) > 0:
                x_data = list(range(len(self.data_queues[i])))
                y_data = list(self.data_queues[i])
                line.set_data(x_data, y_data)
        
        # 自动调整x轴范围
        max_len = max(len(q) for q in self.data_queues)
        if max_len > 0:
            self.ax.set_xlim(0, max_len)
        
        # 刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def update_title(self, frequency=None, success_rate=None):
        """更新图表标题，显示频率和成功率信息"""
        title = 'PSI手套实时角度数据'
        if frequency is not None:
            title += f' | 频率: {frequency:.1f}Hz'
        if success_rate is not None:
            title += f' | 成功率: {success_rate:.1f}%'
        
        # 添加当前y轴范围信息
        if self.y_min != float('inf') and self.y_max != float('-inf'):
            title += f' | 范围: [{self.y_min:.1f}°, {self.y_max:.1f}°]'
        
        self.ax.set_title(title)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def close(self):
        """关闭图表"""
        plt.close(self.fig)


class RealTimeAnglePlotter:
    """实时角度折线图绘制器 - 一个窗口多个子图（每个关节一个子图）"""
    
    def __init__(self, max_points=100):
        if not ENABLE_PLOT:
            raise ImportError("matplotlib未安装，无法使用绘图功能")
            
        self.max_points = max_points
        self.joint_num = len(SELECTED_ANGLES)
        
        # 为每个关节创建独立的子图
        self.fig = None
        self.axes = []
        self.lines = []
        self.data_queues = []
        
        # 为每个关节添加动态缩放相关属性
        self.y_mins = []
        self.y_maxs = []
        self.y_margin = 0.1  # y轴边距比例
        
        # 初始化每个关节的数据队列和缩放参数
        for i in range(self.joint_num):
            self.data_queues.append(deque(maxlen=max_points))
            self.y_mins.append(float('inf'))
            self.y_maxs.append(float('-inf'))
        
        # 设置图表
        self.setup_plots()
        
    def setup_plots(self):
        """在一个窗口中为每个关节设置独立的子图"""
        # 启用交互模式
        plt.ion()
        
        # 计算子图布局：尽量形成矩形排列
        import math
        cols = math.ceil(math.sqrt(self.joint_num))
        rows = math.ceil(self.joint_num / cols)
        
        # 创建一个包含多个子图的窗口
        self.fig, axes = plt.subplots(rows, cols, figsize=(4*cols, 3*rows))
        
        # 如果只有一个子图，axes不是列表
        if self.joint_num == 1:
            axes = [axes]
        elif rows == 1 or cols == 1:
            axes = axes.flatten()
        else:
            axes = axes.flatten()
        
        # 为每个关节设置子图
        for i in range(self.joint_num):
            ax = axes[i]
            
            # 设置图表样式
            ax.set_xlabel('Time (data points)')
            ax.set_ylabel('Angle (degrees)')
            ax.set_title(f'{SELECTED_ANGLES_NAMES[i]} (索引{SELECTED_ANGLES[i]})')
            ax.grid(True, alpha=0.3)
            
            # 创建一条线
            line, = ax.plot([], [], color='blue', linewidth=2, alpha=0.8)
            
            # 设置初始y轴范围
            ax.set_ylim(-100, 500)
            
            # 保存图表对象
            self.axes.append(ax)
            self.lines.append(line)
        
        # 隐藏多余的子图
        for i in range(self.joint_num, len(axes)):
            axes[i].set_visible(False)
        
        # 调整布局
        self.fig.tight_layout()
        
        # 显示窗口
        plt.show()
        
    def update_y_range(self, angles):
        """为每个关节独立更新y轴范围"""
        if angles is None:
            return
            
        # 为每个关节独立更新y轴范围
        for i, angle in enumerate(angles):
            # 更新该关节的最小值和最大值
            if angle < self.y_mins[i]:
                self.y_mins[i] = angle
            if angle > self.y_maxs[i]:
                self.y_maxs[i] = angle
            
            # 计算边距
            y_range = self.y_maxs[i] - self.y_mins[i]
            if y_range > 0:
                margin = y_range * self.y_margin
                new_y_min = self.y_mins[i] - margin
                new_y_max = self.y_maxs[i] + margin
            else:
                # 如果所有值相同，设置一个固定范围
                new_y_min = self.y_mins[i] - 10
                new_y_max = self.y_maxs[i] + 10
            
            # 更新该关节的y轴范围
            self.axes[i].set_ylim(new_y_min, new_y_max)
        
    def reset_y_range(self):
        """重置所有关节的y轴范围"""
        for i in range(self.joint_num):
            self.y_mins[i] = float('inf')
            self.y_maxs[i] = float('-inf')
            self.axes[i].set_ylim(-100, 500)  # 重置为默认范围
        
    def update_plot(self, angles):
        """更新所有关节的图表数据"""
        if angles is None:
            return
        
        if len(angles) != len(SELECTED_ANGLES):
            print(f"❌ 读取角度数量不正确: {len(angles)} != {len(SELECTED_ANGLES)}")
            return
            
        # 更新y轴范围
        self.update_y_range(angles)
            
        # 为每个关节添加新数据点并更新图表
        for i, angle in enumerate(angles):
            # 添加新数据点
            self.data_queues[i].append(angle)
            
            # 更新该关节的线条
            if len(self.data_queues[i]) > 0:
                x_data = list(range(len(self.data_queues[i])))
                y_data = list(self.data_queues[i])
                self.lines[i].set_data(x_data, y_data)
                
                # 自动调整该关节的x轴范围
                if len(self.data_queues[i]) > 0:
                    self.axes[i].set_xlim(0, len(self.data_queues[i]))
        
        # 刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def update_title(self, frequency=None, success_rate=None):
        """更新所有关节图表的标题，显示频率和成功率信息"""
        for i in range(self.joint_num):
            title = f'{SELECTED_ANGLES_NAMES[i]} (索引{SELECTED_ANGLES[i]})'
            if frequency is not None:
                title += f' | {frequency:.1f}Hz'
            if success_rate is not None:
                title += f' | {success_rate:.1f}%'
            
            # 添加当前该关节的y轴范围信息
            if self.y_mins[i] != float('inf') and self.y_maxs[i] != float('-inf'):
                title += f' | [{self.y_mins[i]:.1f}°, {self.y_maxs[i]:.1f}°]'
            
            self.axes[i].set_title(title)
        
        # 刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def close(self):
        """关闭图表窗口"""
        plt.close(self.fig)


def run_visualization_test(reader, plotter):
    """运行可视化测试"""
    last_stats_time = time.time()
    last_print_time = time.time()
    read_count = 0
    success_count = 0
    latest_angles = None
    timing_samples = []

    print("📊 已启用实时绘图")
    print("按Ctrl+C停止")
    print()
    
    while True:
        current_time = time.time()
        

        # 普通模式：只获取角度数据
        all_angles = reader.read_angles()
        if all_angles is None:
            continue
        angles = np.asarray(all_angles)[SELECTED_ANGLES] if all_angles else None
        angles = [angle * SELECTED_DIRECT[i] for i, angle in enumerate(angles)]
        timings = {}
        
        read_count += 1
        if IF_NORMALIZE:
            angles = [(angle - ANGLES_MIN[i]) / (ANGLES_MAX[i] - ANGLES_MIN[i]) for i, angle in enumerate(angles)]
            # clip to 0-1
            angles = [max(0, min(1, angle)) for angle in angles]
        
        if angles is not None and len(angles) == len(SELECTED_ANGLES):
            success_count += 1
            latest_angles = angles
            
            # 在普通模式或调试模式的定期打印中显示角度数据
            if (current_time - last_print_time >= 0.5):
                print(f"\n[{time.strftime('%H:%M:%S')}] 角度数据:")
                for i, angle in enumerate(angles):
                    joint_name = SELECTED_ANGLES_NAMES[i]
                    joint_index = SELECTED_ANGLES[i]
                    print(f"  {joint_name} (索引{joint_index}): {angle:8.2f}")
                # print(f"  范围: [{min(angles):6.2f}, {max(angles):6.2f}]")
            
            plotter.update_plot(angles)
        else:
            plotter.update_plot(None)
        
        # 每5秒输出统计信息和更新图表标题
        if current_time - last_stats_time >= 5.0:
            elapsed_time = current_time - last_stats_time
            avg_frequency = read_count / elapsed_time
            success_rate = (success_count / read_count * 100) if read_count > 0 else 0
            
            # 更新图表标题
            plotter.update_title(avg_frequency, success_rate)
            
            # 重置计数器
            last_stats_time = current_time
            read_count = 0
            success_count = 0
            timing_samples = []
        
        # 控制读取频率
        time.sleep(0.01)  # 约100Hz


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='PSI手套角度可视化测试器')
    parser.add_argument('--debug', action='store_true', default=False,
                       help='启用调试模式，显示详细统计信息')
    parser.add_argument('--port', action='store', default="/dev/ttyACM0",
                       help='串口端口')
    parser.add_argument('--max-points', type=int, default=20,
                       help='图表显示的最大数据点数量')
    parser.add_argument('--multi-window', action='store_true', default=True,
                       help='启用多窗口模式（一个窗口多个子图）')
    parser.add_argument('--single-window', action='store_true', default=False,
                       help='使用单窗口模式（所有关节在一个图表）')
                       
    args = parser.parse_args()
    
    # 如果用户指定了single-window，则禁用multi-window
    if args.single_window:
        args.multi_window = False
    
    if not ENABLE_PLOT:
        print("❌ matplotlib未安装，无法运行可视化测试")
        print("请安装matplotlib: pip install matplotlib")
        return
    
    # 创建读取器和绘图器
    print(f"🎯 使用串口: {args.port}")
    reader = SimpleGloveReader(port=args.port)
    
    while True:
        if reader.read_angles() is not None:
            break
        print(f"[{time.strftime('%Y%m%d_%H%M%S')}]Only got None for now, waiting for glove to connect...")
        time.sleep(0.1)

    if args.multi_window:
        plotter = RealTimeAnglePlotter(max_points=args.max_points)
        print("🎯 启用多窗口模式 - 一个窗口多个子图")
    else:
        plotter = SingleWindowAnglePlotter(max_points=args.max_points)
        print("🎯 启用单窗口模式 - 所有关节在一个图表")
    
    try:
        run_visualization_test(reader, plotter)
            
    except KeyboardInterrupt:
        print("\n程序停止")
    finally:
        reader.close()
        plotter.close()


if __name__ == "__main__":
    main()

""" 
使用示例:
    # 基本可视化测试（默认多窗口模式：一个窗口多个子图）
    python glove_visualization_test.py --port /dev/ttyUSB2
    
    # 单窗口模式（所有关节在一个图表中）
    python glove_visualization_test.py --port /dev/ttyUSB2 --single-window
    
    # 调试模式可视化测试（显示详细统计和时间分析）
    python glove_visualization_test.py --port /dev/ttyUSB3 --debug
    
    # 自定义显示点数
    python glove_visualization_test.py --port /dev/ttyUSB2 --max-points 500

新功能:
    - 支持两种显示模式：
      * 多窗口模式（默认）：在一个窗口中显示多个子图，每个关节一个子图
      * 单窗口模式：所有关节在一个图表中显示（多条线）
    - 每个角度值单独打印，显示关节名称、索引和角度值
    - 图表y轴根据数据范围动态缩放
    - 多窗口模式下子图自动排列为矩形布局
    - 在图表标题中显示频率、成功率和数据范围
    - 实时显示角度数据的最小值和最大值
""" 