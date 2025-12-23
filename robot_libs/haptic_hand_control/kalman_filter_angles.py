#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手套角度数据的卡尔曼滤波器
用于对6个关节角度进行滤波
"""

import numpy as np

class KalmanFilterAngles:
    def __init__(self, num_joints=6, dt=0.01):
        """
        初始化角度卡尔曼滤波器
        num_joints: 关节数量
        dt: 时间间隔(s)
        """
        self.num_joints = num_joints
        
        # 状态向量: [angle1, angle2, ..., angle6, velocity1, velocity2, ..., velocity6]
        # 共12个状态变量
        self.x = np.zeros(2 * num_joints)
        
        # 初始化协方差矩阵
        self.P = np.eye(2 * num_joints) * 0.1
        
        # 状态转移矩阵
        self.F = np.eye(2 * num_joints)
        for i in range(num_joints):
            self.F[i, num_joints + i] = dt  # angle = angle + velocity * dt
        
        # 观测矩阵 - 只测量角度
        self.H = np.zeros((num_joints, 2 * num_joints))
        for i in range(num_joints):
            self.H[i, i] = 1.0
        
        # 过程噪声协方差
        self.Q = np.eye(2 * num_joints)
        # 角度过程噪声
        for i in range(num_joints):
            self.Q[i, i] = 1e-4
        # 速度过程噪声
        for i in range(num_joints):
            self.Q[num_joints + i, num_joints + i] = 1e-2
        
        # 测量噪声协方差
        self.R = np.eye(num_joints) * 1e-3
        
        # 单位矩阵
        self.I = np.eye(2 * num_joints)
    
    def predict(self):
        """预测步骤"""
        # 预测状态
        self.x = self.F @ self.x
        
        # 预测协方差
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x[:self.num_joints]  # 返回角度部分
    
    def update(self, angles):
        """
        更新步骤
        angles: 测量的角度数组 [angle1, angle2, ..., angle6]
        """
        # 构建测量向量
        z = np.array(angles)
        
        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态
        y = z - self.H @ self.x  # 测量残差
        self.x = self.x + K @ y
        
        # 更新协方差
        self.P = (self.I - K @ self.H) @ self.P
        
        return self.x[:self.num_joints]  # 返回滤波后的角度
    
    def filter_angles(self, angles):
        """
        一步完成预测和更新的滤波
        angles: 测量的角度数组
        """
        self.predict()
        return self.update(angles)
    
    def reset(self):
        """重置滤波器状态"""
        self.x = np.zeros(2 * self.num_joints)
        self.P = np.eye(2 * self.num_joints) * 0.1 