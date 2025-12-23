# RealMan RM75 机械臂调试指南

## 📌 环境配置步骤

### 1. 创建Conda环境

```bash
# 创建名为 psibot 的 Python 3.10 环境
conda create -n psibot python=3.10 -y

# 激活环境
conda activate psibot
```

### 2. 安装依赖包

```bash
# 进入项目目录
cd /home/psibot/Documents/psibot_control

# 安装Python依赖
pip install -r requirements.txt
```

### 3. 验证SDK库文件

检查RealMan机械臂SDK是否存在：

```bash
ls -lh robot_libs/lib/libRM_Base.so*
```

应该看到以下文件：
- `libRM_Base.so`
- `libRM_Base.so.1`
- `libRM_Base.so.1.0`
- `libRM_Base.so.1.0.0`

### 4. 配置环境变量（重要！）

将SDK库路径添加到系统库搜索路径：

```bash
# 临时配置（当前终端有效）
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/psibot/Documents/psibot_control/robot_libs/lib

# 永久配置（推荐）
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/psibot/Documents/psibot_control/robot_libs/lib' >> ~/.bashrc
source ~/.bashrc
```

## 🔌 硬件连接检查

### 机械臂网络连接

1. **检查网络连接**
   ```bash
   # 检查左臂连接
   ping 192.168.100.100 -c 3
   
   # 检查右臂连接
   ping 192.168.100.101 -c 3
   ```

2. **如果无法ping通，需要配置网络：**
   - 打开系统设置 → 网络
   - 添加/编辑有线连接
   - 设置静态IP：192.168.100.10（或同网段其他地址）
   - 子网掩码：255.255.255.0
   - 网关：192.168.100.1

### 灵巧手串口连接（如果使用）

```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 添加用户到dialout组（获得串口访问权限）
sudo usermod -a -G dialout $USER

# 重新登录后生效，或临时使用：
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

## 🧪 调试步骤

### 测试1：简单连接测试

使用最简单的测试脚本：

```bash
# 激活环境
conda activate psibot

# 配置环境变量
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/psibot/Documents/psibot_control/robot_libs/lib

# 运行测试脚本
cd /home/psibot/Documents/psibot_control
python test_arm_simple.py
```

**预期输出：**
- 连接成功提示
- 显示当前7个关节的角度（弧度和角度）
- 显示末端执行器位姿

### 测试2：读取关节角度

```bash
python get_joint.py
```

这个脚本会：
- 连接左右两个机械臂
- 读取并打印当前关节角度

### 测试3：基本运动测试

```bash
python begin.py
```

这个脚本会：
- 初始化机械臂和灵巧手
- 移动到预设的初始位置
- 测试完成后关闭连接

**⚠️ 警告：** 运行前确保机械臂周围没有障碍物！

## 🐛 常见问题排查

### 问题1：找不到 libRM_Base.so

**症状：**
```
ImportError: libRM_Base.so: cannot open shared object file
```

**解决方案：**
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/psibot/Documents/psibot_control/robot_libs/lib
```

### 问题2：无法连接机械臂

**症状：**
```
SOCKET_CONNECT_ERR
```

**检查清单：**
1. 机械臂是否上电？
2. 网线是否连接？
3. IP地址是否正确？
4. 能否ping通控制器？
5. 防火墙是否阻止连接？

**解决步骤：**
```bash
# 1. 检查网络连通性
ping 192.168.100.100

# 2. 检查网络接口配置
ip addr show

# 3. 如果需要，配置静态IP
sudo nmcli connection modify "有线连接" ipv4.addresses "192.168.100.10/24"
sudo nmcli connection modify "有线连接" ipv4.method manual
sudo nmcli connection up "有线连接"
```

### 问题3：串口权限不足

**症状：**
```
Permission denied: '/dev/ttyUSB0'
```

**解决方案：**
```bash
# 方法1：添加用户到dialout组（推荐）
sudo usermod -a -G dialout $USER
# 需要注销重新登录

# 方法2：临时修改权限
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

### 问题4：机械臂报错停止

**症状：**
机械臂运动中突然停止

**解决方案：**
```python
# 在代码中清除错误
arm.robot.Clear_System_Err()
```

或在测试脚本中会自动清除错误。

## 📊 测试流程建议

1. **第一次测试：** 只测试一个机械臂
   ```bash
   python test_arm_simple.py
   # 选择选项 2 (单臂测试)
   # 选择 1 (左臂) 或 2 (右臂)
   ```

2. **第二次测试：** 测试双臂读取
   ```bash
   python get_joint.py
   ```

3. **第三次测试：** 测试基本运动
   ```bash
   python begin.py
   ```

4. **完整测试：** 运行完整系统
   ```bash
   python control.py
   ```

## 🔧 高级配置

### 自动激活conda环境

编辑 `~/.bashrc`，添加：

```bash
# Auto activate psibot environment
alias psibot='cd /home/psibot/Documents/psibot_control && conda activate psibot && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/psibot/Documents/psibot_control/robot_libs/lib'
```

使用方法：
```bash
# 以后只需要输入
psibot
```

### 创建启动脚本

创建 `start_test.sh`：

```bash
#!/bin/bash
cd /home/psibot/Documents/psibot_control
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/robot_libs/lib
conda activate psibot
python test_arm_simple.py
```

## 📝 下一步

成功完成基本测试后，你可以：

1. 修改 `test_arm_simple.py` 测试特定的关节运动
2. 使用 `begin.py` 中的预设姿态
3. 开发自己的运动控制程序
4. 集成相机和灵巧手进行复杂操作

## 📞 需要帮助？

如果遇到问题，请提供：
1. 错误信息截图
2. 运行的命令
3. `conda list` 输出
4. 网络配置 `ip addr` 输出

