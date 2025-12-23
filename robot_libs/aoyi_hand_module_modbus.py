from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException
import time
import numpy as np

class Hand:
    def __init__(self, port='/dev/ttyUSB1', node_id=2):
        """
        初始化ROH机械手

        Args:
            port: 串口号
            node_id: 设备ID，默认为2
        """
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
        )
        self.node_id = node_id
        self.finger_names = {
            0: "拇指",
            1: "食指",
            2: "中指",
            3: "无名指",
            4: "小指",
            5: "拇指旋转"
        }

        # 连接设备
        if not self.client.connect():
            raise ConnectionError("无法连接到机械手")
        # 等待设备就绪
        time.sleep(1)

        # 初始化机械手
        # self._init_hand()

        self.lower_limit = [226, 10023, 9782, 10138, 9885, 0]
        self.upper_limit = [3676, 17832, 17601, 17652, 17484, 8997]

        self.lower_limit = np.array(self.lower_limit)
        self.upper_limit = np.array(self.upper_limit)

        self._write_register(1095, 200)
        self._write_register(1096, 200)
        self._write_register(1097, 200)
        self._write_register(1098, 200)
        self._write_register(1099, 200)
        self._write_register(1100, 200)

    def _read_register(self, address, count=1):
        """
        安全地读取寄存器

        Args:
            address: 寄存器地址
            count: 要读取的寄存器数量

        Returns:
            读取到的值列表，失败返回None
        """
        try:
            result = self.client.read_holding_registers(address, count, unit=self.node_id)
            if result.isError():
                print(f"读取寄存器{address}失败")
                return None
            return result.registers
        except ModbusIOException as e:
            print(f"ModBus通信错误: {str(e)}")
            return None
        except Exception as e:
            print(f"读取寄存器时发生错误: {str(e)}")
            return None

    def _write_register(self, address, value):
        """
        安全地写入寄存器

        Args:
            address: 寄存器地址
            value: 要写入的值

        Returns:
            是否成功
        """
        try:
            result = self.client.write_register(address, value, unit=self.node_id)
            if result.isError():
                print(f"写入寄存器{address}失败")
                return False
            return True
        except ModbusIOException as e:
            print(f"ModBus通信错误: {str(e)}")
            return False
        except Exception as e:
            print(f"写入寄存器时发生错误: {str(e)}")
            return False

    def _batch_read_registers(self, start_address, count, max_retries=3):
        """
        带重试机制的批量读取寄存器
        
        Args:
            start_address: 起始寄存器地址
            count: 要读取的寄存器数量
            max_retries: 最大重试次数
        """
        for attempt in range(max_retries):
            try:
                result = self.client.read_holding_registers(start_address, count, unit=self.node_id)
                if not result.isError():
                    return result.registers
                
                # 如果失败，打印详细信息并重试
                print(f"批量读取尝试 {attempt + 1}/{max_retries} 失败")
                print(f"起始地址: {start_address}, 数量: {count}")
                print(f"错误信息: {result}")
                
                if attempt < max_retries - 1:
                    time.sleep(0.01 * (attempt + 1))  # 递增延迟
                    continue
                    
            except Exception as e:
                print(f"ModBus通信错误 (尝试 {attempt + 1}/{max_retries}): {str(e)}")
                if attempt < max_retries - 1:
                    time.sleep(0.01 * (attempt + 1))
                    continue
        
        # 如果所有重试都失败，尝试单个读取
        print("批量读取失败，切换到单个读取模式")
        try:
            registers = []
            for addr in range(start_address, start_address + count):
                result = self._read_register(addr)
                if result is None:
                    return None
                registers.extend(result)
            return registers
        except Exception as e:
            print(f"单个读取也失败: {str(e)}")
            return None

    def _batch_write_registers(self, start_address, values):
        """
        批量写入连续的寄存器

        Args:
            start_address: 起始寄存器地址
            values: 要写入的值列表

        Returns:
            是否成功
        """
        try:
            result = self.client.write_registers(start_address, values, unit=self.node_id)
            if result.isError():
                print(f"批量写入寄存器失败，起始地址: {start_address}")
                return False
            return True
        except ModbusIOException as e:
            print(f"ModBus通信错误: {str(e)}")
            return False
        except Exception as e:
            print(f"写入寄存器时发生错误: {str(e)}")
            return False

    def _init_hand(self):
        """初始化机械手"""
        try:
            # 设置自检级别为1(允许开机归零)
            if not self._write_register(1008, 1):
                raise RuntimeError("设置自检级别失败")

            # 开始初始化
            if not self._write_register(1013, 1):
                raise RuntimeError("开始初始化失败")

            self._write_register(1013, 1)

            # 等待初始化完成
            retry_count = 0
            while retry_count < 30:  # 最多等待30秒
                error = self._check_error()
                if error is None:
                    time.sleep(1)
                elif error == 0:  # 初始化完成
                    print("初始化finish")
                    break
                elif error != 1:  # 不是等待初始化状态
                    raise RuntimeError(f"初始化过程出错，错误代码: {error}")
                time.sleep(1)
                retry_count += 1

            if retry_count >= 30:
                raise RuntimeError("初始化超时")

        except Exception as e:
            raise RuntimeError(f"初始化失败: {str(e)}")

    def _check_error(self):
        """
        检查错误状态

        Returns:
            错误代码，None表示读取失败
        """
        result = self._read_register(1006)
        if result is None:
            return None

        error = result[0]
        if error:
            error_dict = {
                1: "等待初始化",
                2: "等待校正",
                3: "无效数据",
                4: "电机堵转",
                5: "操作失败",
                6: "保存失败"
            }
            print(f"错误: {error_dict.get(error, '未知错误')}")
        
        return error

    def set_angles(self, angles):
        """
        同时设置多个手指的角度

        Args:
            angles: 长度为6的列表，包含所有手指的目标角度 (0-100的百分比)
                [拇指, 食指, 中指, 无名指, 小指, 拇指旋转]
        """
        if len(angles) != 6:
            raise ValueError("angles必须是长度为6的列表")
        
        try:
            # 计算所有目标位置
            # 使用numpy进行批量计算
            angles = np.clip(angles, 0, 1)
            targets = (self.lower_limit + (self.upper_limit - self.lower_limit) * angles).astype(int)
            
            # 批量写入所有目标位置
            if not self._batch_write_registers(1155, targets):
                raise RuntimeError("批量写入目标位置失败")
                        
        except Exception as e:
            raise RuntimeError(f"设置角度失败: {str(e)}")

    def get_angles(self):
        """
        同时读取所有手指的当前角度，带错误处理
        """
        try:
            # 尝试批量读取
            current_positions = self._batch_read_registers(1165, 6)
            if current_positions is None:
                # 如果批量读取失败，尝试逐个读取
                current_positions = []
                for i in range(6):
                    result = self._read_register(1165 + i)
                    if result is None:
                        raise RuntimeError(f"读取手指 {i} 位置失败")
                    current_positions.extend(result)
            
            # 计算角度
            current_positions = np.array(current_positions)
            angles = (current_positions - self.lower_limit) / (self.upper_limit - self.lower_limit)
            angles = np.clip(angles, 0, 1)
            
            return angles.tolist()
            
        except Exception as e:
            print(f"读取角度失败: {str(e)}")
            # 返回上一次的有效值，或者默认值
            if hasattr(self, '_last_valid_angles'):
                print("使用上一次的有效值")
                return self._last_valid_angles
            return [0.5] * 6  # 默认中间位置

    def close(self):
        """关闭连接"""
        if hasattr(self, 'client'):
            self.client.close()

    def __del__(self):
        """析构函数，确保关闭连接"""
        self.close()

if __name__ == "__main__":
    try:
        hand = Hand(port='/dev/ttyUSB1')

        # 测试同时控制所有手指
        print("测试所有手指...")

        # 同时设置所有手指到50%位置
        hand.set_angles([1, 1, 1, 1, 1, 1.0])
        time.sleep(1)

        # 读取所有手指当前角度
        current_angles = hand.get_angles()

        steps = 10
        min_value = 0.45
        max_value = 1.0

        while True:  # 无限循环
            # 使用正弦函数生成往复运动
            for i in range(steps):
                # 使用正弦函数生成0到1之间的值，然后映射到0.5-1区间
                t = i / steps * 2 * np.pi  # 0 到 2π
                current_value = min_value + (max_value - min_value) * (np.sin(t) + 1) / 2
                
                # 前5个数设置为当前值，最后一个保持为0
                angles = [current_value] * 5 + [1.0]
                
                # 设置角度
                hand.set_angles(angles)
                time.sleep(0.01)
                
                # 读取当前角度
                current_angles = hand.get_angles()
                print(f"Current value: {current_value:.3f}")

    except Exception as e:
        print(f"错误: {str(e)}")
    finally:
        hand.close()