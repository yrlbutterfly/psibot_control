from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException
import time
import numpy as np
import threading


class Hand:
	def __init__(self, port='/dev/ttyUSB1', node_id=2, update_rate=100):
		"""
		初始化ROH机械手

		Args:
			port: 串口号
			node_id: 设备ID，默认为2
			update_rate: 角度更新频率(Hz)，默认100Hz
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
		self.update_rate = update_rate
		self.finger_names = {
			0: "拇指",
			1: "食指",
			2: "中指",
			3: "无名指",
			4: "小指",
			5: "拇指旋转"
		}

		self.lower_limit = np.array([226, 10023, 9782, 10138, 9885, 0])
		self.upper_limit = np.array([3676, 17832, 17601, 17652, 17484, 8997])

		# 线程控制
		self._running = False
		self._angle_thread = None
		self._current_thread = None
		self._action_thread = None
		self._angle_lock = threading.Lock()
		self._current_lock = threading.Lock()
		self._action_lock = threading.Lock()
		
		# 角度缓存
		self._init_angle = [0.5]*6
		self._current_angles = [0.5] * 6
		self._last_valid_angles = [0.5] * 6
		self._last_angle_update_time = 0
		
		# 电流缓存
		self._current_finger_currents = [0.0] * 6
		self._last_valid_currents = [0.0] * 6
		self._last_current_update_time = 0
		
		# 动作控制
		self._action_last = [0.5] * 6  # 当前目标动作
		self._action_last_time = None
		self._action_execution_rate = update_rate  # 动作执行频率(Hz)

		# 连接设备
		if not self.client.connect():
			raise ConnectionError("无法连接到机械手")

		# 等待设备就绪
		time.sleep(1)

		# 初始化机械手
		self._init_hand()



		self._write_register(1095, 200)
		self._write_register(1096, 200)
		self._write_register(1097, 200)
		self._write_register(1098, 200)
		self._write_register(1099, 200)
		self._write_register(1100, 200)
		
		# 连接后读取手指电流
		self.get_finger_currents()

		# 启动角度更新线程
		self._start_angle_update_thread()
		
		# 启动电流更新线程
		self._start_current_update_thread()
		
		# 启动动作执行线程
		self._start_action_execution_thread()


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




	def _start_angle_update_thread(self):
		"""启动角度更新线程"""
		self._running = True
		self._angle_thread = threading.Thread(target=self._angle_update_loop, daemon=True)
		self._angle_thread.start()
		print(f"角度更新线程已启动，频率: {self.update_rate}Hz")

	def _angle_update_loop(self):
		"""角度更新循环, 最大频率更新"""
		while self._running:
			try:
				# 读取当前角度
				angles = self._read_angles_from_hardware()
				if angles is not None:
					with self._angle_lock:
						self._current_angles = angles
						self._last_valid_angles = angles[:]
						self._last_angle_update_time = time.time()
				
			except Exception as e:
				print(f"角度更新线程错误: {str(e)}")

	def _read_angles_from_hardware(self):
		"""从硬件读取角度（线程安全）， dt大约为0.016s"""
		try:
			# 尝试批量读取
			current_positions = self._batch_read_registers(1165, 6)
			if current_positions is None:
				# 如果批量读取失败，尝试逐个读取
				current_positions = []
				for i in range(6):
					result = self._read_register(1165 + i)
					if result is None:
						return None
					current_positions.extend(result)
			
			# 计算角度
			current_positions = np.array(current_positions)
			angles = (current_positions - self.lower_limit) / (self.upper_limit - self.lower_limit)
			angles = np.clip(angles, 0, 1)
			
			return angles.tolist()
			
		except Exception as e:
			print(f"硬件读取角度失败: {str(e)}")
			return None

	def get_angles_non_block(self):
		"""
		获取当前角度（从缓存读取，线程安全）
		
		Returns:
			list: 6个手指的当前角度列表 [0-1]
		"""
		with self._angle_lock:
			# 检查数据是否过期（超过1秒没有更新）
			delay = time.time() - self._last_angle_update_time
			if delay > 1.0:
				print(f"警告: 角度数据可能过期, delay：{delay*1000:.4f}ms")
				return self._last_valid_angles
			
			return self._current_angles[:] 

	def get_angles(self):
		"""
		同时读取所有手指的当前角度，带错误处理
		"""
		try:
			# 尝试批量读取
			current_positions = self._batch_read_registers(1165, 6)
			if current_positions is None:
				# 如果批量读取失败，尝试逐个读取
				print("批量读取失败，尝试逐个读取")
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
		



	def _start_current_update_thread(self):
		"""启动电流更新线程"""
		self._current_thread = threading.Thread(target=self._current_update_loop, daemon=True)
		self._current_thread.start()
		print(f"电流更新线程已启动，频率: {self.update_rate}Hz")
	
	def get_finger_currents(self, verbose=False):
		"""
		读取所有手指的电流值
		寄存器地址：1105-1110 (ROH_FINGER_CURRENT0-5)
		"""
		try:
			if verbose:
				print("📊 读取手指电流值...")
			
			# 批量读取手指电流
			currents = self._batch_read_registers(1105, 6)
			if currents is None:
				print("❌ 读取手指电流失败")
				return None
			
			# 打印电流值
			finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指旋转"]
			if verbose:
				print("🔌 手指电流值:")
				for i, (name, current) in enumerate(zip(finger_names, currents)):
					print(f"  {name}(寄存器{1105+i}): {current}")
			
			return currents
			
		except Exception as e:
			print(f"❌ 读取手指电流时发生错误: {str(e)}")
			return None

	def get_finger_currents_non_block(self):
		"""
		获取当前电流（从缓存读取，线程安全）
		
		Returns:
			list: 6个手指的当前电流值列表
		"""
		with self._current_lock:
			# 检查数据是否过期（超过1秒没有更新）
			if time.time() - self._last_current_update_time > 1.0:
				print("警告: 电流数据可能过期")
				return self._last_valid_currents
			
			return self._current_finger_currents[:]  # 返回副本

	def _current_update_loop(self):
		"""电流更新循环, 最大频率更新"""
		while self._running:
			try:
				# 读取当前电流
				currents = self._read_currents_from_hardware()
				if currents is not None:
					with self._current_lock:
						self._current_finger_currents = currents
						self._last_valid_currents = currents[:]
						self._last_current_update_time = time.time()
				
			except Exception as e:
				print(f"电流更新线程错误: {str(e)}")

	def _read_currents_from_hardware(self):
		"""从硬件读取电流（线程安全）"""
		try:
			# 批量读取手指电流
			currents = self._batch_read_registers(1105, 6)
			if currents is None:
				return None
			
			# 确保数据是有效的数值
			safe_currents = []
			for current in currents:
				if current is None:
					safe_currents.append(0.0)
				else:
					try:
						safe_currents.append(float(current))
					except (ValueError, TypeError):
						safe_currents.append(0.0)
			
			return safe_currents
			
		except Exception as e:
			print(f"硬件读取电流失败: {str(e)}")
			return None




	def _start_action_execution_thread(self):
		"""启动动作执行线程"""
		self._action_thread = threading.Thread(target=self._action_execution_loop, daemon=True)
		self._action_thread.start()
		print(f"动作执行线程已启动，频率: {self.update_rate}Hz")
	
	def set_angles(self, angles):
		"""
		同时设置多个手指的角度, block 模式

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
	
	def _action_execution_loop(self):
		"""动作执行循环"""
		execution_interval = 1.0 / self._action_execution_rate
		
		while self._running:
			try:
				start_time = time.time()

				if self._action_last_time is None:
					continue
				else:
					delay_time = time.time() - self._action_last_time
					if delay_time > 0.1:
						# print(f"动作过期，跳过执行，延迟时间: {delay_time*1000:.2f}ms")
						continue

				# 执行动作
				self._execute_action()
				
				# 控制执行频率
				elapsed = time.time() - start_time
				if elapsed < execution_interval:
					time.sleep(execution_interval - elapsed)
					# print(f"动作执行时间: {elapsed*1000:.2f}ms")
				else:
					print(f"动作执行时间大于dt/Hz: {elapsed*1000:.2f}ms")
				
			except Exception as e:
				print(f"动作执行线程错误: {str(e)}")
				time.sleep(0.01)  # 短暂等待后继续

	def _execute_action(self):
		"""执行动作"""
		self.set_angles(self._action_last)

	def set_angles_non_block(self, angles):
		"""
		非阻塞设置角度（添加到动作队列）
		
		Args:
			angles: 长度为6的列表，包含所有手指的目标角度 (0-1的百分比)
				[拇指, 食指, 中指, 无名指, 小指, 拇指旋转]
		"""
		if len(angles) != 6:
			raise ValueError("angles必须是长度为6的列表")
		
		# 验证角度范围
		angle = np.clip(angles, 0.0, 1.0)
		# angles = [max(0.0, min(1.0, float(angle))) for angle in angles]
		
		with self._action_lock:
			# 更新当前目标动作
			self._action_last = angles[:]
			self._action_last_time = time.time()
	


	def _init_hand(self):
		"""初始化机械手"""
		try:
			self.set_angles([0.9]*6)
			time.sleep(1)
			self.set_angles(self._init_angle)
			time.sleep(2)
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



	def close(self):
		"""关闭连接"""
		# 停止所有更新线程
		if self._running:
			self._running = False
			
			# 等待角度更新线程结束
			if self._angle_thread and self._angle_thread.is_alive():
				self._angle_thread.join(timeout=1.0)
				if self._angle_thread.is_alive():
					print("警告: 角度更新线程未能正常结束")
				else:
					print("角度更新线程已停止")
			
			# 等待电流更新线程结束
			if self._current_thread and self._current_thread.is_alive():
				self._current_thread.join(timeout=1.0)
				if self._current_thread.is_alive():
					print("警告: 电流更新线程未能正常结束")
				else:
					print("电流更新线程已停止")
			
			# 等待动作执行线程结束
			if self._action_thread and self._action_thread.is_alive():
				self._action_thread.join(timeout=1.0)
				if self._action_thread.is_alive():
					print("警告: 动作执行线程未能正常结束")
				else:
					print("动作执行线程已停止")
		
		# 关闭Modbus连接
		if hasattr(self, 'client'):
			self.client.close()

	def __del__(self):
		"""析构函数，确保关闭连接"""
		self.close()



def reading_test():
	"""测试线程化角度和电流读取功能"""
	hand = Hand(port='/dev/ttyUSB1', update_rate=100)
	
	print("开始测试线程化角度和电流读取...")
	print("按 Ctrl+C 停止测试")
	
	try:
		for i in range(100):  # 测试100次
			# 获取角度（从缓存读取，很快）
			t0 = time.time()
			current_angles = hand.get_angles_non_block()
			t1 = time.time()
			print(f"角度读取时间: {(t1-t0)*1000:.2f}ms (缓存) 延迟：{(time.time()-hand._last_angle_update_time)*1000:.2f}ms")
			
			# 获取电流（从缓存读取，很快）
			t2 = time.time()
			current_finger_currents = hand.get_finger_currents_non_block()
			t3 = time.time()
			print(f"电流读取时间: {(t3-t2)*1000:.2f}ms (缓存) 延迟：{(time.time()-hand._last_current_update_time)*1000:.2f}ms")
			
			# 对比：直接从硬件读取
			_ = hand.get_angles()
			t4 = time.time()
			print(f"角度读取时间: {(t4-t3)*1000:.2f}ms (硬件)")
			
			_ = hand.get_finger_currents(verbose=False)
			t5 = time.time()
			print(f"电流读取时间: {(t5-t4)*1000:.2f}ms (硬件)")
			
			print(f"\n--- 测试 {i+1} ---")
			print(f"当前角度: {[f'{a:.3f}' for a in current_angles]}")
			print(f"手指电流: {current_finger_currents}")
			
			time.sleep(0.1)  # 100ms间隔
			
	except KeyboardInterrupt:
		print("\n测试被用户中断")
	finally:
		hand.close()
		print("测试结束")

def moving_test():
	"""测试运动过程中的线程化角度和电流读取"""
	try:
		hand = Hand(port='/dev/ttyUSB1', update_rate=100)
		
		print("开始运动测试（使用线程化角度和电流读取）...")
		print("按 Ctrl+C 停止测试")
		
		# 设置初始位置
		hand.set_angles([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
		time.sleep(1)
		
		steps = 50
		min_value = 0.3
		max_value = 0.8
		
		while True:  # 无限循环
			# 使用正弦函数生成往复运动
			for i in range(steps):
				# 使用正弦函数生成0到1之间的值，然后映射到min_value-max_value区间
				t = i / steps * 2 * np.pi  # 0 到 2π
				current_value = min_value + (max_value - min_value) * (np.sin(t) + 1) / 2
				
				# 前5个手指设置为当前值，最后一个保持为0.5
				angles = [current_value] * 5 + [0.5]
				
				# 设置角度
				t0 = time.time()
				hand.set_angles(angles)
				t1 = time.time()
				print(f"设置角度时间: {(t1-t0)*1000:.2f}ms")
				
				# 快速读取当前角度和电流（从缓存）
				current_angles = hand.get_angles_non_block()
				current_currents = hand.get_finger_currents_non_block()
				
				print(f"目标: {current_value:.3f}, 实际: {[f'{a:.3f}' for a in current_angles[:5]]}, 电流: {[f'{c:.1f}' for c in current_currents[:5]]}")
				
				time.sleep(0.02)  # 50Hz控制频率
				
	except KeyboardInterrupt:
		print("\n运动测试被用户中断")
	except Exception as e:
		print(f"运动测试错误: {str(e)}")
	finally:
		hand.close()
		print("运动测试结束")

def moving_test_non_block():
	"""测试运动过程中的线程化角度和电流读取"""
	try:
		hand = Hand(port='/dev/ttyUSB1', update_rate=100)
		
		print("开始测试非阻塞角度设置...")
		print("按 Ctrl+C 停止测试")
		
		# 设置初始位置
		hand.set_angles([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
		time.sleep(2)
		
		steps = 50
		min_value = 0.3
		max_value = 0.8
		
		while True:  # 无限循环
			# 使用正弦函数生成往复运动
			for i in range(steps):
				# 使用正弦函数生成0到1之间的值，然后映射到min_value-max_value区间
				t = i / steps * 2 * np.pi  # 0 到 2π
				current_value = min_value + (max_value - min_value) * (np.sin(t) + 1) / 2
				
				# 前5个手指设置为当前值，最后一个保持为0.5
				angles = [current_value] * 5 + [0.5]
				
				# 设置角度
				t0 = time.time()
				hand.set_angles_non_block(angles)
				t1 = time.time()
				print(f"设置角度时间: {(t1-t0)*1000:.2f}ms")
				
				# 快速读取当前角度和电流（从缓存）
				t2 = time.time()
				current_angles = hand.get_angles_non_block()
				t3 = time.time()
				print(f"角度读取时间: {(t3-t2)*1000:.2f}ms")
				current_currents = hand.get_finger_currents_non_block()
				t4 = time.time()
				print(f"电流读取时间: {(t4-t3)*1000:.2f}ms")
				
				print(f"目标: {current_value:.3f}, 实际: {[f'{a:.3f}' for a in current_angles[:5]]}, 电流: {[f'{c:.1f}' for c in current_currents[:5]]}")
				
				time.sleep(0.1)  # 10Hz action 控制频率
				
	except KeyboardInterrupt:
		print("\n运动测试被用户中断")
	except Exception as e:
		print(f"运动测试错误: {str(e)}")
	finally:
		hand.close()
		print("运动测试结束")

if __name__ == "__main__":
	print("选择测试模式:")
	print("1. 角度读取测试 (reading_test)")
	print("2. 动作测试 (moving_test)")
	print("3. 动作测试 (moving_test_non_block)")
	
	choice = input("请输入选择 (1, 2 或 3): ").strip()
	
	if choice == "1":
		reading_test()
	elif choice == "2":
		moving_test()
	elif choice == "3":
		moving_test_non_block()
	else:
		print("无效选择")

"""
The motion of hand currently is not time precise, not smooth, not accurate, not repeatable.
使用正弦函数生成往复运动经常会卡住， 手指在动作的时候会颤抖。
TODO: (Maybe)考虑对手指的joint pos进行平滑，毕竟是位置控制.
"""


"""
reading_test():
--- 测试 10 ---
当前角度: ['0.676', '0.676', '0.677', '0.677', '0.676', '0.500']
手指电流: [4.0, 0.0, 3.0, 2.0, 3.0, 1.0]
角度读取时间: 0.01ms (缓存) 延迟：20.07ms
电流读取时间: 0.00ms (缓存) 延迟：4.15ms
角度读取时间: 43.87ms (硬件)
电流读取时间: 47.93ms (硬件)

--- 测试 11 ---
当前角度: ['0.676', '0.676', '0.677', '0.677', '0.676', '0.500']
手指电流: [4.0, 0.0, 3.0, 3.0, 3.0, 1.0]
角度读取时间: 0.01ms (缓存) 延迟：4.17ms
电流读取时间: 0.00ms (缓存) 延迟：20.17ms
角度读取时间: 43.88ms (硬件)
电流读取时间: 47.87ms (硬件)
"""