#！/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time : 2024/11/11
# @Author : zhanglf


import struct
import ctypes
import platform
import threading
# import osrf_pycommon
import serial
from time import sleep
import time
import math
import can
import sys
import select
import os


CAN_ID = 'can0'


_HERE = os.path.dirname(os.path.abspath(__file__))


# sudo modprobe peak_usb
# sudo modprobe can
# sudo modprobe can-raw
# sudo modprobe can-gw
# sudo modprobe slcan
# sudo modprobe vcan
# ls /dev/ttyACM*    
# sudo slcand -o -c -s8 /dev/ttyACM0 can0
# sudo ip link set can0 up type can bitrate 1000000
# sudo ip link set can0 txqueuelen 1000
# sudo ifconfig can0 up

# sudo ifconfig can0 txqueuelen 1000
# ip -d -s link show can0

# Example: Sending a CAN frame using socketcan on Linux

systype = platform.system()
cpuarch = platform.machine().lower()
print(f"System Type: {systype}, CPU Architecture: {cpuarch}")


# Determine the operating system and load the appropriate shared library
if systype == 'Windows':

    if struct.calcsize("P") * 8 == 64:
        dll_path = os.path.join(_HERE, "RyhandLibx64.dll")
    else:
        dll_path = os.path.join(_HERE, "RyhandLibx86.dll")
    lib = ctypes.CDLL(dll_path)

else:

    if 'arm' in cpuarch or 'aarch64' in cpuarch:
        if 'arm' in cpuarch :
            lib = ctypes.CDLL(os.path.join(_HERE, 'libRyhand_arm.so'))
        elif 'aarch64' in cpuarch:
            lib = ctypes.CDLL(os.path.join(_HERE, 'libRyhand_arm64.so'))
    else:
        # System Type: Linux, CPU Architecture: x86_64
        # 默认使用 x86 或 x86_64 架构
        if struct.calcsize("P") * 8 == 64:
            lib = ctypes.CDLL(os.path.join(_HERE, 'libRyhand64_1.so'))
        else:
            lib = ctypes.CDLL(os.path.join(_HERE, 'libRyhand.so'))




response_code = 0
retries = 1000
timeout = 0


USE_RS485 = False
USE_CANFD = False

CMD_HEADER = 0
CMD_ID = 1
CMD_LEN = 2
CMD_DATA = 3
CMD_CHECK = 4
CMD_MAX_DAT_LEN = 64

class Frame_t(ctypes.Structure):
    _fields_ = [
        ("head", ctypes.c_uint8),
        ("id", ctypes.c_uint16),
        ("len", ctypes.c_uint8),
        ("dat", ctypes.c_uint8 * CMD_MAX_DAT_LEN),
        ("check", ctypes.c_uint8)
    ]
    _pack_ = 1  # Ensure the structure is byte-aligned


# 定义全局的 ticks_var
ticks_var = ctypes.c_uint16(0)
rcvsta = CMD_HEADER
RxIndex = 0
check = 0
tcmd = Frame_t()
pdata = (ctypes.c_uint8 * ctypes.sizeof(Frame_t)).from_address(ctypes.addressof(tcmd))

if USE_RS485:
    bus = serial.Serial("COM6", 5000000, timeout=1)
else:
    if  USE_CANFD:
        bus = can.interface.Bus(channel=CAN_ID, interface='socketcan', fd=True, bitrate=1000000, data_bitrate=5000000)
    else:
        bus = can.interface.Bus(channel=CAN_ID, interface='socketcan', bitrate=1000000)



NULL = 0
ULIB_ENABLE = 1
ULIB_DISABLE = 0

OK = 0
FAIL = 1
ERROR = -1

CMD_SUCCEED = 1
CMD_FAIL = 0

def SERVO_BACK_ID(x):
    return x + 256

UPDATE_INFO = 0xa0

class enret_t:
    enServo_OK = 0
    enServo_TempratureHighW = 1
    enServo_TempratureHighE = 2
    enServo_VoltageLowE = 3
    enServo_VoltageHighE = 4
    enServo_CurrentOverE = 5
    enServo_TorqueOverE = 6
    enServo_FuseE = 7
    enServo_PwmE = 8
    enServo_DriveE = 9
    enServo_HallE = 10
    enServoStatus_Bottom = 11
    enServo_Fail = 250
    enServo_ParamErr = 251
    enServo_LibInitErr = 252
    enCanFormatError = 253
    enCanMsgSentFail = 254
    enLibHookApplyFailed = 255

class CanMsg(ctypes.Structure):
    _fields_ = [
        ("ulId", ctypes.c_uint32),      # id
        ("ucLen", ctypes.c_uint8),      # 数据长度
        ("pucDat", ctypes.c_uint8 * 64) # 数据内容
    ]



BusWrite_t = ctypes.CFUNCTYPE(ctypes.c_int8, ctypes.POINTER(CanMsg))
Callback_t = ctypes.CFUNCTYPE(None, ctypes.POINTER(CanMsg), ctypes.c_void_p)


  
class DevTimeCmd(ctypes.Structure):
    _fields_ = [
        ("ucCmd", ctypes.c_uint64, 8),    # 前面的CMD
        ("ub_year", ctypes.c_uint64, 10), # 2000年后的年份，2021 对应 21 
        ("ub_month", ctypes.c_uint64, 4),
        ("ub_day", ctypes.c_uint64, 5),
        ("ub_hour", ctypes.c_uint64, 5),
        ("ub_min", ctypes.c_uint64, 6),
        ("ub_s", ctypes.c_uint64, 6),
        ("ub_ms", ctypes.c_uint64, 10),
        ("ub_us", ctypes.c_uint64, 10)
    ]



class FingerServoInfo(ctypes.Structure):
    _fields_ = [
        ("cmd", ctypes.c_uint64, 8),    # 前面的CMD
        ("ucStatus", ctypes.c_uint64, 8),  # 故障状态 ，0 表示无故障，异常详情见 enret_t
        ("ub_P", ctypes.c_uint64, 12),  # 当前位置，0-4095 对应 0到满行程
        ("ub_V", ctypes.c_uint64, 12),  # 当前速度，-2048~2047 单位 0.001行程/s
        ("ub_I", ctypes.c_uint64, 12),  # 当前电流，-2048~2047 单位 0.01A
        ("ub_F", ctypes.c_uint64, 12)   # 当前位置，0-4095 对应手指压力传感器Adc原始值
    ]

class FingerServoCmd(ctypes.Structure):
    _fields_ = [
        ("cmd", ctypes.c_uint64, 8),    # 前面的CMD
        ("usTp", ctypes.c_uint64, 16),  # 目标位置
        ("usTv", ctypes.c_uint64, 16),  # 目标速度
        ("usTc", ctypes.c_uint64, 16),  # 目标电流
        ("reserved", ctypes.c_uint64, 8)  # 保留
    ]



class ServoData(ctypes.Union):
    _fields_ = [
        ("stuCmd", FingerServoCmd),
        ("stuInfo", FingerServoInfo),
        ("pucDat", ctypes.c_uint8 * 64)
    ]



class MsgHook(ctypes.Structure):
    _fields_ = [
        ("ucEn", ctypes.c_uint8),
        ("ucAlive", ctypes.c_uint8),
        ("pstuMsg", ctypes.POINTER(CanMsg)),
        ("funCbk", Callback_t)
    ]


class MsgListen(ctypes.Structure):
    _fields_ = [
        ("stuListen", MsgHook),
        ("stuRet", ServoData),
        ("ucConfidence", ctypes.c_uint8)
    ]


class RyCanServoBus(ctypes.Structure):
    _fields_ = [
        ("pusTicksMs", ctypes.POINTER(ctypes.c_uint16)),
        ("usTicksPeriod", ctypes.c_uint16),
        ("usHookNum", ctypes.c_uint16),
        ("usListenNum", ctypes.c_uint16),
        ("pstuHook", ctypes.POINTER(MsgHook)),
        ("pstuListen", ctypes.POINTER(MsgListen)),
        ("pfunWrite", BusWrite_t)
    ]



class Servos_t(ctypes.Structure):
    _fields_ = [
        ("raw", FingerServoInfo),
        ("p", ctypes.c_int16),
        ("v", ctypes.c_int16),
        ("t", ctypes.c_int16),
        ("f",ctypes.c_int16)
    ]
    


lib.AddHook.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.POINTER(CanMsg), ctypes.CFUNCTYPE(ctypes.c_int)]
lib.AddHook.restype = ctypes.c_int

lib.DeleteHook.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.POINTER(CanMsg)]
lib.DeleteHook.restype = ctypes.c_int

lib.AddListen.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.POINTER(CanMsg), Callback_t ]
lib.AddListen.restype = ctypes.c_int

lib.DeleteListen.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.POINTER(CanMsg)]
lib.DeleteListen.restype = ctypes.c_int

lib.GetServoUpdateInfo.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(MsgListen)]
lib.GetServoUpdateInfo.restype = ctypes.c_int8

lib.RyCanServoLibRcvMsg.argtypes = [ctypes.POINTER(RyCanServoBus), CanMsg]
lib.RyCanServoLibRcvMsg.restype = ctypes.c_int8

lib.GetRyCanServoLibVersion.argtypes = [ctypes.c_uint8 * 30]
lib.GetRyCanServoLibVersion.restype = None

lib.RyCanServoBusInit.argtypes = [ctypes.POINTER(RyCanServoBus), BusWrite_t, ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16]
lib.RyCanServoBusInit.restype = ctypes.c_uint8

lib.RyCanServoBusDeInit.argtypes = [ctypes.POINTER(RyCanServoBus)]
lib.RyCanServoBusDeInit.restype = None

lib.RyFunc_GetServoInfo.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ServoData), ctypes.c_uint16]
lib.RyFunc_GetServoInfo.restype = ctypes.c_uint8

lib.RyFunc_GetVersion.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint8 * 64, ctypes.c_uint16]
lib.RyFunc_GetVersion.restype = ctypes.c_uint8

lib.RyFunc_StartUpgrade.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
lib.RyFunc_StartUpgrade.restype = ctypes.c_uint8

lib.RyFunc_WriteUpgradeData.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyFunc_WriteUpgradeData.restype = ctypes.c_uint8

lib.RyFunc_FinishUpgrade.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyFunc_FinishUpgrade.restype = ctypes.c_uint8

lib.RyFunc_SetSNCode.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint8 * 40, ctypes.c_uint16]
lib.RyFunc_SetSNCode.restype = ctypes.c_uint8

lib.RyFunc_GetSNCode.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint8 * 40, ctypes.c_uint16]
lib.RyFunc_GetSNCode.restype = ctypes.c_uint8

lib.RyParam_SetTime.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, DevTimeCmd, ctypes.c_uint16]
lib.RyParam_SetTime.restype = ctypes.c_uint8

lib.RyParam_SetID.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyParam_SetID.restype = ctypes.c_uint8

lib.RyParam_Recover.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyParam_Recover.restype = ctypes.c_uint8

lib.RyParam_GetRigidity.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
lib.RyParam_GetRigidity.restype = ctypes.c_uint8

lib.RyParam_SetRigidity.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16]
lib.RyParam_SetRigidity.restype = ctypes.c_uint8

lib.RyParam_SetPosition.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16]
lib.RyParam_SetPosition.restype = ctypes.c_uint8

lib.RyParam_ClearFault.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint16]
lib.RyParam_ClearFault.restype = ctypes.c_uint8

lib.RyParam_GetProtectionCfg.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16]
lib.RyParam_GetProtectionCfg.restype = ctypes.c_uint8

lib.RyParam_SetProtectionCfg.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint16]
lib.RyParam_SetProtectionCfg.restype = ctypes.c_uint8

lib.RyParam_GetStroke.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyParam_GetStroke.restype = ctypes.c_uint8

lib.RyParam_SetStroke.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint16]
lib.RyParam_SetStroke.restype = ctypes.c_uint8

lib.RyParam_GetStroke_H.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyParam_GetStroke_H.restype = ctypes.c_uint8

lib.RyParam_SetStroke_H.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint16]
lib.RyParam_SetStroke_H.restype = ctypes.c_uint8

lib.RyParam_GetStroke_L.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16]
lib.RyParam_GetStroke_L.restype = ctypes.c_uint8

lib.RyParam_SetStroke_L.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint16]
lib.RyParam_SetStroke_L.restype = ctypes.c_uint8

lib.RyMotion_ServoMove_Speed.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_int16, ctypes.c_uint16, ctypes.POINTER(ServoData), ctypes.c_uint16]
lib.RyMotion_ServoMove_Speed.restype = ctypes.c_uint8

lib.RyMotion_ServoMove_Pwm.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_int16, ctypes.POINTER(ServoData), ctypes.c_uint16]
lib.RyMotion_ServoMove_Pwm.restype = ctypes.c_uint8

lib.RyMotion_CurrentMode.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_int16, ctypes.POINTER(ServoData), ctypes.c_uint16]
lib.RyMotion_CurrentMode.restype = ctypes.c_uint8

lib.RyMotion_ServoMove_Mix.argtypes = [ctypes.POINTER(RyCanServoBus), ctypes.c_uint8, ctypes.c_int16, ctypes.c_uint16, ctypes.c_uint16, ctypes.POINTER(ServoData), ctypes.c_uint16]
lib.RyMotion_ServoMove_Mix.restype = ctypes.c_uint8




servo_bus = RyCanServoBus()
# can_msg = CanMsg()
response_msg = CanMsg()

#新增测试案例函数

# 电机ID与实际角度的映射关系（单位：度）
MOTOR_ANGLE_MAPPING = {
    1: 110.0,
    2: 30.0,
    3: 84.0,
    4: 90.0,
    5: 90.0,
    6: 88.0
}

def map_value_to_angle_deg1(motor_id, raw_value):
    """将原始值转换为实际角度（基于电机ID的映射关系）"""
    max_raw = 4095  # 假设原始值范围是0-4095
    if motor_id in MOTOR_ANGLE_MAPPING:
        # 线性映射：raw_value/max_raw * 实际最大角度
        return (raw_value / max_raw) * MOTOR_ANGLE_MAPPING[motor_id]
    else:
        return 0.0  # 默认返回0度（未知电机ID）

def map_angle_deg_to_value1(motor_id, angle):
    """将角度转换为原始值（基于电机ID的映射关系）"""
    max_raw = 4095
    if motor_id in MOTOR_ANGLE_MAPPING:
        # 反向线性映射：angle/实际最大角度 * max_raw
        max_angle = MOTOR_ANGLE_MAPPING[motor_id]
        return int((angle / max_angle) * max_raw)
    else:
        return 0  # 默认返回0（未知电机ID）


def get_motor_angles(motor_id):
    """获取电机当前角度"""
    print("\n=== 获取电机角度（实际值） ===")
    # for motor_id in range(1, 7):  # ID 1-6
    servo_data = ServoData()
    ret = lib.RyFunc_GetServoInfo(ctypes.byref(servo_bus), motor_id, 
                                    ctypes.byref(servo_data), 100)
    if ret == OK:
        # 使用电机ID特定的映射关系
        angle = map_value_to_angle_deg1(motor_id, servo_data.stuInfo.ub_P)
        print(f"电机 {motor_id} 当前角度: {angle:.1f}° (标准值: {MOTOR_ANGLE_MAPPING[motor_id]}°)")
    else:
        print(f"获取电机 {motor_id} 角度失败，错误码: {ret}")

def set_speed_control_mode(motor_id,speed,target_angle):# speed = 2000  # 速度值2000 (单位: 0.001行程/s)
    """设置速度控制模式（修正版）"""
    print("\n=== 设置速度控制模式（使用实际角度） ===")
    # for motor_id in range(1, 7):
        # # 获取该电机的标准角度作为目标
        # target_angle = MOTOR_ANGLE_MAPPING[motor_id]
        # speed = 2000  # 速度值2000 (单位: 0.001行程/s)
        
    servo_data = ServoData()
    target_value = map_angle_deg_to_value1(motor_id, target_angle)
    ret = lib.RyMotion_ServoMove_Speed(
        ctypes.byref(servo_bus), motor_id, target_value, 
        speed, ctypes.byref(servo_data), 0)
        
    if ret == OK:
        print(f"电机 {motor_id} 已设为速度模式 (目标: {target_angle}°)")
    else:
        print(f"设置电机 {motor_id} 失败，错误码: {ret}")

def set_speed_current_control_mode(motor_id,speed,current,target_angle):# speed = 2000  # 速度值2000 (单位: 0.001行程/s) current = 800  # 电流值80 (单位: 0.001A)
    """设置力控模式（修正版）"""
    print("\n=== 设置速度控制模式（使用实际角度） ===")
    # for motor_id in range(1, 7):
        # # 获取该电机的标准角度作为目标
        # target_angle = MOTOR_ANGLE_MAPPING[motor_id]
        # speed = 2000  # 速度值2000 (单位: 0.001行程/s)
    # clear fault first
    lib.RyParam_ClearFault(ctypes.byref(servo_bus), motor_id, 0)    
    servo_data = ServoData()
    target_value = map_angle_deg_to_value1(motor_id, target_angle)
    ret = lib.RyMotion_ServoMove_Mix(ctypes.byref(servo_bus),motor_id,target_value,speed,current, ctypes.byref(servo_data), 0)        
    if ret == OK:
        print(f"电机 {motor_id} 已设为力控模式 (目标: {target_angle}°)")
    else:
        print(f"设置电机 {motor_id} 失败，错误码: {ret}")

def set_current_control_mode(motor_id,target_current):
    """测试用例3：设置电流/力控制模式"""
    print("\n=== 设置电流控制模式 ===")
    # target_current = 500  # 目标电流500 (单位: 0.01A)
    
    # for i in range(6):  # 对每个电机设置
    servo_data = ServoData()
    ret = lib.RyMotion_CurrentMode(
        ctypes.byref(servo_bus), 
        motor_id, 
        target_current, 
        ctypes.byref(servo_data), 
        100
    )
    if ret == OK:
        print(f"电机 {motor_id} 已设置为电流控制模式 (目标电流: {target_current*0.01}A)")
    else:
        print(f"设置电机 {motor_id} 电流控制模式失败，错误码: {ret}")


def set_operation_mode(mode):
    """测试用例4：设置工作模式"""
    print("\n=== 设置工作模式 ===")
    if mode == "speed":
        print("切换到速度控制模式")
        set_speed_control_mode(1,100,100)
    elif mode == "current":
        print("切换到电流控制模式")
        set_current_control_mode(1,100)
    else:
        print("未知模式，请选择'speed'或'current'")
    print("=====================\n")


# 在主循环中添加测试菜单
def main_menu():
    while True:
        print("\n===== 测试菜单 =====")
        print("1. 获取指定电机角度")
        print("2. 设置速度控制模式,设置指定电机的速度和目标角度")
        print("3. 设置电流控制模式")
        print("4. 切换工作模式")
        print("5. 设置力控控制模式,设置指定电机的速度、电流和目标角度")
        print("6. 退出")
        
        try:
            choice = input("请输入选项(1-6): ")
            if choice == '1':
                get_motor_angles(2)#id
            elif choice == '2':
                set_speed_control_mode(1,2000,80)#id, speed, target_angle
                set_speed_control_mode(2,2000,30)#id, speed, target_angle
                set_speed_control_mode(1,2000,0)#id, speed, target_angle
                set_speed_control_mode(2,2000,0)#id, speed, target_angle    

                set_speed_current_control_mode(1,2000,40,80)                
                set_speed_current_control_mode(2,2000,40,30)       
                set_speed_current_control_mode(1,2000,40,0)                
                set_speed_current_control_mode(1,2000,40,0)     
                # set_speed_control_mode(4,2000,80)#id, speed, target_angle
                # set_speed_control_mode(5,2000,80)#id, speed, target_angle
                # set_speed_control_mode(6,2000,80)#id, speed, target_angle
            elif choice == '3':
                set_current_control_mode(4,150)
            elif choice == '4':
                mode = input("请输入模式(speed/current): ")
                set_operation_mode(mode)
            elif choice == '5':
                set_speed_current_control_mode(1,2000,400,30)#id, speed, current,target_angle
            elif choice == '6':
                print("退出测试菜单")
                break
            else:
                print("无效输入，请重新选择")
        except KeyboardInterrupt:
            print("\n检测到中断，返回主菜单")
            break



def bus_write_callback(can_msg):

    # 获取 can_msg 的地址
    # print(f"can_msg address: {ctypes.addressof(can_msg.contents):#x}")

    if systype == 'Linux' and cpuarch == 'x86_64':
        # 在 Linux ARM64 上，可能需要调整地址偏移
        # 这里假设需要加上 0x600 的偏移
        # 这可能是因为 ctypes 在处理指针时的对齐方式不同

        # 获取 can_msg.contents 的地址
        adjusted_address = ctypes.addressof(can_msg.contents) + 0x600
        # print(f"Adjusted can_msg contents address: {adjusted_address:#x}")

        # # Convert the adjusted address back to a CanMsg object
        adjusted_can_msg = ctypes.cast(adjusted_address, ctypes.POINTER(CanMsg)).contents
        can_msg.contents = adjusted_can_msg

    # Print the message data in hexadecimal format
    data_hex = ' '.join(f'{byte:02X}' for byte in can_msg.contents.pucDat[:can_msg.contents.ucLen])
    print(f"Tx> {can_msg.contents.ulId:04X} ：{data_hex}")
  
    # Implement the callback logic here
    if USE_RS485:

        # // "按如下格式对《嵌入式功能模块通信协议》进入可变长封包：
        # // |  1byte   |  2byte   |    1byte |     n byte |  1byte  |
        # // |  header  |    id    |    len   |    data[]  |  Check  |
        # // |  0xA5    |    x     |     n    |     ...    |   x     |"
        data = []
        data.append(0xA5)
        data.append(can_msg.contents.ulId & 0xFF)
        data.append(can_msg.contents.ulId >> 8 & 0xFF)
        data.append(can_msg.contents.ucLen)
        data.extend(can_msg.contents.pucDat[:can_msg.contents.ucLen])
        check = 0
        for byte in data:
            check += byte
        data.append(check & 0xFF)
        bus.write( data )
        # print(" ".join(f"{byte:02X}" for byte in data))

    else:

        try:    
            # 构造一个 CAN 消息
            msg = can.Message(
                arbitration_id=can_msg.contents.ulId,             # CAN 报文 ID
                data=can_msg.contents.pucDat[:can_msg.contents.ucLen],   # 数据
                is_extended_id=False,    # 是否为扩展 ID
                is_fd=USE_CANFD          # 如果使用 CAN FD，需要设置为 True
            )
            bus.send(msg)
        except can.CanError as e:
            print(f"Failed to send CAN message: {e}")


    return OK

c_bus_write = BusWrite_t(bus_write_callback)



class ReadThread(threading.Thread):
    def __init__(self, serial_port, callback):
        threading.Thread.__init__(self)
        self.serial_port = serial_port
        self.callback = callback
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():

            if self.serial_port is None:
                self.callback(None)
            elif self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                self.callback(data)
            self.stop_event.wait(0.001)

    def stop(self):
        self.stop_event.set()



def bus_read_callback(data):

    # print(" ".join(f"{byte:02X}" for byte in data))
    # return
    
    if USE_RS485:

        global rcvsta, RxIndex, check, tcmd, pdata

        for byte in data:
            if RxIndex == 0:
                check = byte
            else:
                check += byte

            pdata[RxIndex] = byte
            RxIndex += 1

            if rcvsta == CMD_HEADER:
                if tcmd.head != 0xA5:
                    RxIndex = 0
                else:
                    rcvsta = CMD_ID

            elif rcvsta == CMD_ID:
                if RxIndex >= 3:
                    rcvsta = CMD_LEN

            elif rcvsta == CMD_LEN:
                if tcmd.len > CMD_MAX_DAT_LEN:
                    rcvsta = CMD_HEADER
                elif tcmd.len == 0:
                    tcmd.check = check
                    rcvsta = CMD_CHECK
                else:
                    rcvsta = CMD_DATA

            elif rcvsta == CMD_DATA:
                if RxIndex >= (tcmd.len + 4):
                    tcmd.check = check
                    rcvsta = CMD_CHECK

            elif rcvsta == CMD_CHECK:
                if tcmd.check == pdata[RxIndex - 1]:

                    # Process the received message
                    received_msg = CanMsg()
                    received_msg.ulId = tcmd.id
                    received_msg.ucLen = tcmd.len
                    received_msg.pucDat = tcmd.dat
                    lib.RyCanServoLibRcvMsg(ctypes.byref(servo_bus), received_msg)

                    # for i in range(tcmd.len):
                    #     print(f"{tcmd.dat[i]:02X}",  end=' ')
                    # print()

                    # print(f"ID: {tcmd.id:X}, len: {tcmd.len}", end=',')
                    # print(f"Data: {[tcmd.dat[i] for i in range(tcmd.len)]}")

                # Clear the buffer content for safety
                ctypes.memset(ctypes.addressof(tcmd), 0, ctypes.sizeof(Frame_t))
                RxIndex = 0
                rcvsta = CMD_HEADER

            else:
                RxIndex = 0
                rcvsta = CMD_HEADER


    else:
        msg = bus.recv(timeout=0)
        if msg is not None:
            # print(f"收到数据帧：ID: {msg.arbitration_id:x}, 数据: {msg.data}, DLC: {msg.dlc}")
            received_msg = CanMsg()
            received_msg.ulId = msg.arbitration_id
            received_msg.ucLen = msg.dlc
            received_msg.pucDat = (ctypes.c_uint8 * 64)(*msg.data)
            lib.RyCanServoLibRcvMsg(ctypes.byref(servo_bus), received_msg)





def listened_callback(can_msg, para):

    global servos_info

    id = can_msg.contents.ulId & 0xFF
    pstuListen = ctypes.cast(para, ctypes.POINTER(MsgListen)).contents

    # Print the raw data content of the received CAN message
    # can_msg.contents = ctypes.cast(can_msg.contents.ulId, ctypes.POINTER(CanMsg)).contents
    # data_hex = ' '.join(f'{byte:02X}' for byte in can_msg.contents.pucDat[:can_msg.contents.ucLen])
    # print(f"Raw Data: {data_hex}")

    if id and (id < 7):
        servos_info[id-1].raw = ctypes.cast(can_msg.contents.pucDat, ctypes.POINTER(FingerServoInfo)).contents
        servos_info[id-1].p = servos_info[id-1].raw.ub_P
        servos_info[id-1].v = servos_info[id-1].raw.ub_V
        servos_info[id-1].t = servos_info[id-1].raw.ub_I

        if servos_info[id-1].v > 2048:
            servos_info[id-1].v -= 4096
        if servos_info[id-1].t > 2048:
            servos_info[id-1].t -= 4096

        # print(f"id: {id}, Status: {servos_info[id-1].raw.ucStatus}")
        # print(f"id: {id}, P: {servos_info[id-1].p}")
        # print(f"id: {id}, V: {servos_info[id-1].v}")
        # print(f"id: {id}, I: {servos_info[id-1].t}")

    return OK



c_callback1 = Callback_t(listened_callback)



class TimerThread(threading.Thread):
    def __init__(self, interval, callback):
        threading.Thread.__init__(self)
        self.interval = interval
        self.callback = callback
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            self.callback()
            self.stop_event.wait(self.interval)

    def stop(self):
        self.stop_event.set()

def update_ticks():
    global ticks_var

    current_time_seconds = time.time()
    ticks_var.value = int(current_time_seconds * 1000) % 1000

    # 调用硬件的数据接收函数，如果成功接收到数据，则调用 RyCanServoLibRcvMsg
    if False:
        # 下面是一个模拟的接收数据的过程
        received_msg = CanMsg()

        received_msg.ulId = 0x101
        received_msg.ucLen = 8
        received_msg.pucDat[0] = 0xA0
        received_msg.pucDat[1] = 0x02
        received_msg.pucDat[2] = 0x03
        received_msg.pucDat[3] = 0x04
        received_msg.pucDat[4] = 0x05
        received_msg.pucDat[5] = 0x06
        received_msg.pucDat[6] = 0x07
        received_msg.pucDat[7] = 0x08

        # Call the RyCanServoLibRcvMsg function to receive data
        lib.RyCanServoLibRcvMsg(ctypes.byref(servo_bus), received_msg)

        # Process the received message
        # print(f"Received Message ID: {received_msg.ulId}, Data Length: {received_msg.ucLen}")
        # print(f"Received Message Data: {[received_msg.pucDat[i] for i in range(received_msg.ucLen)]}")



def map_angle_deg_to_value(deg, limit_l = 0, limit_h = 90):
    return int((deg - limit_l) / (limit_h - limit_l) * 4095)


def map_value_to_angle_deg(value, limit_l=0, limit_h=90):
    return (value / 4095) * (limit_h - limit_l) + limit_l



def update_motor_positions( angles ):
    """
        Set the positions of the 15 servos based on the given angles and hand orientation.
        :param angles: A list of 15 positions in radians. The list must contain exactly 15 elements.
        :param hand_lr: An integer indicating the hand orientation. 0 for left hand, 1 for right hand.
        :raises ValueError: If the angles list does not contain exactly 15 elements.
        The function performs the following steps:
        1. Validates the length of the angles list.
        2. Clamps the angles to their respective valid ranges.
        3. Converts the angles to servo positions.
        4. Converts the positions to servo commands based on the hand orientation.
        5. Clamps the commands to the valid range [0, 4095].
        6. Sends the commands to the servos.
    """
    if len(angles) != 6:
        raise ValueError("The positions list must contain exactly 6 elements.")
        return
    
    cmds = [0] * 6
    cmds[0] = map_angle_deg_to_value( angles[0],0,110 )
    cmds[1] = map_angle_deg_to_value( angles[1],0,30 )
    cmds[2] = map_angle_deg_to_value( angles[2],0,84 )
    cmds[3] = map_angle_deg_to_value( angles[3],0,90 )
    cmds[4] = map_angle_deg_to_value( angles[4],0,90 )
    cmds[5] = map_angle_deg_to_value( angles[5],0,88 )


    # 发送指令
    for i in range(6):
        servo_data = ServoData()

        # lib.RyMotion_ServoMove_Speed(ctypes.byref(servo_bus), i + 1, cmds[i], 3000, ctypes.byref(servo_data), 0)
        ret = lib.RyMotion_ServoMove_Mix(ctypes.byref(servo_bus), i + 1, cmds[i], 3000, 1000, ctypes.byref(servo_data), 0)
        if ret != OK:
            print(f"Error sending command to servo {i + 1}: {ret}")
        
        sleep(0.0005)
        




def update_kps( kps ):
    """
        Set the positions of the 15 servos based on the given positions.
        :param kps: A list of 15 positions. The list must contain exactly 15 elements.
    """
    if len(kps) != 6:
        return
    
    i = 0
    for kp in kps:
        if kp < 0 or kp > 20:
            kp = 14

        lib.RyParam_SetRigidity(ctypes.byref(servo_bus), i + 1, kp, 0)
        sleep(0.0005)
        i = i+1


  


if __name__ == '__main__':

    preticks = 0

    servos_info = (Servos_t * 15)()


    timer_thread = TimerThread(0.001, update_ticks)
    timer_thread.start()

    # Create and start the bus read thread
    BusRead_thread = ReadThread( None, bus_read_callback)
    BusRead_thread.start()

    # Reset the contents of servo_bus
    ctypes.memset(ctypes.byref(servo_bus), 0, ctypes.sizeof(RyCanServoBus))

    # 指定最大支持的Hook数，由用户根据实际应用情况来指定个数，建议给定值大于2 ，（一个总线至少需一个Hook，用户可以不给定，库内部最少会申请一个Hook）
    servo_bus.usHookNum = 5;                                                 	 		 
    servo_bus.usListenNum = 30+1;                                  
    
    # 初始化库, 内部会使用 malloc 需保证有足够的栈空间，请查看栈空间设置
    response_code = lib.RyCanServoBusInit(ctypes.byref(servo_bus), c_bus_write, ctypes.byref(ticks_var) , 1000) 

    # 添加监听，这里添加了30个监听，分别监听15个舵机的反馈数据，监听对象是 0xa0 和 0xaa 指令数据
    if response_code == 0:
        stuListenMsg = (CanMsg * 30)()
        for i in range(15):
            stuListenMsg[i].ulId = SERVO_BACK_ID(i + 1)
            stuListenMsg[i].pucDat[0] = 0xA0
            ret = lib.AddListen(ctypes.byref(servo_bus), ctypes.byref(stuListenMsg[i]), c_callback1 )
        for i in range(15,30):
            stuListenMsg[i].ulId = SERVO_BACK_ID(i-15 + 1)
            stuListenMsg[i].pucDat[0] = 0xAA
            ret = lib.AddListen(ctypes.byref(servo_bus), ctypes.byref(stuListenMsg[i]), c_callback1 )
    
     # 添加测试菜单调用
    print("系统初始化完成，输入 'test' 进入测试模式，或按其他键继续运行")
    if input().strip().lower() == 'test':
        main_menu()
            

    k = 0
    #主循环
    while True:

        # rxinfo = ServoData()

    
        # # 模拟清除故障
        # if (1000 + ticks_var.value - preticks) % 1000 > 500:
        #     preticks = ticks_var.value
        #     # for i in range(15):
        #     #     # lib.RyParam_ClearFault(ctypes.byref(servo_bus), i + 1, 0)
        #     #     lib.RyFunc_GetServoInfo(ctypes.byref(servo_bus), i + 1, ctypes.byref(rxinfo), 0)
        #     #     sleep(0.0005)

        #     if k == 0 :
        #         angles = [0, 15, 30, 45, 60, 88]
        #         update_motor_positions( angles )
        #     elif k == 1:
        #         angles = [0, 15, 84, 90, 90, 88]
        #         update_motor_positions( angles )
        #     elif k == 2:
        #         angles = [0, 0, 0, 0, 0, 0]
        #         update_motor_positions( angles )

        #     k = (k + 1)%3

        #     # # 测试更新kp    
        #     # kps = [ 14,14,14, 14,14,14 ]
        #     # update_kps( kps )

        # 如果有 esc 键，或 ctrl+c 按下，退出循环，
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '\x1b':  # ESC key
                    break
        except KeyboardInterrupt:
            break

    timer_thread.stop()
    timer_thread.join()
    BusRead_thread.stop()
    BusRead_thread.join()

    bus.shutdown()
    print("设备关闭")