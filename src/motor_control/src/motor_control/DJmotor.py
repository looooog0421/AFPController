#!/usr/bin/env python3


# 适用于大疆M2006电机 1khz时的PID控制器
# sudo ip link set can0 up type can bitrate 1000000
import rospy
import threading
import time
import queue
import can
import numpy as np
from enum import Enum
from motor_msgs.msg import Motor
from dataclasses import dataclass
import copy

M2006_RATIO = 36
M3508_RATIO = 19

DISABLE = False
ENABLE = True

class DJ_MOTOR_MODE(Enum):
    ZERO = 0
    VELOCITY = 1
    POSITION = 2



class DJMotorParam:
    def __init__(self):
        ## 电机参数， 默认设置的是M2006的参数
        self.PulsePerRound = 8192  # 每转脉冲数
        self.RATIO = 36
        self.Current_Limit = 6000 # -10000 ~ 10000 - -10A ~ 10A
        self.GearRatio = 1.0

class DJMotorState:
    def __init__(self):
        self.arrived = False
        self.struck = False
        self.timeout = False
        self.isSetZero = False

class DJMotorValue:
    def __init__(self):
        self.angle = 0.0 # 输出角度 单位度
        self.speed = 0.0 # 输出速度 单位rpm
        self.current = 0.0 # 输出电流 单位A
        self.pulse = 0 # 编码器脉冲数
        self.pulseRead = 0 # 读取的编码器脉冲数
        self.current_read = 0.0 # 读取的电流 单位A
        self.temperature = 0.0 # 电机温度 单位摄氏度

class DJMotorLimit:
    def __init__(self):
        self.PosLimit_ON = False
        self.maxAngle = 0.0 # 位置模式下的最大角度限制 单位度
        self.minAngle = 0.0 # 位置模式下的最小角度

        self.PosSPLimit_ON = False
        self.PosSPLimit = 0.0 # 位置模式下的最大速度限制 单位rpm

        # self.isReleaseWhenStruck = False # 碰撞后是否释放电机
        # self.stuckDetection_ON = False
        # self.timeoutDetection_ON = False
        
        self.ZeroSP = 0.0 # 寻零模式下的速度
        self.ZeroCurrent = 0.0 # 寻零模式下的电流

class DJMotorArgum:
    def __init__(self):
        self.lockPusle = 0 # 电机锁定时的脉冲数
        self.distance = 0.0 # 当前反馈脉冲与上次反馈脉冲之差
        self.lastRxTime = 0.0 # 上次接收数据的时间
        self.ZeroCnt = 0 # 寻零计数器
        self.TimeoutCnt = 0 # 超时计数器 范围是0~65535，超过后重新从0开始计数
        self.StuckCnt = 0 # 碰撞计数器

class Inc_PID:
    def __init__(self):
        self.setVal = 0.0
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.delta = 0.0
        self.CurVal = 0.0
        self.midVal = [0.0, 0.0, 0.0] # 存储上次三个时刻的值
        self.init = False

    def Inc_PID_Init(self, kp, ki, kd, setVal):
        self.CurVal = 0.0
        self.setVal = setVal
        self.midVal = [0.0, 0.0, 0.0]
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.init = True

    def Inc_PID_Operation(self):
        self.midVal[0] = self.setVal - self.CurVal
        self.delta = self.Kp * (self.midVal[0] - self.midVal[1]) \
              + self.Ki * self.midVal[0] \
              + self.Kd * (self.midVal[0] - 2 * self.midVal[1] + self.midVal[2])
        self.midVal[2] = self.midVal[1]
        self.midVal[1] = self.midVal[0]


def GetS16(val):
    """
    将16位无符号整数转换为有符号整数
    """
    if val < 0x8000:
        return val
    else:
        return val - 0x10000
    
def Dec2Hex16(val):
    """
    Convert Decimal to Hex
    return hex, high_byte, low_byte
    example:
        input: val = 512
        output: 0x0200, 0x02, 0x00
    """
    OFFSET = 1 << 16
    MASK = OFFSET - 1
    hexs = '%04x' % (val + OFFSET & MASK)
    high = '0x' + hexs[0:2]
    low = '0x' + hexs[2:4]
    return hexs, high, low

def PEAK(a, b):
    """
    将a限制在±b范围内
    """
    if (abs(a) > abs(b)):
        a = np.sign(a) * abs(b)
    return a

class DJmotor:
    def __init__(self, id):
        ### =================================电机设置=============================
        self.id = id # 电机ID，在CAN总线上唯一标识一个电机
        self._lock = threading.Lock()
        # limit
        MotorLimit = DJMotorLimit()
        MotorLimit.PosLimit_ON = True
        MotorLimit.PosSPLimit = 3000
        MotorLimit.maxAngle = 270
        MotorLimit.minAngle = -270

        MotorLimit.ZeroSP = 500
        MotorLimit.ZeroCurrent = 3500
        # MotorLimit.isReleaseWhenStruck = False
        # MotorLimit.stuckDetection_ON = False
        # MotorLimit.timeoutDetection_ON = True
        self.Limit = MotorLimit

        # argum
        MotorArgum = DJMotorArgum()
        MotorArgum.lockPusle = 0
        self.Argum = MotorArgum
        
        # param
        M2006Param = DJMotorParam()
        M2006Param.PulsePerRound = 8192
        M2006Param.RATIO = 36
        M2006Param.Current_Limit = 6000
        M2006Param.GearRatio = 1.0
        self.Param = M2006Param

        self.Status = DJMotorState()

        self.enable = False
        self.begin = False
        self.mode = DJ_MOTOR_MODE.POSITION

        self.valueSet = DJMotorValue()
        self.Out_valueNow = DJMotorValue()
        self.valueReal = DJMotorValue()
        self.valuePrv = DJMotorValue()

        self.valueSet.angle = 0.0
        self.valueSet.speed = 100
        self.valueSet.current = 0.0

        self.PID_POS = Inc_PID()
        self.PID_POS.Inc_PID_Init(kp=0.06, ki=0.06, kd=0.0, setVal=0.0)
        self.PID_VEL = Inc_PID()
        self.PID_VEL.Inc_PID_Init(kp=8.0, ki=0.5, kd=0.0, setVal=0.0)

        self.firstPosCnt = 0 # 用于电机开机之后的初始位置计数，在计数到一定次数以前，认为其数据不稳定

        mid, high, low = Dec2Hex16(self.id)
        filters = [{"can_id": self.id, "can_mask": 0x7FF, "extended": False}]
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, can_filters=filters)
        self.can_running = True
        self.rx_thread = threading.Thread(target=self.DJ_ReceiveData_CAN2, daemon=True)
        self.rx_thread.start()

        self.tx_thread = threading.Thread(target=self.DJ_SendCurrent_CAN, daemon=True)
        self.tx_thread.start()

    def DJ_SetZero(self):
        
        self.Status.isSetZero = False
        self.valueReal.pulse = 0
        self.valueReal.angle = 0
        self.Argum.lockPusle = 0

    def DJ_ZeroMode(self):
        self.PID_VEL.setVal = self.Limit.ZeroSP
        self.PID_VEL.CurVal = self.valueReal.speed

        self.PID_VEL.Inc_PID_Operation()

        self.valueSet.current += self.PID_VEL.delta

        self.valueSet.current = PEAK(self.valueSet.current, self.Limit.ZeroCurrent)

        if(abs(self.Argum.distance) < 10):
            self.Argum.ZeroCnt += 1
        else:
            self.Argum.ZeroCnt = 0
        
        if (self.Argum.ZeroCnt > 100):
            self.Argum.ZeroCnt = 0
            self.DJ_SetZero()
            print("电机寻零完成！")
            self.mode = DJ_MOTOR_MODE.POSITION
            self.begin = False

    def DJ_PositionMode(self):
        self.valueSet.pulse = self.valueSet.angle * self.Param.PulsePerRound * self.Param.RATIO * self.Param.GearRatio / 360 ## 累计脉冲
        self.PID_POS.setVal = self.valueSet.pulse

        if (self.Limit.PosLimit_ON):
            if (self.PID_POS.setVal > self.Limit.maxAngle * self.Param.PulsePerRound * self.Param.RATIO * self.Param.GearRatio / 360):
                self.PID_POS.setVal = self.Limit.maxAngle * self.Param.PulsePerRound * self.Param.RATIO * self.Param.GearRatio / 360
            if (self.PID_POS.setVal < self.Limit.minAngle * self.Param.PulsePerRound * self.Param.RATIO * self.Param.GearRatio / 360):
                self.PID_POS.setVal = self.Limit.minAngle * self.Param.PulsePerRound * self.Param.RATIO * self.Param.GearRatio / 360

        self.PID_POS.CurVal = self.valueReal.pulse

        self.PID_POS.Inc_PID_Operation()

        self.PID_VEL.setVal = self.PID_POS.delta # 速度环的设定值为位置环的输出
        if(self.Limit.PosSPLimit_ON):
            self.PID_VEL.setVal = PEAK(self.PID_VEL.setVal, self.Limit.PosSPLimit)
        
        self.PID_VEL.CurVal = self.valueReal.speed

        self.PID_VEL.Inc_PID_Operation()

        self.valueSet.current += self.PID_VEL.delta

        # 其实没用上
        if (abs(self.valueSet.pulse - self.valueReal.pulse) < 60):
            self.Status.arrived = True
        else:
            self.Status.arrived = False

    def DJ_SpeedMode(self):
        self.PID_VEL.setVal = self.valueSet.speed
        self.PID_VEL.CurVal = self.valueReal.speed

        self.PID_VEL.Inc_PID_Operation()

        self.valueSet.current += self.PID_VEL.delta

    def DJ_LockPosition(self):
        self.PID_POS.setVal = self.Argum.lockPusle
        self.PID_POS.CurVal = self.valueReal.pulse
        self.PID_POS.Inc_PID_Operation()

        self.PID_VEL.setVal = self.PID_POS.delta
        self.PID_VEL.CurVal = self.valueReal.speed
        self.PID_VEL.Inc_PID_Operation()

        self.valueSet.current += self.PID_VEL.delta

    def DJ_ReceiveData_CAN2(self):
        """优化的CAN接收线程"""
        while self.can_running:
            try:
                msg = self.bus.recv(timeout=0)  # 非阻塞接收
                
                # 处理所有缓存的消息
                while msg and self.can_running:
                    if len(msg.data) >= 6:
                        data = msg.data
                        
                        # 使用锁保护数据更新
                        with threading.Lock() if not hasattr(self, '_lock') else self._lock:
                            self.valueReal.pulseRead = GetS16(data[0] * 256 + data[1])
                            # print("Pulse Read:", self.valueReal.pulseRead)
                            self.valueReal.speed = GetS16(data[2] * 256 + data[3])
                            self.valueReal.current = GetS16(data[4] * 256 + data[5]) / 100.0

                            self.DJ_Position_Calculate() # 计算位置

                            if(self.Param.RATIO == M2006_RATIO):
                                self.Out_valueNow.speed = float(self.valueReal.speed * 1 / M2006_RATIO)
                                self.Out_valueNow.current = float(self.valueReal.current / 1000)
                                self.Out_valueNow.angle = float(self.valueReal.angle)
                            # print("Output_pos:", self.valueReal.angle,
                            #       "Output_speed:", self.Out_valueNow.speed,
                            #       "Output_current:", self.Out_valueNow.current)
                            
                            self.Argum.lastRxTime = 0

                            if(self.firstPosCnt < 10):
                                print(f"电机{self.id}初始位置校准中...cnt={self.firstPosCnt+1}")
                                self.firstPosCnt += 1
                                self.DJ_SetZero()
                                self.enable = DISABLE
                                self.mode = DJ_MOTOR_MODE.POSITION
                                self.begin = True
                            else:
                                self.enable = ENABLE
                    
                    msg = self.bus.recv(timeout=0)  # 继续读取下一帧
                
                # 添加小延时，避免CPU占用过高
                time.sleep(0.0001)  # 0.1ms，允许最高10kHz处理频率
                
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"CAN接收错误: {e}")
                time.sleep(0.001)

    def DJ_Position_Calculate(self):
        """
        根据读到的脉冲计算电机位置
        """
        self.Argum.distance = self.valueReal.pulseRead - self.valuePrv.pulseRead
        # print("Real Pulse:", self.valueReal.pulseRead, "Prv Pulse:", self.valuePrv.pulseRead, "Distance:", self.Argum.distance)
        self.valuePrv = copy.deepcopy(self.valueReal)

        if(abs(self.Argum.distance) > (self.Param.PulsePerRound / 2)):
            self.Argum.distance = self.Argum.distance - np.sign(self.Argum.distance) * self.Param.PulsePerRound

        self.valueReal.pulse = self.valueReal.pulse + self.Argum.distance
        # 计算输出轴转角
        self.valueReal.angle = self.valueReal.pulse * 360.0 / self.Param.PulsePerRound / self.Param.RATIO / self.Param.GearRatio
    
        if(self.begin):
            self.Argum.lockPusle = self.valueReal.pulse
        if(self.Status.isSetZero):
            self.DJ_SetZero()

    def DJ_SendCurrent_CAN(self):
        """
        发送电流命令到电机
        """
        # msg = can.Message()
        while self.can_running:
            try:
                with self._lock:
                    if not self.enable:
                        self.valueSet.current = 0.0
                    else:
                        if self.mode == DJ_MOTOR_MODE.ZERO:
                            self.DJ_ZeroMode()
                        elif self.mode == DJ_MOTOR_MODE.POSITION:
                            self.DJ_PositionMode()
                        elif self.mode == DJ_MOTOR_MODE.VELOCITY:
                            self.DJ_SpeedMode()
                        else:
                            self.DJ_LockPosition()

                        # 限制电流在允许范围内
                        self.valueSet.current = PEAK(self.valueSet.current, self.Param.Current_Limit)

                _, hexh, hexl = Dec2Hex16(int(self.valueSet.current))

                if self.id <= 0x200+4:
                    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                    data[2*(self.id - 0x200 - 1)]   = int(hexh, 16)
                    data[2*(self.id - 0x200 - 1)+1] = int(hexl, 16)
                    msg = can.Message(arbitration_id=0x200, is_extended_id=False, data=data)
                elif self.id <= 0x200+8:
                    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                    data[2*(self.id - 0x200 - 5)]   = int(hexh, 16)
                    data[2*(self.id - 0x200 - 5)+1] = int(hexl, 16)
                    msg = can.Message(arbitration_id=0x1FF, is_extended_id=False, data=data)
                else:
                    print("电机ID超出范围，无法发送命令！")
                    return
                rospy.logdebug_throttle(1.0, f"发送电机{self.id}电流命令: {self.valueSet.current} mA")
                self.bus.send(msg)
                time.sleep(0.001)  # 1ms 发送周期，1kHz
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"发送命令错误: {e}")
                time.sleep(0.001)
                continue


# @dataclass
# class ReceivedCAN:
#     id: int
#     angle_pulse: int
#     speed_pulse: int
#     torque_pulse: int


class MotorLowLevelControl:
    def __init__(self, num_motors=1):
        self.num_motors = num_motors

        self.motors = [DJmotor(0x200+i+1) for i in range(num_motors)]

        # self.can_manager = CANManager()

        self.motor_status_pub1 = rospy.Publisher('/motor1_status', Motor, queue_size=1)
        self.motor_cmd_sub1 = rospy.Subscriber('/motor1_cmd', Motor, self.sub_motor_cmd1)
        
        self.motor_status_pub2 = rospy.Publisher('/motor2_status', Motor, queue_size=1)
        self.motor_cmd_sub2 = rospy.Subscriber('/motor2_cmd', Motor, self.sub_motor_cmd2)

        
    def __del__(self):
        """析构函数，确保线程正确退出"""
        if hasattr(self, 'can_manager'):
            self.can_manager.can_running = False
    
    def publish_status(self):
        # msg = self.can_manager.rx_msg
        for motor in self.motors:
            # if msg and msg.arbitration_id == motor.id:
            #     motor.DJ_ReceiveData_CAN2(msg)
            with motor._lock:

                motor_msg = Motor()
                motor_msg.position = float(motor.Out_valueNow.angle)
                motor_msg.velocity = float(motor.Out_valueNow.speed)
                motor_msg.torque = float(motor.Out_valueNow.current)
                motor_msg.kp = float(motor.PID_POS.Kp)
                motor_msg.kd = float(motor.PID_POS.Kd)
                motor_msg.ki = float(motor.PID_POS.Ki)

                if motor.id == 0x201:
                    self.motor_status_pub1.publish(motor_msg)
                elif motor.id == 0x202:
                    self.motor_status_pub2.publish(motor_msg)

    def sub_motor_cmd1(self, msg: Motor):
        """
        接收电机命令，控制电机
        """
        motor = self.motors[0]
        with motor._lock:
            motor.valueSet.angle = msg.position
            motor.valueSet.speed = msg.velocity

    def sub_motor_cmd2(self, msg: Motor):
        """
        接收电机命令，控制电机2
        """
        motor = self.motors[1]
        with motor._lock:
            motor.valueSet.angle = msg.position
            motor.valueSet.speed = msg.velocity

            


if __name__ == "__main__":
    try:
        rospy.init_node("dj_motor_ros_interface", anonymous=True)
        motor_control = MotorLowLevelControl(num_motors=2)
        rate = rospy.Rate(1000)  # 1kHz 发布频率

        while not rospy.is_shutdown():
            motor_control.publish_status()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass