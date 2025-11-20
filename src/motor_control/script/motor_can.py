#！/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import can.logger
import rospy
import time
import os
import threading
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32
from motor_msgs.msg import Motor
import numpy as np

"""
2 robotmaster M2006 motor, with C610 can control
ids = [0x201, 0x202](hex2dec: [513, 514])
CAN protocal
1. send data to motor
    eg: send -1A(-1000 = [0xFC, 0x18]) to motor with id 0x201, send 100mA (100 = [0x00, 0x64]) to motor with id 0x202
    data = [0xFC, 0x18, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00]
2. receive data from motor
    eg: receive data from motor with id 0x201
    angle: (data[0] * 256 + data[1]) / 8192 * 360
    speed: (data[2] * 256 + data[3])
    torque: (data[4] * 256 + data[5])
"""

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

    

"""
single motor control
"""
class PIDcontroller:
    def __init__(self, kp, ki, kd, 
                output_limits=(-10, 10),  # 电流限制范围(A)
                intergral_limit=0, deadband=0,
                period=0.05, max_err=360):
        
        self.target = 0
        self.lastNoneZeroTarget = 0.001

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.pos = 0
        self.pos_last = 0
        self.vel = 0
        self.vel_last = 0

        self.pout = 0
        self.iout = 0
        self.dout = 0

        self.output = 0
        self.output_last = 0

        self.output_max = output_limits[1] * 1000 # mA
        self.output_min = output_limits[0] * 1000 # mA
        self.IntegralLimit = intergral_limit    
        self.DeadBand = deadband
        self.controlPeriod = period
        self.MaxErr = max_err

        self.thistime = time.time()
        self.lasttime = self.thistime
        self.dt = 0

    def pid_reset(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid_update(self, pos, vel):
        self.lasttime = self.thistime
        self.thistime = time.time()
        self.dt = self.thistime - self.lasttime

        self.pos_last = self.pos # x_(t-1) 
        self.pos = pos # x_t
        self.output_last = self.output
        self.vel_last = self.vel
        self.vel = vel
        
    def pid_calculate(self, target):
        self.target = target
        err = self.target - self.pos
        
        # 处理角度环绕问题（-180到180度）
        if err > 180:
            err -= 360
        elif err < -180:
            err += 360

        if(abs(err) > self.DeadBand):
            self.pout = self.kp * err
            # self.iout += self.ki * err * self.dt  # 积分项
            self.dout = self.kd * (0 - self.vel)

            self.output = self.pout + self.iout + self.dout

            if(self.output > self.output_max):
                self.output = self.output_max
            if(self.output < self.output_min):
                self.output = self.output_min
        else:
            self.output = 0

        return self.output

class MotorControl:
    def __init__(self, id, kp=1.5, ki=0.1, kd=0.0, output_limits = (-10, 10), curr_level=5):
        self.id = id
        self.curr_level = curr_level  # current level in 0.1A
        mid, h, l = Dec2Hex16(id)
        filters = [{"can_id": int(mid, 16), "can_mask": 0xFFFF, "extended": False}]
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, can_filters=filters)

        self.pid_controller = PIDcontroller(kp, ki, kd, 
                                 output_limits=(-10, 10), intergral_limit=0, deadband=0,
                                 period=0.05, max_err=360)
        
        # 电机状态变量，使用线程锁保护
        self.lock = threading.Lock()
        self.last_angle = 0
        self.last_speed = 0
        self.last_torque = 0
        self.data_updated = False
        
        # 启动CAN接收线程
        self.running = True
        self.can_thread = threading.Thread(target=self._can_receiver_thread)
        self.can_thread.daemon = True
        self.can_thread.start()

    def _can_receiver_thread(self):
        """CAN消息接收线程，持续接收并更新电机状态"""
        while self.running:
            try:
                # 使用timeout=0实现非阻塞接收，类似test_can.py
                msg = self.bus.recv(timeout=0)
                
                # 处理所有缓存的消息
                while msg and self.running:
                    if len(msg.data) >= 6:
                        angle = GetS16(msg.data[0] * 256 + msg.data[1]) / 8192 * 360
                        speed = GetS16(msg.data[2] * 256 + msg.data[3])
                        torque = GetS16(msg.data[4] * 256 + msg.data[5])
                        
                        with self.lock:
                            self.last_angle = angle
                            self.last_speed = speed
                            self.last_torque = torque
                            self.data_updated = True
                            
                        self.pid_controller.pid_update(angle, speed)
                    
                    # 继续读取下一条消息
                    msg = self.bus.recv(timeout=0)
                
                # 短暂休眠，避免占用过多CPU
                time.sleep(0.0001)  # 100μs
                
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"CAN接收错误: {e}")
                time.sleep(0.001)

    def measure(self):
        """获取最新的电机状态数据"""
        with self.lock:
            return self.last_angle, self.last_speed, self.last_torque
            
    def stop(self):
        """停止CAN接收线程"""
        self.running = False
        if self.can_thread.is_alive():
            self.can_thread.join(timeout=1.0)

    # def measure(self):
    #     # 使用非阻塞方式读取，避免延迟
    #     msg = self.bus.recv(timeout=0.001)  # 1ms超时
    #     if msg is None:
    #         # 如果没有收到消息，返回上次的值
    #         return getattr(self, 'last_angle', 0), getattr(self, 'last_speed', 0), getattr(self, 'last_torque', 0)
            
    #     angle = GetS16(msg.data[0] * 256 + msg.data[1]) / 8192
    #     speed = GetS16(msg.data[2] * 256 + msg.data[3])
    #     torque = GetS16(msg.data[4] * 256 + msg.data[5])

    #     # 保存最新值
    #     self.last_angle = angle
    #     self.last_speed = speed  
    #     self.last_torque = torque

    #     self.pid_controller.pid_update(angle, speed)
        
    #     return angle, speed, torque

    def move(self, target):
        output_current = self.pid_controller.pid_calculate(target)
        print(f"Motor ID: {self.id}, Target: {target:.2f}, Output Current: {output_current:.2f} mA")
        self.send_msg(output_current)

    def send_msg(self, curr):
        _, hexh, hexl = Dec2Hex16(int(curr))
        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.id == 513: # 电机1
            data[0] = int(hexh, 16)
            data[1] = int(hexl, 16)
        elif self.id == 514: # 电机2
            data[2] = int(hexh, 16)
            data[3] = int(hexl, 16)
        
        msg = can.Message(arbitration_id=512, is_extended_id=False, data=data)
        self.bus.send(msg)

class ServoMotorLowLevelControl:
    def __init__(self):
        rospy.loginfo("Setting up the node")
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        self.motor_status_pub1 = rospy.Publisher('motor1_status', Motor, queue_size=1)
        self.motor_status_pub2 = rospy.Publisher('motor2_status', Motor, queue_size=1)
        self.motor_cmd_sub1 = rospy.Subscriber('/motor1_cmd', Motor, self.motor_cmd_cb1)
        self.motor_cmd_sub2 = rospy.Subscriber('/motor2_cmd', Motor, self.motor_cmd_cb2)
        self.m1ctrl = MotorControl(513)
        # self.last_angle = 0
        self.m2ctrl = MotorControl(514)
        
    def __del__(self):
        """析构函数，确保线程正确退出"""
        if hasattr(self, 'm1ctrl'):
            self.m1ctrl.stop()

    def motor_cmd_cb1(self, data):
        # 接收Motor消息并控制电机
        # 使用position作为目标位置，kp, kd, ki更新PID参数
        self.m1ctrl.pid_controller.pid_reset(data.kp, data.ki, data.kd)
        self.m1ctrl.move(data.position)

    def motor_cmd_cb2(self, data):
        # 接收Motor消息并控制电机
        # 使用position作为目标位置，kp, kd, ki更新PID参数
        self.m2ctrl.pid_controller.pid_reset(data.kp, data.ki, data.kd)
        self.m2ctrl.move(data.position)


    def publish_status(self):
        # 发布电机状态使用Motor消息格式
        angle, speed, torque = self.m1ctrl.measure()
        # err = angle - self.last_angle
        # if err > 180:
        #     err -= 360
        # elif err < -180:
        #     err += 360
        motor_msg = Motor()
        motor_msg.position = float(angle)
        motor_msg.velocity = float(speed)
        motor_msg.torque = float(torque)
        motor_msg.kp = float(self.m1ctrl.pid_controller.kp)
        motor_msg.kd = float(self.m1ctrl.pid_controller.kd)
        motor_msg.ki = float(self.m1ctrl.pid_controller.ki)
        # self.last_angle = angle
        self.motor_status_pub1.publish(motor_msg)

        angle2, speed2, torque2 = self.m2ctrl.measure()

        motor_msg2 = Motor()
        motor_msg2.position = float(angle2)
        motor_msg2.velocity = float(speed2)
        motor_msg2.torque = float(torque2)
        motor_msg2.kp = float(self.m2ctrl.pid_controller.kp)
        motor_msg2.kd = float(self.m2ctrl.pid_controller.kd)
        motor_msg2.ki = float(self.m2ctrl.pid_controller.ki)

        self.motor_status_pub2.publish(motor_msg2)
        


if __name__ == "__main__":
    motor_interface = ServoMotorLowLevelControl()
    
    # 选择发布模式
    HIGH_FREQUENCY_MODE = True  # 设置为True启用高频模式
    
    try:
        if HIGH_FREQUENCY_MODE:
            # 高频模式：最大可能的发布频率
            rospy.loginfo("启用高频模式 - 无频率限制")
            while not rospy.is_shutdown():
                motor_interface.publish_status()
                time.sleep(0.001)  # 1ms延时，理论上可达1000Hz
        else:
            # 标准模式：固定频率
            rate = rospy.Rate(100)  # 100 Hz
            rospy.loginfo("启用标准模式 - 100Hz")
            while not rospy.is_shutdown():
                motor_interface.publish_status()
                rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down motor interface...")
    finally:
        # 确保线程正确退出
        motor_interface.m1ctrl.stop()
    
