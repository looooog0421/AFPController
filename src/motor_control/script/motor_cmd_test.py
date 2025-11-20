#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor命令测试工具
用于向/motor_cmd话题发送测试命令
"""

import rospy
import time
from motor_msgs.msg import Motor
import numpy as np

class MotorCommandTester:
    def __init__(self):
        rospy.init_node('motor_cmd_tester', anonymous=True)
        self.pub = rospy.Publisher('/motor1_cmd', Motor, queue_size=1)
        rospy.sleep(1.0)  # 等待发布器初始化
        rospy.loginfo("Motor命令测试器已启动")

    def send_position_command(self, position, kp=0.06, kd=0.06, ki=0.0):
        """发送位置命令"""
        msg = Motor()
        msg.position = float(position)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.kp = float(kp)
        msg.kd = float(kd)
        msg.ki = float(ki)
        
        self.pub.publish(msg)
        rospy.loginfo(f"发送位置命令: {position}度, PID: kp={kp}, kd={kd}, ki={ki}")

    def send_continuous_sine_wave(self, amplitude=90.0, period=5.0, duration=30.0):
        """发送正弦波位置命令"""
        rospy.loginfo(f"开始发送正弦波命令: 幅度={amplitude}度, 周期={period}秒, 持续={duration}秒")
        
        start_time = time.time()
        rate = rospy.Rate(50)  # 50Hz
        
        while time.time() - start_time < duration and not rospy.is_shutdown():
            t = time.time() - start_time
            position = amplitude * np.sin(2 * 3.14159 * t / period)
            self.send_position_command(position, kp=0.06, kd=0.06, ki=0.0)
            rate.sleep()

    def interactive_mode(self):
        """交互式命令模式"""
        rospy.loginfo("进入交互模式，输入命令:")
        rospy.loginfo("格式: <位置> [kp] [kd] [ki]")
        rospy.loginfo("例如: 90 2.0 0.1 0.0")
        rospy.loginfo("输入 'q' 退出")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("请输入命令: ").strip()
                if user_input.lower() == 'q':
                    break
                
                parts = user_input.split()
                if len(parts) >= 1:
                    position = float(parts[0])
                    kp = float(parts[1]) if len(parts) > 1 else 2.0
                    kd = float(parts[2]) if len(parts) > 2 else 0.1
                    ki = float(parts[3]) if len(parts) > 3 else 0.0
                    
                    self.send_position_command(position, kp, kd, ki)
                    
            except (ValueError, KeyboardInterrupt):
                rospy.loginfo("输入无效或收到中断信号")
                break

def main():
    try:
        tester = MotorCommandTester()
        
        rospy.loginfo("Motor命令测试选项:")
        rospy.loginfo("1. 发送单个位置命令")
        rospy.loginfo("2. 发送连续正弦波命令")
        rospy.loginfo("3. 交互式命令模式")
        rospy.loginfo("4. 快速测试序列")
        
        choice = input("请选择模式 (1-4): ").strip()
        
        if choice == "1":
            position = float(input("输入目标位置 (度): "))
            tester.send_position_command(position)
            
        elif choice == "2":
            import math
            sin = math.sin
            amplitude = float(input("输入振幅 (度, 默认90): ") or "90")
            period = float(input("输入周期 (秒, 默认5): ") or "5")
            duration = float(input("输入持续时间 (秒, 默认30): ") or "30")
            tester.send_continuous_sine_wave(amplitude, period, duration)
            
        elif choice == "3":
            tester.interactive_mode()
            
        elif choice == "4":
            # 快速测试序列
            positions = [0, 90, -90, 180, -180, 0]
            rospy.loginfo("开始快速测试序列...")
            for pos in positions:
                tester.send_position_command(pos, kp=2.0, kd=0.1, ki=0.0)
                rospy.sleep(2.0)  # 每个位置保持2秒
                
        else:
            rospy.loginfo("无效选择")
            
    except KeyboardInterrupt:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"程序错误: {e}")

if __name__ == "__main__":
    main()
