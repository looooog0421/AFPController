#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
from motor_msgs.msg import Motor
from std_msgs.msg import Float32, Int32
import sys

# ==================== 路径配置 ====================
# 必须保留这行，否则找不到 tension_control
sys.path.append('/home/hzk/AFPController/src/AFP_controller/script')
# ================================================

from tension_control import TensionController 

class DataSynchronizer:
    """
    数据同步器：负责缓存各传感器的最新数据
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_motor_pos = None    # 电机当前位置 (度/脉冲)
        self.robot_dist_buffer = 0.0    # 机械臂前馈 "蓄水池"

    def update_motor(self, pos):
        with self.lock:
            self.latest_motor_pos = pos

    def accumulate_robot_dist(self, dist):
        with self.lock:
            self.robot_dist_buffer += dist

    def get_and_clear_robot_dist(self):
        with self.lock:
            val = self.robot_dist_buffer
            self.robot_dist_buffer = 0.0 
            return val

    def get_motor_pos(self):
        with self.lock:
            return self.latest_motor_pos

class AFPIntegrationNode:
    def __init__(self):
        rospy.init_node("afp_integration_node")
        
        # 1. 初始化同步器
        self.sync = DataSynchronizer()
        
        # 【新增】节点存活标志位 (防止Ctrl+C时的竞态条件)
        self.node_alive = True 

        # 2. 实例化张力控制器 
        # kp设为0.1防止太激进
        self.pid_controller = TensionController(kp=0.6, kd=0.25, ki=0.0, sub_freq=50.0)
        
        # 3. 订阅硬件状态
        rospy.Subscriber("/motor1_status", Motor, self.motor_status_cb)
        rospy.Subscriber("/afp/robot_feedforward_dist", Float32, self.robot_feedforward_cb)

        # 4. 发布指令
        self.motor_pub = rospy.Publisher("/motor1_cmd", Motor, queue_size=1)
        self.relay_pub = rospy.Publisher("/relay2/cmd", Int32, queue_size=1)

        # 5. 控制参数
        self.control_freq = 50.0 
        self.dt = 1.0 / self.control_freq
        
        # ================= 注册停机回调函数 =================
        # 这里的 self 指的是 AFPIntegrationNode 实例，它才有 shutdown_hook
        rospy.on_shutdown(self.shutdown_hook)
        
        # 定时器启动主循环
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        rospy.loginfo("AFP Integration Node Started. Waiting for inputs...")

    def motor_status_cb(self, msg):
        self.sync.update_motor(msg.position)

    def robot_feedforward_cb(self, msg):
        self.sync.accumulate_robot_dist(msg.data)
        
    def shutdown_hook(self):
        """
        节点关闭时的安全刹车逻辑
        """
        # 1. 第一时间切断主循环 (防诈尸)
        self.node_alive = False
        
        # 2. 稍微等一下，让正在运行的那一次循环跑完
        rospy.sleep(0.05) 
        
        rospy.logwarn("AFP Integration Node is shutting down. Sending STOP command...")
        
        # 3. 发送刹车指令 (Kp=0)
        stop_msg = Motor()
        stop_msg.position = 0.0 
        stop_msg.velocity = 0.0
        stop_msg.kp = 0.0   # 关键：失去力气
        stop_msg.kd = 0.0
        stop_msg.ki = 0.0
        
        # 连续发送确保底层收到
        for i in range(5):
            self.motor_pub.publish(stop_msg)
            rospy.sleep(0.01)
            
        rospy.logwarn("Motor STOP command sent. Safe to exit.")

    def control_loop(self, event):
        """
        主控制循环 50Hz
        """
        # 【新增】如果节点已经宣告死亡，直接退出
        if not self.node_alive:
            return

        # 1. 获取当前电机位置
        current_motor_pos = self.sync.get_motor_pos()

        if current_motor_pos is None:
            rospy.logwarn_throttle(2, "Waiting for Motor1 feedback...")
            return

        # 2. 获取各项 Delta
        delta_ur = self.sync.get_and_clear_robot_dist()
        # =================【临时测试代码】=================
        # 假装机械臂每秒走 0.05米 (50mm/s)
        # 每帧 (0.02s) 走的距离 = 0.05 * 0.02 = 0.001 米
        if delta_ur == 0:  # 如果没收到真实机械臂数据
             fake_speed_m_s = 1.00
             fake_dist_m = fake_speed_m_s * self.dt
             # 转换为角度
             roller_diameter = 0.10
             scale_factor = (1.0 / (np.pi * roller_diameter)) * 360.0
             delta_ur = fake_dist_m * scale_factor
        # ================================================

        try:
            delta_pid_raw = self.pid_controller.delta_x()
        except TypeError:
            delta_pid_raw = 0.0

        # ==================== 单位转换 ====================
        roller_diameter = 0.10  
        if delta_pid_raw != 0:
            scale_factor = (1.0 / (np.pi * roller_diameter)) * 360.0
            delta_pid_degree = delta_pid_raw * scale_factor
        else:
            delta_pid_degree = 0.0
        # ================================================

        # 3. 合成目标
        # ================= 修改后 =================
        # 变号！把 + 改成 - 
        # 逻辑：张力大 -> 误差为负 -> 减去负数 = 加上正数 -> 电机前转放料
        target_pos = current_motor_pos + delta_ur - delta_pid_degree

        # 4. 发送指令
        cmd_msg = Motor()
        cmd_msg.position = target_pos
        cmd_msg.velocity = 0.0 
        
        # 恢复 Kp (刚性)
        cmd_msg.kp = 80.0 
        cmd_msg.kd = 2.0
        cmd_msg.ki = 0.0

        self.motor_pub.publish(cmd_msg)

        # 调试日志
        if abs(delta_pid_degree) > 0.001:
           rospy.loginfo(f"Raw(m): {delta_pid_raw:.5f} -> Deg: {delta_pid_degree:.2f} | Tgt: {target_pos:.2f}")

if __name__ == "__main__":
    try:
        node = AFPIntegrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass