#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float32

class AFPSmoother:
    def __init__(self):
        rospy.init_node("afp_smoother_node")
        
        # ================== [参数配置区] ==================
        self.roller_diameter = 0.05  # 辊筒直径 0.1m
        
        # [关键调试参数] 方向修正
        # 如果电机反转，直接把这里改成 -1.0，不用改其他代码
        self.motor_dir_sign = -1.0    
        
        # [关键调试参数] 最大转速限制 (度/秒)
        # 限制目标值的跳变速度，像减震器一样吸收震荡
        self.max_speed_deg_per_sec = 60.0 
        # ================================================
        
        self.robot_dist_zero = None
        self.last_target_deg = 0.0
        self.last_time = rospy.Time.now()
        
        # 1. 订阅：来自机械臂的绝对距离 (米)
        rospy.Subscriber("/afp/robot_feedforward_dist", Float32, self.robot_cb)
        
        # 2. 发布：给电机的目标角度 (度)
        self.target_pub = rospy.Publisher("/afp/motor_target_angle", Float32, queue_size=1)
        
        rospy.loginfo(">>> 中间层启动: 平滑器 (Smoother) <<<")
        rospy.loginfo(f"参数: 直径={self.roller_diameter}m, 方向={self.motor_dir_sign}, 限速={self.max_speed_deg_per_sec}°/s")
        
        rospy.spin()

    def robot_cb(self, msg):
        current_dist = msg.data
        
        # 第一次收到数据，锁定机械臂的零点
        if self.robot_dist_zero is None:
            self.robot_dist_zero = current_dist
            rospy.loginfo(f"锁定机械臂基准位置: {current_dist:.4f}m")
            # 此时目标角度归零
            self.last_target_deg = 0.0
            return

        # 1. 计算相对位移 (米)
        delta_m = current_dist - self.robot_dist_zero
        
        # 2. 换算成理论角度 (度)
        # 公式: (位移 / 周长) * 360 * 方向
        raw_target_deg = (delta_m / (math.pi * self.roller_diameter)) * 360.0 * self.motor_dir_sign
        
        # 3. 速度平滑处理 (Ramping)
        # 这是消除震荡的核心！防止目标值瞬间跳变。
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        
        if dt > 0:
            # 计算这一帧允许的最大角度变化量
            max_step = self.max_speed_deg_per_sec * dt
            
            # 计算理论值和上一次值的差
            diff = raw_target_deg - self.last_target_deg
            
            # 限幅逻辑
            if diff > max_step:
                raw_target_deg = self.last_target_deg + max_step
            elif diff < -max_step:
                raw_target_deg = self.last_target_deg - max_step
        
        # 更新状态
        self.last_time = now
        self.last_target_deg = raw_target_deg
        
        # 4. 发布最终平滑后的目标
        self.target_pub.publish(Float32(data=raw_target_deg))
        
        # 调试日志 (只有动的时候才打印)
        if abs(raw_target_deg) > 0.1:
            rospy.loginfo_throttle(0.5, f"Input:{delta_m*1000:.1f}mm -> Output:{raw_target_deg:.1f}°")

if __name__ == "__main__":
    try:
        AFPSmoother()
    except rospy.ROSInterruptException:
        pass
