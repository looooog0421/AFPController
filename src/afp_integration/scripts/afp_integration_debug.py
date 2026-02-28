#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from motor_msgs.msg import Motor
from std_msgs.msg import Float32

class AFPIntegrationNode:
    def __init__(self):
        rospy.init_node("afp_integration_node")
        
        self.current_motor_pos = None 
        self.motor_zero_pos = None
        
        # 两个数据源，默认为 0
        self.smoother_angle = 0.0     # 机械臂算出来的
        self.pid_correction = 0.0     # 张力计反馈回来的
        
        self.has_received_smoother = False
        
        # 订阅电机状态 (为了锁零点)
        rospy.Subscriber("/motor1_status", Motor, self.motor_status_cb)
        
        # 订阅中间层 1: 机械臂平滑器
        rospy.Subscriber("/afp/motor_target_angle", Float32, self.smoother_cb)
        
        # 订阅中间层 2: 张力PID控制器
        rospy.Subscriber("/afp/tension_correction_angle", Float32, self.pid_cb)
        
        self.motor_pub = rospy.Publisher("/motor1_cmd", Motor, queue_size=1)
        
        self.rate = rospy.Rate(50)
        rospy.loginfo(">>> 集成控制层启动: 正在等待电机和中间层数据... <<<")
        
        self.control_loop()

    def motor_status_cb(self, msg):
        self.current_motor_pos = msg.position
        if self.motor_zero_pos is None:
            self.motor_zero_pos = self.current_motor_pos
            rospy.loginfo(f"电机零点已锁定: {self.motor_zero_pos:.2f}")

    def smoother_cb(self, msg):
        self.smoother_angle = msg.data
        self.has_received_smoother = True

    def pid_cb(self, msg):
        self.pid_correction = msg.data

    def control_loop(self):
        while not rospy.is_shutdown():
            # 1. 还没准备好，就等待
            if self.current_motor_pos is None or self.motor_zero_pos is None:
                self.rate.sleep()
                continue
            
            # 2. 如果机械臂还没动，就锁死在零点 (或者根据PID微调)
            # 这里的逻辑是：即使机械臂没动，如果张力不对，电机也要动一动来调整张力
            # 所以只要锁定了零点，就可以开始执行融合逻辑
            
            # ================= [最终融合] =================
            # 最终目标 = 零点 + 机械臂前馈 + 张力PID补偿
            final_target = self.motor_zero_pos + self.smoother_angle + self.pid_correction
            # ============================================

            # 3. 下发指令
            cmd = Motor()
            cmd.position = final_target
            
            # 这里的 Kp Ki Kd 是给电机底层驱动的，让它硬一点
            cmd.kp = 120.0   
            cmd.ki = 80.0
            cmd.kd = 8.0     
            
            self.motor_pub.publish(cmd)
            
            # 调试显示
            # 只有当系统处于活跃状态（有补偿或有运动）时才打印
            if abs(self.smoother_angle) > 0.1 or abs(self.pid_correction) > 1.0:
                rospy.loginfo_throttle(0.5, 
                    f"前馈:{self.smoother_angle:.1f}° + 张力补偿:{self.pid_correction:.1f}° = 目标:{final_target:.1f}")
                
            self.rate.sleep()

if __name__ == "__main__":
    try:
        AFPIntegrationNode()
    except rospy.ROSInterruptException:
        pass