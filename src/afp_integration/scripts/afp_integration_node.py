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

        # ================= [安全限幅参数] =================
        self.pid_correction_limit = 800      # 张力补偿最大绝对值（度）
        self.final_target_rel_limit = 2000.0  # 相对零点最大偏移（度）- 提高到10000以支持长距离运动
        self.control_rate = rospy.get_param("~control_rate", 200.0)
        # ================================================

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

        # 调试发布
        self.final_target_pub = rospy.Publisher('/afp/debug/final_target', Float32, queue_size=1)

        self.rate = rospy.Rate(self.control_rate)
        rospy.loginfo(f">>> 集成控制层启动: 正在等待电机和中间层数据... <<< (control_rate={self.control_rate:.1f}Hz)")

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
            if self.current_motor_pos is None or self.motor_zero_pos is None:
                self.rate.sleep()
                continue

            pid_corr_limited = max(-self.pid_correction_limit,
                                   min(self.pid_correction, self.pid_correction_limit))

            final_target = self.motor_zero_pos + self.smoother_angle + pid_corr_limited

            final_min = self.motor_zero_pos - self.final_target_rel_limit
            final_max = self.motor_zero_pos + self.final_target_rel_limit
            final_target = max(final_min, min(final_target, final_max))

            cmd = Motor()
            cmd.position = final_target
            cmd.kp = 800.0  
            cmd.ki = 0.0
            cmd.kd = 8.0

            self.motor_pub.publish(cmd)
            self.final_target_pub.publish(Float32(data=final_target))

            if abs(self.smoother_angle) > 0.1 or abs(self.pid_correction) > 1.0:
                rospy.loginfo_throttle(0.5,
                    f"前馈:{self.smoother_angle:.1f}° + 张力补偿:{self.pid_correction:.1f}° (限幅后:{pid_corr_limited:.1f}°) = 目标:{final_target:.1f}°")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        AFPIntegrationNode()
    except rospy.ROSInterruptException:
        pass
