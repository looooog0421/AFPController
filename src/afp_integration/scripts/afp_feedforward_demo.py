#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float32
from motor_msgs.msg import Motor


class AFPFeedforwardDemo:
    def __init__(self):
        rospy.init_node("afp_feedforward_demo")

        # ====== 可调参数 ======
        self.publish_rate = rospy.get_param("~publish_rate", 200.0)               # Hz
        self.motion_mode = rospy.get_param("~motion_mode", "constant")           # constant / sine_positive
        self.linear_speed = rospy.get_param("~linear_speed", 0.05)                # m/s, constant模式使用
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.05)        # m/s, sine_positive模式峰值
        self.sine_period = rospy.get_param("~sine_period", 6.0)                   # s, sine_positive模式周期
        self.roller_diameter = rospy.get_param("~roller_diameter", 0.05)          # m
        self.run_duration = rospy.get_param("~run_duration", 0.0)                 # s, 0表示一直运行
        self.start_angle = rospy.get_param("~start_angle", 0.0)                   # deg
        self.direction = rospy.get_param("~direction", 1.0)                       # 1.0送带, -1.0回卷
        self.time_mode = rospy.get_param("~time_mode", "fixed")                  # fixed / realtime
        # ====================

        if self.publish_rate <= 0:
            rospy.logerr("publish_rate 必须大于 0")
            rospy.signal_shutdown("invalid publish_rate")
            return

        if self.roller_diameter <= 0:
            rospy.logerr("roller_diameter 必须大于 0")
            rospy.signal_shutdown("invalid roller_diameter")
            return

        if self.sine_period <= 0:
            rospy.logerr("sine_period 必须大于 0")
            rospy.signal_shutdown("invalid sine_period")
            return

        if self.direction not in (-1, -1.0, 1, 1.0):
            rospy.logerr("direction 只能是 1 或 -1")
            rospy.signal_shutdown("invalid direction")
            return

        if self.motion_mode not in ("constant", "sine_positive"):
            rospy.logerr("motion_mode 只能是 constant 或 sine_positive")
            rospy.signal_shutdown("invalid motion_mode")
            return

        if self.time_mode not in ("fixed", "realtime"):
            rospy.logerr("time_mode 只能是 fixed 或 realtime")
            rospy.signal_shutdown("invalid time_mode")
            return

        self.pub = rospy.Publisher("/afp/motor_target_angle", Float32, queue_size=1)
        self.desired_speed_pub = rospy.Publisher("/afp/demo/desired_linear_speed", Float32, queue_size=1)
        self.desired_angular_speed_pub = rospy.Publisher("/afp/demo/desired_angular_speed_deg_s", Float32, queue_size=1)
        self.virtual_distance_pub = rospy.Publisher("/afp/demo/virtual_distance", Float32, queue_size=1)
        self.actual_linear_speed_pub = rospy.Publisher("/afp/demo/actual_linear_speed", Float32, queue_size=1)
        self.actual_angular_speed_pub = rospy.Publisher("/afp/demo/actual_angular_speed_deg_s", Float32, queue_size=1)

        rospy.Subscriber("/motor1_status", Motor, self.motor_status_cb)

        self.circumference = math.pi * self.roller_diameter
        self.fixed_dt = 1.0 / self.publish_rate

        self.target_angle = self.start_angle
        self.virtual_distance = 0.0
        self.last_time = rospy.Time.now().to_sec()
        self.start_time = self.last_time
        self.sim_elapsed_time = 0.0

        self.last_motor_pos = None
        self.last_motor_time = None
        self.actual_angular_speed_deg_s = 0.0
        self.actual_linear_speed = 0.0

        rospy.loginfo(
            f"AFP 前馈 demo 启动 | mode={self.motion_mode} | publish_rate={self.publish_rate:.1f}Hz | "
            f"roller_diameter={self.roller_diameter:.4f}m | direction={self.direction:.0f} | time_mode={self.time_mode}"
        )

        if self.motion_mode == "constant":
            rospy.loginfo(f"恒速模式 | linear_speed={self.linear_speed:.3f}m/s")
        else:
            rospy.loginfo(f"正弦单向模式 | 速度范围=[0, {self.max_linear_speed:.3f}]m/s | period={self.sine_period:.2f}s")

        self.loop()

    def motor_status_cb(self, msg):
        now = rospy.Time.now().to_sec()
        current_pos = msg.position

        if self.last_motor_pos is not None and self.last_motor_time is not None:
            dt = now - self.last_motor_time
            if dt > 0:
                delta_angle = current_pos - self.last_motor_pos
                self.actual_angular_speed_deg_s = delta_angle / dt
                self.actual_linear_speed = self.actual_angular_speed_deg_s / 360.0 * self.circumference

        self.last_motor_pos = current_pos
        self.last_motor_time = now

    def get_desired_linear_speed(self, elapsed_time):
        if self.motion_mode == "constant":
            return self.direction * self.linear_speed

        omega = 2.0 * math.pi / self.sine_period
        speed = 0.5 * self.max_linear_speed * (1.0 + math.sin(omega * elapsed_time - math.pi / 2.0))
        return self.direction * speed

    def loop(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()

            if self.time_mode == "fixed":
                dt = self.fixed_dt
                self.sim_elapsed_time += dt
                elapsed_time = self.sim_elapsed_time
            else:
                dt = now - self.last_time
                if dt < 0:
                    dt = 0.0
                elapsed_time = now - self.start_time

            self.last_time = now

            desired_linear_speed = self.get_desired_linear_speed(elapsed_time)
            desired_angular_speed_deg_s = desired_linear_speed / self.circumference * 360.0

            delta_distance = desired_linear_speed * dt
            delta_angle = desired_angular_speed_deg_s * dt

            self.virtual_distance += delta_distance
            self.target_angle += delta_angle

            self.pub.publish(Float32(data=self.target_angle))
            self.desired_speed_pub.publish(Float32(data=desired_linear_speed))
            self.desired_angular_speed_pub.publish(Float32(data=desired_angular_speed_deg_s))
            self.virtual_distance_pub.publish(Float32(data=self.virtual_distance))
            self.actual_linear_speed_pub.publish(Float32(data=self.actual_linear_speed))
            self.actual_angular_speed_pub.publish(Float32(data=self.actual_angular_speed_deg_s))

            rospy.loginfo_throttle(
                0.5,
                f"目标速度:{desired_linear_speed:.4f}m/s | 实际速度:{self.actual_linear_speed:.4f}m/s | "
                f"目标角度:{self.target_angle:.2f}° | 虚拟位移:{self.virtual_distance:.4f}m"
            )

            if self.run_duration > 0 and elapsed_time >= self.run_duration:
                rospy.loginfo("demo 运行结束，停止发布")
                break

            rate.sleep()

        self.pub.publish(Float32(data=self.target_angle))


if __name__ == "__main__":
    try:
        AFPFeedforwardDemo()
    except rospy.ROSInterruptException:
        pass
