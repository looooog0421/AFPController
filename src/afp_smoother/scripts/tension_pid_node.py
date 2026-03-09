#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32


class TensionPIDNode:
    def __init__(self):
        rospy.init_node("tension_pid_node")

        # ================= [参数配置] =================
        self.target_tension = rospy.get_param("~target_tension", 0.05)

        # 速度系数 (Gain)
        # 含义：误差每相差 1.0，每一帧增加多少度
        self.speed_gain = rospy.get_param("~speed_gain", 1.0)

        # 累计角度的上限（防止无限卷）
        self.accumulate_limit = rospy.get_param("~accumulate_limit", 800.0)

        # 泄漏系数（防止长期漂移）
        self.leak = rospy.get_param("~leak", 0.999)

        # 传感器超时保护
        self.sensor_timeout = rospy.get_param("~sensor_timeout", 0.2)

        # 死区
        self.deadband = rospy.get_param("~deadband", 0.02)

        self.control_rate = rospy.get_param("~control_rate", 50.0)
        # ============================================

        self.current_tension = 0.0
        self.accumulated_angle = 0.0
        self.last_tension_time = rospy.Time.now().to_sec()

        rospy.Subscriber("/tension_sensor/data", Float32, self.tension_cb)
        self.correction_pub = rospy.Publisher("/afp/tension_correction_angle", Float32, queue_size=1)

        self.rate = rospy.Rate(self.control_rate)
        rospy.loginfo(">>> 张力控制层启动 (泄漏积分模式) <<<")
        rospy.loginfo(
            f"目标张力: {self.target_tension}, 速度系数: {self.speed_gain}, 泄漏: {self.leak}, "
            f"死区: {self.deadband}, 控制频率: {self.control_rate:.1f}Hz"
        )

        self.control_loop()

    def tension_cb(self, msg):
        self.current_tension = msg.data
        self.last_tension_time = rospy.Time.now().to_sec()

    def control_loop(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            timeout = (now - self.last_tension_time) > self.sensor_timeout

            if timeout:
                error = 0.0
                increment = 0.0
            else:
                error = self.current_tension - self.target_tension

                if abs(error) < self.deadband:
                    error = 0.0

                increment = error * self.speed_gain

            self.accumulated_angle = self.leak * self.accumulated_angle + increment

            if self.accumulated_angle > self.accumulate_limit:
                self.accumulated_angle = self.accumulate_limit
            if self.accumulated_angle < -self.accumulate_limit:
                self.accumulated_angle = -self.accumulate_limit

            self.correction_pub.publish(Float32(data=self.accumulated_angle))

            if timeout:
                rospy.logwarn_throttle(1.0, "张力传感器数据超时，暂停累加补偿")
            elif abs(increment) > 0.01:
                state = "送带(正转)" if increment > 0 else "收带(回卷)"
                rospy.loginfo_throttle(
                    0.2,
                    f"张力:{self.current_tension:.2f} | 误差:{error:.2f} | {state} >>> 总修正:{self.accumulated_angle:.1f}°"
                )

            self.rate.sleep()


if __name__ == "__main__":
    try:
        TensionPIDNode()
    except rospy.ROSInterruptException:
        pass
