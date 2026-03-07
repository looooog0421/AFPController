#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

class TensionPIDNode:
    def __init__(self):
        rospy.init_node("tension_pid_node")

        # ================= [参数配置] =================
        self.target_tension = 0.05 # 目标张力 (你可以根据手感调)

        # 速度系数 (Gain)
        # 含义：误差每相差 1.0，每一帧(0.02秒)增加多少度？
        self.speed_gain = 1.0

        # 累计角度的上限（防止无限卷）
        self.accumulate_limit = 800.0   # 从 18000 降到 800，更保守

        # 泄漏系数（防止长期漂移）
        self.leak = 0.999               # 每帧保留 99.9%，慢慢释放偏置

        # 传感器超时保护
        self.sensor_timeout = 0.2       # 200ms 没数据就停止累加
        # ============================================

        self.current_tension = 0.0
        self.accumulated_angle = 0.0  # 【核心】这是我们累计的总修正量
        self.last_tension_time = rospy.Time.now().to_sec()

        rospy.Subscriber("/tension_sensor/data", Float32, self.tension_cb)
        self.correction_pub = rospy.Publisher("/afp/tension_correction_angle", Float32, queue_size=1)

        self.rate = rospy.Rate(50) # 50Hz
        rospy.loginfo(f">>> 张力控制层启动 (泄漏积分模式) <<<")
        rospy.loginfo(f"目标张力: {self.target_tension}, 速度系数: {self.speed_gain}, 泄漏: {self.leak}")

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
                # 1. 计算误差
                # 张力 > 目标 (紧) -> Error > 0 -> 需要正转送带
                # 张力 < 目标 (松) -> Error < 0 -> 需要反转收带
                error = self.current_tension - self.target_tension

                # 2. 死区处理 (防止数值跳动导致电机一直滋滋响)
                if abs(error) < 0.02:
                    error = 0.0

                # 3. 计算本帧的增量 (Increment)
                increment = error * self.speed_gain

            # 4. 泄漏 + 累加到总角度
            self.accumulated_angle = self.leak * self.accumulated_angle + increment

            # 5. 安全限幅 (防止数值溢出)
            if self.accumulated_angle > self.accumulate_limit:
                self.accumulated_angle = self.accumulate_limit
            if self.accumulated_angle < -self.accumulate_limit:
                self.accumulated_angle = -self.accumulate_limit

            # 6. 发布总累计角度
            self.correction_pub.publish(Float32(data=self.accumulated_angle))

            # 调试日志
            if timeout:
                rospy.logwarn_throttle(1.0, "张力传感器数据超时，暂停累加补偿")
            elif abs(increment) > 0.01:
                state = "送带(正转)" if increment > 0 else "收带(回卷)"
                rospy.loginfo_throttle(0.2,
                    f"张力:{self.current_tension:.2f} | 误差:{error:.2f} | {state} >>> 总修正:{self.accumulated_angle:.1f}°")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        TensionPIDNode()
    except rospy.ROSInterruptException:
        pass