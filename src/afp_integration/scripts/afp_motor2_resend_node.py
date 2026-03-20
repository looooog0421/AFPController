#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from motor_msgs.msg import Motor
from std_srvs.srv import Trigger, TriggerResponse


class AFPMotor2ResendNode:
    def __init__(self):
        rospy.init_node("afp_motor2_resend_node")

        self.lock = threading.Lock()
        self.current_position = None
        self.state = "IDLE"
        self.start_pos = 0.0
        self.target_pos = 0.0
        self.start_time = 0.0

        self.resend_length_m = float(rospy.get_param("~resend_length_m", 0.09))
        self.roller_diameter_m = float(rospy.get_param("~roller_diameter_m", 0.02))
        self.direction_sign = 1.0 if float(rospy.get_param("~direction_sign", 1.0)) >= 0.0 else -1.0
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 20.0))
        self.ramp_speed_deg_s = float(rospy.get_param("~ramp_speed_deg_s", 90.0))
        self.hold_after_done = bool(rospy.get_param("~hold_after_done", False))

        self.kp = float(rospy.get_param("~kp", 800.0))
        self.ki = float(rospy.get_param("~ki", 0.0))
        self.kd = float(rospy.get_param("~kd", 8.0))

        if self.resend_length_m <= 0.0:
            raise ValueError("~resend_length_m must be > 0")
        if self.roller_diameter_m <= 0.0:
            raise ValueError("~roller_diameter_m must be > 0")
        if self.command_rate_hz <= 0.0:
            raise ValueError("~command_rate_hz must be > 0")
        if self.ramp_speed_deg_s <= 0.0:
            raise ValueError("~ramp_speed_deg_s must be > 0")

        self.resend_angle_deg = (self.resend_length_m / (math.pi * self.roller_diameter_m)) * 360.0
        self.move_duration = self.resend_angle_deg / self.ramp_speed_deg_s

        self.cmd_pub = rospy.Publisher("/motor2_cmd", Motor, queue_size=1)
        self.status_sub = rospy.Subscriber("/motor2_status", Motor, self.motor_status_cb, queue_size=1)
        self.trigger_srv = rospy.Service("/afp/motor2_resend/trigger", Trigger, self.handle_trigger)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.command_rate_hz), self.timer_cb)

        rospy.loginfo(
            "motor2 重送节点已启动: length=%.3fm angle=%.1fdeg speed=%.1fdeg/s hold_after_done=%s",
            self.resend_length_m,
            self.resend_angle_deg,
            self.ramp_speed_deg_s,
            self.hold_after_done,
        )

    def motor_status_cb(self, msg):
        with self.lock:
            self.current_position = msg.position

    def build_cmd(self, position):
        cmd = Motor()
        cmd.position = position
        cmd.kp = self.kp
        cmd.ki = self.ki
        cmd.kd = self.kd
        return cmd

    def handle_trigger(self, _req):
        with self.lock:
            if self.current_position is None:
                return TriggerResponse(success=False, message="motor2_status not ready")
            if self.state == "RUNNING":
                return TriggerResponse(success=False, message="motor2 resend already running")

            self.start_pos = self.current_position
            self.target_pos = self.start_pos + self.direction_sign * self.resend_angle_deg
            self.start_time = rospy.get_time()
            self.state = "RUNNING"

            start_pos = self.start_pos
            target_pos = self.target_pos

        rospy.loginfo(
            "开始执行 motor2 重送: start=%.1fdeg target=%.1fdeg delta=%.1fdeg duration=%.2fs",
            start_pos,
            target_pos,
            target_pos - start_pos,
            self.move_duration,
        )
        return TriggerResponse(success=True, message="motor2 resend started")

    def timer_cb(self, _event):
        done_log = None
        cmd_position = None

        with self.lock:
            if self.state == "IDLE":
                return

            if self.state == "HOLDING":
                cmd_position = self.target_pos
            else:
                elapsed = max(0.0, rospy.get_time() - self.start_time)
                progress = min(1.0, elapsed / self.move_duration) if self.move_duration > 0.0 else 1.0
                cmd_position = self.start_pos + (self.target_pos - self.start_pos) * progress

                if progress >= 1.0:
                    cmd_position = self.target_pos
                    if self.hold_after_done:
                        self.state = "HOLDING"
                        done_log = "motor2 重送完成，保持最终位置"
                    else:
                        self.state = "IDLE"
                        done_log = "motor2 重送完成，停止发布命令，等待底层 watchdog 自动释放"

        self.cmd_pub.publish(self.build_cmd(cmd_position))

        if done_log is not None:
            rospy.loginfo(done_log)


if __name__ == "__main__":
    try:
        AFPMotor2ResendNode()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exc:
        rospy.logerr(f"motor2 重送节点启动失败: {exc}")
