#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import rospy
from std_msgs.msg import Float32
from motor_msgs.msg import Motor


class AFPExperimentLogger:
    def __init__(self):
        rospy.init_node("afp_experiment_logger")

        self.sample_rate = rospy.get_param("~sample_rate", 200.0)
        self.run_duration = rospy.get_param("~run_duration", 0.0)
        self.output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/afp_experiment_logs"))
        self.file_prefix = rospy.get_param("~file_prefix", "afp_periodic")

        if self.sample_rate <= 0:
            rospy.logerr("sample_rate 必须大于 0")
            rospy.signal_shutdown("invalid sample_rate")
            return

        os.makedirs(self.output_dir, exist_ok=True)

        safe_name = rospy.get_param("~file_name", "")
        if safe_name:
            csv_name = safe_name
        else:
            csv_name = "{}_{}.csv".format(self.file_prefix, rospy.Time.now().to_nsec())

        self.csv_path = os.path.join(self.output_dir, csv_name)
        self.start_time = rospy.Time.now().to_sec()

        self.latest = {
            "motor_target_angle": None,
            "final_target": None,
            "tension": None,
            "tension_correction_angle": None,
            "desired_linear_speed": None,
            "desired_angular_speed_deg_s": None,
            "virtual_distance": None,
            "actual_linear_speed": None,
            "actual_angular_speed_deg_s": None,
            "motor_status_position": None,
            "motor_status_velocity": None,
            "motor_status_torque": None,
            "motor_cmd_position": None,
            "motor_cmd_velocity": None,
            "motor_cmd_torque": None,
            "motor_cmd_kp": None,
            "motor_cmd_kd": None,
            "motor_cmd_ki": None,
        }

        self.msg_stamp = {key: None for key in self.latest.keys()}

        self.headers = [
            "time_rel",
            "time_abs",
            "motor_target_angle",
            "final_target",
            "motor_status_position",
            "motor_status_velocity",
            "motor_status_torque",
            "motor_cmd_position",
            "motor_cmd_velocity",
            "motor_cmd_torque",
            "motor_cmd_kp",
            "motor_cmd_kd",
            "motor_cmd_ki",
            "tension",
            "tension_correction_angle",
            "desired_linear_speed",
            "desired_angular_speed_deg_s",
            "virtual_distance",
            "actual_linear_speed",
            "actual_angular_speed_deg_s",
            "stamp_motor_target_angle",
            "stamp_final_target",
            "stamp_motor_status",
            "stamp_motor_cmd",
            "stamp_tension",
            "stamp_tension_correction_angle",
            "stamp_desired_linear_speed",
            "stamp_desired_angular_speed_deg_s",
            "stamp_virtual_distance",
            "stamp_actual_linear_speed",
            "stamp_actual_angular_speed_deg_s",
        ]

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.headers)

        rospy.Subscriber("/afp/motor_target_angle", Float32, self.motor_target_cb)
        rospy.Subscriber("/afp/debug/final_target", Float32, self.final_target_cb)
        rospy.Subscriber("/tension_sensor/data", Float32, self.tension_cb)
        rospy.Subscriber("/afp/tension_correction_angle", Float32, self.tension_correction_cb)
        rospy.Subscriber("/afp/demo/desired_linear_speed", Float32, self.desired_linear_speed_cb)
        rospy.Subscriber("/afp/demo/desired_angular_speed_deg_s", Float32, self.desired_angular_speed_cb)
        rospy.Subscriber("/afp/demo/virtual_distance", Float32, self.virtual_distance_cb)
        rospy.Subscriber("/afp/demo/actual_linear_speed", Float32, self.actual_linear_speed_cb)
        rospy.Subscriber("/afp/demo/actual_angular_speed_deg_s", Float32, self.actual_angular_speed_cb)
        rospy.Subscriber("/motor1_status", Motor, self.motor_status_cb)
        rospy.Subscriber("/motor1_cmd", Motor, self.motor_cmd_cb)

        rospy.on_shutdown(self.close)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self.timer_cb)

        rospy.loginfo("AFP 实验数据记录启动")
        rospy.loginfo("CSV 保存路径: %s", self.csv_path)
        rospy.loginfo("采样频率: %.1f Hz", self.sample_rate)

    def _update_float(self, key, msg):
        self.latest[key] = msg.data
        self.msg_stamp[key] = rospy.Time.now().to_sec()

    def motor_target_cb(self, msg):
        self._update_float("motor_target_angle", msg)

    def final_target_cb(self, msg):
        self._update_float("final_target", msg)

    def tension_cb(self, msg):
        self._update_float("tension", msg)

    def tension_correction_cb(self, msg):
        self._update_float("tension_correction_angle", msg)

    def desired_linear_speed_cb(self, msg):
        self._update_float("desired_linear_speed", msg)

    def desired_angular_speed_cb(self, msg):
        self._update_float("desired_angular_speed_deg_s", msg)

    def virtual_distance_cb(self, msg):
        self._update_float("virtual_distance", msg)

    def actual_linear_speed_cb(self, msg):
        self._update_float("actual_linear_speed", msg)

    def actual_angular_speed_cb(self, msg):
        self._update_float("actual_angular_speed_deg_s", msg)

    def motor_status_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.latest["motor_status_position"] = msg.position
        self.latest["motor_status_velocity"] = msg.velocity
        self.latest["motor_status_torque"] = msg.torque
        self.msg_stamp["motor_status_position"] = now
        self.msg_stamp["motor_status_velocity"] = now
        self.msg_stamp["motor_status_torque"] = now

    def motor_cmd_cb(self, msg):
        now = rospy.Time.now().to_sec()
        self.latest["motor_cmd_position"] = msg.position
        self.latest["motor_cmd_velocity"] = msg.velocity
        self.latest["motor_cmd_torque"] = msg.torque
        self.latest["motor_cmd_kp"] = msg.kp
        self.latest["motor_cmd_kd"] = msg.kd
        self.latest["motor_cmd_ki"] = msg.ki
        self.msg_stamp["motor_cmd_position"] = now
        self.msg_stamp["motor_cmd_velocity"] = now
        self.msg_stamp["motor_cmd_torque"] = now
        self.msg_stamp["motor_cmd_kp"] = now
        self.msg_stamp["motor_cmd_kd"] = now
        self.msg_stamp["motor_cmd_ki"] = now

    def timer_cb(self, _event):
        now = rospy.Time.now().to_sec()
        row = [
            now - self.start_time,
            now,
            self.latest["motor_target_angle"],
            self.latest["final_target"],
            self.latest["motor_status_position"],
            self.latest["motor_status_velocity"],
            self.latest["motor_status_torque"],
            self.latest["motor_cmd_position"],
            self.latest["motor_cmd_velocity"],
            self.latest["motor_cmd_torque"],
            self.latest["motor_cmd_kp"],
            self.latest["motor_cmd_kd"],
            self.latest["motor_cmd_ki"],
            self.latest["tension"],
            self.latest["tension_correction_angle"],
            self.latest["desired_linear_speed"],
            self.latest["desired_angular_speed_deg_s"],
            self.latest["virtual_distance"],
            self.latest["actual_linear_speed"],
            self.latest["actual_angular_speed_deg_s"],
            self.msg_stamp["motor_target_angle"],
            self.msg_stamp["final_target"],
            self.msg_stamp["motor_status_position"],
            self.msg_stamp["motor_cmd_position"],
            self.msg_stamp["tension"],
            self.msg_stamp["tension_correction_angle"],
            self.msg_stamp["desired_linear_speed"],
            self.msg_stamp["desired_angular_speed_deg_s"],
            self.msg_stamp["virtual_distance"],
            self.msg_stamp["actual_linear_speed"],
            self.msg_stamp["actual_angular_speed_deg_s"],
        ]
        self.writer.writerow(row)

        if self.run_duration > 0 and (now - self.start_time) >= self.run_duration:
            rospy.loginfo("达到设定记录时长，停止记录")
            rospy.signal_shutdown("logging finished")

    def close(self):
        if hasattr(self, "csv_file") and self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            rospy.loginfo("CSV 已保存: %s", self.csv_path)


if __name__ == "__main__":
    try:
        AFPExperimentLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
