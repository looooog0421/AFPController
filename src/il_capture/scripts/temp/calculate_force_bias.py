#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取力传感器 1s 平均值，结合 capture_config.yaml 的 mass 与 com，
计算当前力/力矩零漂，并回写配置文件。
"""

import rospy
import numpy as np
import yaml
import os
import sys
import time
import tf.transformations as tf_trans
from geometry_msgs.msg import WrenchStamped
from il_capture.msg import KineticStateStamped


class ForceBiasCalculator:
    def __init__(self, config_path, duration=1.0,
                 wrench_topic="/netft_data",
                 pose_topic="/mimic_tool/kinetic_state"):
        self.config_path = config_path
        self.duration = duration

        self.mass = None
        self.com = None
        self.R_tool_sensor = np.eye(3)

        self.force_buf = []
        self.torque_buf = []
        self.quat_buf = []

        self.collecting = False

        self.load_config()

        rospy.Subscriber(wrench_topic, WrenchStamped, self.wrench_cb, queue_size=200)
        rospy.Subscriber(pose_topic, KineticStateStamped, self.pose_cb, queue_size=50)

    def load_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config not found: {self.config_path}")

        with open(self.config_path, "r") as f:
            data = yaml.safe_load(f)

        self.mass = float(data["mass_kg"])
        self.com = np.array(data["center_of_mass_m"], dtype=float)

        # 如果提供了 T_sensor_to_ee / T_ee_to_tool，就计算 R_tool_sensor
        if "T_sensor_to_ee" in data and "T_ee_to_tool" in data:
            T_sensor_to_ee = np.array(data["T_sensor_to_ee"], dtype=float)
            T_ee_to_tool = np.array(data["T_ee_to_tool"], dtype=float)
            R_sensor_to_ee = T_sensor_to_ee[:3, :3]
            R_ee_to_tool = T_ee_to_tool[:3, :3]
            R_sensor_to_tool = R_ee_to_tool @ R_sensor_to_ee
            self.R_tool_sensor = R_sensor_to_tool.T
        else:
            self.R_tool_sensor = np.eye(3)

    def wrench_cb(self, msg: WrenchStamped):
        if not self.collecting:
            return
        f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        t = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        if np.any(np.isnan(f)) or np.any(np.isnan(t)) or np.any(np.isinf(f)) or np.any(np.isinf(t)):
            return
        self.force_buf.append(f)
        self.torque_buf.append(t)

    def pose_cb(self, msg: KineticStateStamped):
        if not self.collecting:
            return
        q = msg.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if np.any(np.isnan(quat)) or np.any(np.isinf(quat)):
            return
        self.quat_buf.append(quat)

    def average_quat(self, quats):
        if len(quats) == 0:
            return np.array([0.0, 0.0, 0.0, 1.0])
        ref = quats[0]
        aligned = []
        for q in quats:
            if np.dot(q, ref) < 0:
                q = -q
            aligned.append(q)
        q_mean = np.mean(aligned, axis=0)
        q_mean /= np.linalg.norm(q_mean)
        return q_mean

    def collect(self):
        self.force_buf.clear()
        self.torque_buf.clear()
        self.quat_buf.clear()

        self.collecting = True
        start = time.time()
        rate = rospy.Rate(200)

        while (time.time() - start) < self.duration and not rospy.is_shutdown():
            rate.sleep()

        self.collecting = False

        if len(self.force_buf) == 0 or len(self.torque_buf) == 0:
            raise RuntimeError("力传感器数据为空，无法计算偏置")

        f_mean = np.mean(np.stack(self.force_buf), axis=0)
        t_mean = np.mean(np.stack(self.torque_buf), axis=0)
        q_mean = self.average_quat(self.quat_buf)

        return f_mean, t_mean, q_mean

    def compute_bias(self, f_mean, t_mean, q_mean):
        # R_world_tool
        R_world_tool = tf_trans.quaternion_matrix(q_mean)[:3, :3]
        # R_world_sensor
        R_world_sensor = R_world_tool @ self.R_tool_sensor

        g_vec_world = np.array([0.0, 0.0, -self.mass * 9.81])
        g_vec_sensor = R_world_sensor.T @ g_vec_world
        t_gravity_sensor = np.cross(self.com, g_vec_sensor)

        f_bias = f_mean - g_vec_sensor
        t_bias = t_mean - t_gravity_sensor

        return f_bias, t_bias

    def update_yaml(self, f_bias, t_bias):
        with open(self.config_path, "r") as f:
            data = yaml.safe_load(f)

        data["static_force_offset_N"] = [float(x) for x in f_bias]
        data["static_torque_offset_Nm"] = [float(x) for x in t_bias]

        with open(self.config_path, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

    def run(self):
        rospy.loginfo(f"开始采样 {self.duration:.2f} s...")
        f_mean, t_mean, q_mean = self.collect()
        rospy.loginfo(f"平均力: {f_mean}, 平均力矩: {t_mean}")
        rospy.loginfo(f"平均姿态四元数: {q_mean}")

        f_bias, t_bias = self.compute_bias(f_mean, t_mean, q_mean)
        rospy.loginfo(f"计算得到静态力偏置: {f_bias}")
        rospy.loginfo(f"计算得到静态力矩偏置: {t_bias}")

        self.update_yaml(f_bias, t_bias)
        rospy.loginfo(f"已写入 {self.config_path}")


def main():
    rospy.init_node("force_bias_calculator", anonymous=True)

    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        config_path = "/home/lgx/Project/AFP/src/il_capture/config/capture_config.yaml"

    duration = rospy.get_param("~duration", 1.0)

    calc = ForceBiasCalculator(config_path=config_path, duration=duration)
    calc.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")