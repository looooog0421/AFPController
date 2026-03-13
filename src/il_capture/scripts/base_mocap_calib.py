#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定采集与求解:
- mocap 位置: /vrpn_client_node/mimic_tool_Marker1/pose (geometry_msgs/PoseStamped)
- robot 位置: /tf (tf2_msgs/TFMessage), 仅使用 transforms[0] 且要求 header.frame_id == 'base'
- 每组采样窗口 1s，取均值
- 多组姿态后，使用 SVD 计算 T_base->mocap
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
import yaml
import sys
import time

class PoseAverager:
    def __init__(self, window_sec=1.0):
        self.window_sec = window_sec
        self.mocap_buf = []
        self.robot_buf = []
        self.last_mocap = None
        self.last_robot = None
        self.last_mocap_stamp = None
        self.last_robot_stamp = None

        rospy.Subscriber("/vrpn_client_node/mimic_tool_Marker1/pose",
                         PoseStamped, self.mocap_cb, queue_size=50)
        rospy.Subscriber("/tf", TFMessage, self.robot_cb, queue_size=50)

    def mocap_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.last_mocap = np.array([p.x, p.y, p.z]) / 1000.0  # mm -> m
        self.last_mocap_stamp = msg.header.stamp

    def robot_cb(self, msg: TFMessage):
        if not msg.transforms:
            return
        t0 = msg.transforms[0]
        if t0.header.frame_id != 'base':
            return
        p = t0.transform.translation
        self.last_robot = np.array([p.x, p.y, p.z])
        self.last_robot_stamp = t0.header.stamp

    def collect_once(self):
        """采集 window_sec 内的均值，返回 (mocap_mean, robot_mean) 或 (None, None)"""
        self.mocap_buf.clear()
        self.robot_buf.clear()
        start = time.time()
        rate = rospy.Rate(60)

        last_mocap_seen = None
        last_robot_seen = None
        while (time.time() - start) < self.window_sec and not rospy.is_shutdown():
            if self.last_mocap is not None and self.last_mocap_stamp != last_mocap_seen:
                self.mocap_buf.append(self.last_mocap.copy())
                last_mocap_seen = self.last_mocap_stamp

            if self.last_robot is not None and self.last_robot_stamp != last_robot_seen:
                self.robot_buf.append(self.last_robot.copy())
                last_robot_seen = self.last_robot_stamp

            rate.sleep()

        if len(self.mocap_buf) == 0 or len(self.robot_buf) == 0:
            rospy.logwarn("采样失败：mocap 或 robot 数据为空")
            return None, None

        mocap_mean = np.mean(np.stack(self.mocap_buf), axis=0)
        robot_mean = np.mean(np.stack(self.robot_buf), axis=0)
        rospy.loginfo(f"采样完成：mocap {len(self.mocap_buf)} 条，robot {len(self.robot_buf)} 条")
        return mocap_mean, robot_mean

def solve_svd(pts_robot, pts_mocap):
    """Kabsch：求解 Mocap = R * Robot + t"""
    assert pts_robot.shape == pts_mocap.shape
    c_src = pts_robot.mean(axis=0)
    c_tgt = pts_mocap.mean(axis=0)
    src_c = pts_robot - c_src
    tgt_c = pts_mocap - c_tgt
    H = src_c.T @ tgt_c
    U, S, Vt = np.linalg.svd(H)
    R_est = Vt.T @ U.T
    if np.linalg.det(R_est) < 0:
        Vt[-1, :] *= -1
        R_est = Vt.T @ U.T
    t_est = c_tgt - R_est @ c_src
    T = np.eye(4)
    T[:3, :3] = R_est
    T[:3, 3] = t_est
    return T

def save_yaml(T, path="robot_mocap_calib.yaml"):
    trans = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()  # x,y,z,w
    rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    data = {
        "source_frame": "base",
        "target_frame": "mocap_world",
        "translation": {"x": float(trans[0]), "y": float(trans[1]), "z": float(trans[2])},
        "rotation_quaternion": {"x": float(quat[0]), "y": float(quat[1]), "z": float(quat[2]), "w": float(quat[3])},
        "rpy_deg": {"roll": float(rpy[0]), "pitch": float(rpy[1]), "yaw": float(rpy[2])},
        "homogeneous_matrix": T.tolist()
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    rospy.loginfo(f"标定结果已保存: {path}")

def main():
    rospy.init_node("base_mocap_calibrator")
    window = rospy.get_param("~window_sec", 1.0)
    averager = PoseAverager(window_sec=window)

    pts_mocap = []
    pts_robot = []

    rospy.loginfo("准备采样。每次按回车采 1 秒；输入 q 回车 完成并计算。")
    while not rospy.is_shutdown():
        user_in = input("按回车采样，输入 q 回车结束: ").strip().lower()
        if user_in == "q":
            break
        mocap_mean, robot_mean = averager.collect_once()
        if mocap_mean is not None and robot_mean is not None:
            pts_mocap.append(mocap_mean)
            pts_robot.append(robot_mean)
            rospy.loginfo(f"样本 #{len(pts_mocap)} 记录成功：mocap {mocap_mean}, robot {robot_mean}")

    if len(pts_mocap) < 3:
        rospy.logerr("样本不足（至少需要3组姿态）。标定终止。")
        return

    pts_mocap = np.stack(pts_mocap)
    pts_robot = np.stack(pts_robot)

    T = solve_svd(pts_robot, pts_mocap)
    rmse = np.sqrt(np.mean(np.linalg.norm((T @ np.hstack([pts_robot, np.ones((len(pts_robot),1))]).T).T[:, :3] - pts_mocap, axis=1)**2))
    rospy.loginfo("标定完成。T (base->mocap):\n" + np.array2string(T, formatter={'float_kind':lambda x: f"{x: .6f}"}))
    rospy.loginfo(f"RMSE: {rmse*1000:.2f} mm")

    save_yaml(T)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass