#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import csv
import os
import cv2
import sys
import select
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ================= 配置 =================
IMAGE_TOPIC = "/camera/color/image_raw"
BASE_FRAME  = "base"
TOOL_FRAME  = "tool0_controller"

SAVE_DIR = "raw_data/valid"
CSV_FILE = "raw_data/valid/robot_poses.csv"
# ========================================

class SyncSnapshotKey:
    def __init__(self):
        rospy.init_node("sync_snapshot_key")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.want_capture = False
        self.img_idx = 0

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.sub = rospy.Subscriber(
            IMAGE_TOPIC, Image, self.image_cb, queue_size=1
        )

        rospy.loginfo("启动成功，按 s + 回车 采集一组数据")

    def image_cb(self, msg):
        if not self.want_capture:
            return

        stamp = msg.header.stamp
        try:
            trans = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                TOOL_FRAME,
                stamp,
                rospy.Duration(0.2)
            )
        except Exception as e:
            rospy.logwarn(f"TF 获取失败: {e}")
            return

        # 保存图像
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_name = f"frame_{self.img_idx:04d}.png"
        cv2.imwrite(os.path.join(SAVE_DIR, img_name), cv_img)

        # 保存位姿
        t = trans.transform.translation
        q = trans.transform.rotation

        row = [
            self.img_idx,
            t.x, t.y, t.z,
            q.x, q.y, q.z, q.w
        ]

        file_exists = os.path.isfile(CSV_FILE)
        with open(CSV_FILE, "a") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(
                    ["img_index", "x", "y", "z", "qx", "qy", "qz", "qw"]
                )
            writer.writerow(row)

        rospy.loginfo(f"已采集第 {self.img_idx} 组同步数据")
        self.img_idx += 1
        self.want_capture = False   # 只采一次

    def keyboard_loop(self):
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.readline().strip()
                if key == "s":
                    rospy.loginfo("收到采集指令，等待下一帧图像...")
                    self.want_capture = True


if __name__ == "__main__":
    node = SyncSnapshotKey()
    node.keyboard_loop()
