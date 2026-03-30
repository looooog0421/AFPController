#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 只接触不闭环测力实验脚本

功能：
1. 向 /reference_trajectory 持续发布固定姿态的位姿参考
2. 先在空中悬停，自动对 /mujoco/ee_wrench 做零偏标定（tare）
3. 按多个 Z 高度分阶段缓慢下压桌面
4. 每个阶段停住并统计 raw / zeroed wrench
5. 若零偏后的力过大，自动中止并抬起

使用前提：
- roscore 已启动
- mujoco_sim_.py 已启动
- ur5e_velocity_controller.py 已启动，并已进入速度跟踪模式

运行：
python3 src/ur5e_control/scripts/openloop_contact_probe.py
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped


class OpenLoopContactProbe:
    def __init__(self):
        rospy.init_node("openloop_contact_probe")

        # 话题
        self.ref_pub = rospy.Publisher("/reference_trajectory", PoseStamped, queue_size=1)
        self.wrench_sub = rospy.Subscriber("/mujoco/ee_wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber("/mujoco/ee_pose", PoseStamped, self.pose_callback, queue_size=1)

        # 频率
        self.freq = rospy.get_param("~freq", 200.0)
        self.rate = rospy.Rate(self.freq)

        # ===== 实验参数（Base 坐标系，单位 m）=====
        # 选在桌面的内侧区域，较容易到达；如果不合适可以调这两个值
        self.x = rospy.get_param("~x", 0.45)
        self.y = rospy.get_param("~y", -0.25)

        # scene_contact_nomesh.xml 里桌面顶部高度约 0.40 m
        self.z_surface = rospy.get_param("~z_surface", 0.40)

        # 固定姿态：沿用你当前控制器常用姿态
        quat = rospy.get_param("~quat_xyzw", [0.0, 1.0, 0.0, 0.0])
        self.quat = np.array(quat, dtype=float)
        self.quat = self.quat / np.linalg.norm(self.quat)

        # 分阶段高度（相对桌面顶部，单位 mm）
        # 正值表示高于桌面，负值表示压入少量
        self.stage_offsets_mm = rospy.get_param(
            "~stage_offsets_mm",
            [20.0, 10.0, 5.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0, -5.0]
        )

        # 每个阶段停留时间
        self.hold_sec = rospy.get_param("~hold_sec", 2.0)

        # 安全阈值：零偏后力的模长超过该值则立即中止并抬起
        self.abort_force_norm = rospy.get_param("~abort_force_norm", 5.0)

        # 统计数据
        self.latest_wrench = None
        self.latest_pose = None
        self.baseline_wrench = np.zeros(6)
        self.wrench_received = False
        self.pose_received = False

    def wrench_callback(self, msg: WrenchStamped):
        self.latest_wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float)
        self.wrench_received = True

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=float)
        self.pose_received = True

    def publish_pose(self, z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(self.quat[0])
        msg.pose.orientation.y = float(self.quat[1])
        msg.pose.orientation.z = float(self.quat[2])
        msg.pose.orientation.w = float(self.quat[3])
        self.ref_pub.publish(msg)

    def wait_for_wrench(self, timeout=5.0):
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.wrench_received:
                return True
            if rospy.Time.now().to_sec() - start > timeout:
                return False
            self.rate.sleep()
        return False

    def wait_for_pose(self, timeout=5.0):
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.pose_received:
                return True
            if rospy.Time.now().to_sec() - start > timeout:
                return False
            self.rate.sleep()
        return False

    def wait_until_pose_reached(self, z, timeout=5.0, pos_tol=0.01):
        start = rospy.Time.now().to_sec()
        target = np.array([self.x, self.y, z], dtype=float)
        while not rospy.is_shutdown():
            self.publish_pose(z)
            if self.latest_pose is not None:
                current = self.latest_pose[:3]
                err = np.linalg.norm(current - target)
                if err < pos_tol:
                    return True
            if rospy.Time.now().to_sec() - start > timeout:
                return False
            self.rate.sleep()
        return False

    def hold_and_sample(self, z, duration, use_abort=False):
        reached = self.wait_until_pose_reached(z, timeout=max(3.0, duration))
        if not reached:
            if self.latest_pose is not None:
                current = self.latest_pose[:3]
                rospy.logwarn(
                    f"Target pose not reached for z={z:.4f} m. "
                    f"current pose = ({current[0]:.4f}, {current[1]:.4f}, {current[2]:.4f}) m"
                )
            else:
                rospy.logwarn(f"Target pose not reached for z={z:.4f} m and no /mujoco/ee_pose received")
            return None

        samples = []
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start) < duration:
            self.publish_pose(z)
            if self.latest_wrench is not None:
                w = self.latest_wrench.copy()
                samples.append(w)
                if use_abort:
                    w_zero = w - self.baseline_wrench
                    if np.linalg.norm(w_zero[:3]) > self.abort_force_norm:
                        raise RuntimeError(
                            f"Force norm {np.linalg.norm(w_zero[:3]):.2f} N exceeded abort threshold {self.abort_force_norm:.2f} N"
                        )
            self.rate.sleep()

        if len(samples) == 0:
            return None
        return np.vstack(samples)

    @staticmethod
    def summarize(samples):
        mean = np.mean(samples, axis=0)
        std = np.std(samples, axis=0)
        return mean, std

    def print_stage_result(self, stage_name, z, samples):
        mean_raw, std_raw = self.summarize(samples)
        mean_zero = mean_raw - self.baseline_wrench

        print("\n" + "=" * 72)
        print(f"[{stage_name}] z = {z:.4f} m")
        print("raw wrench mean  [Fx Fy Fz Tx Ty Tz]:")
        print(np.array2string(mean_raw, precision=5, suppress_small=False))
        print("raw wrench std   [Fx Fy Fz Tx Ty Tz]:")
        print(np.array2string(std_raw, precision=5, suppress_small=False))
        print("zeroed mean      [Fx Fy Fz Tx Ty Tz]:")
        print(np.array2string(mean_zero, precision=5, suppress_small=False))
        print(f"zeroed force norm: {np.linalg.norm(mean_zero[:3]):.5f} N")
        print("=" * 72)

    def retract(self, z_safe, duration=1.5):
        rospy.loginfo(f"Retracting to safe height z={z_safe:.4f} m")
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start) < duration:
            self.publish_pose(z_safe)
            self.rate.sleep()

    def run(self):
        rospy.loginfo("Waiting for /mujoco/ee_wrench and /mujoco/ee_pose ...")
        if not self.wait_for_wrench(timeout=5.0):
            rospy.logerr("No wrench received from /mujoco/ee_wrench")
            return
        if not self.wait_for_pose(timeout=5.0):
            rospy.logerr("No pose received from /mujoco/ee_pose")
            return

        rospy.sleep(1.0)

        z_safe = self.z_surface + max(self.stage_offsets_mm) / 1000.0
        rospy.loginfo(f"Probe XY = ({self.x:.3f}, {self.y:.3f}) m")
        rospy.loginfo(f"Surface Z = {self.z_surface:.3f} m")
        rospy.loginfo(f"Safe height Z = {z_safe:.3f} m")
        rospy.loginfo(f"Stage offsets (mm) = {self.stage_offsets_mm}")
        rospy.loginfo("Start tare in free space...")

        # 阶段0：空中标零
        tare_samples = self.hold_and_sample(z_safe, self.hold_sec, use_abort=False)
        if tare_samples is None:
            rospy.logerr("Failed to collect tare samples")
            return
        self.baseline_wrench, _ = self.summarize(tare_samples)

        print("\n" + "#" * 72)
        print("[TARE] baseline wrench [Fx Fy Fz Tx Ty Tz]:")
        print(np.array2string(self.baseline_wrench, precision=5, suppress_small=False))
        print("说明：后续 zeroed wrench = raw wrench - baseline")
        print("#" * 72)

        # 逐阶段下压
        try:
            for i, offset_mm in enumerate(self.stage_offsets_mm):
                z = self.z_surface + offset_mm / 1000.0
                stage_name = f"stage_{i:02d}_{offset_mm:+.1f}mm"
                rospy.loginfo(f"Holding {stage_name} at z={z:.4f} m")
                samples = self.hold_and_sample(z, self.hold_sec, use_abort=True)
                if samples is None:
                    rospy.logwarn(f"No samples collected for {stage_name}")
                    continue
                self.print_stage_result(stage_name, z, samples)

        except RuntimeError as e:
            rospy.logwarn(str(e))
        finally:
            self.retract(z_safe)
            rospy.loginfo("Open-loop contact probe finished.")


if __name__ == "__main__":
    try:
        OpenLoopContactProbe().run()
    except rospy.ROSInterruptException:
        pass
