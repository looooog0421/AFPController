#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UR→电机前馈测试节点（关闭张力回路）

目标：URSim/真机 走 10cm 直线，电机同步放 10cm 带。

说明：
- 不再直接用 RTDE Python 接口读取 TCP 状态（rtde_receive）。
- 参考 [ur5e_velocity_controller.py](ur5e_control/src/ur5e_control/ur5e_velocity_controller.py) 的做法：
  订阅 ROS /joint_states，并用 Pinocchio 高频计算 tool0 的位姿与速度。
- 机械臂运动仍用 URScript socket (30002) 下发 movel。

测试步骤：
1) 启动 URSim 或真机 ur_robot_driver（保证 /joint_states 有数据）
2) 启动 motor_can.py
3) 启动 afp_integration_node.py（张力回路可先关）
4) 启动本节点
"""

import math
import socket
import threading
import time

import numpy as np
import pinocchio as pin
import rospy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from motor_msgs.msg import Motor


# ==================== 参数配置 ====================
ROBOT_IP = "192.168.10.103"  # 真机 IP；URSim 可改回 127.0.0.1
CONTROL_FREQUENCY = 200.0  # 控制循环频率（Hz），不依赖 RTDE

# 机械臂运动参数
LINE_DISTANCE = 0.10  # 10 cm
LINE_SPEED = 0.008     # 0.8 cm/s（比 1cm/s 慢一点，更保守）
LINE_ACCEL = 0.1

# 电机/滚轮参数（注意：motor_can.py 已把编码器角度“除以减速比”，所以这里把 position 当作输出轴角度使用）
ROLLER_DIAMETER = 0.057  # 滚轮直径 (m)
ROLLER_CIRCUMFERENCE = math.pi * ROLLER_DIAMETER

# 前馈增益（用于调节）
FEEDFORWARD_GAIN = 1.0

# 从 ROS /joint_states 计算 TCP 位姿和速度（参考 ur5e_velocity_controller.py）
MODEL_PATH = "/home/hzk/AFPController/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def linear_speed_to_output_deg_s(v_linear: float) -> float:
    """线速度(m/s) → 输出轴角速度(deg/s)。"""
    roller_rev_s = v_linear / ROLLER_CIRCUMFERENCE
    return roller_rev_s * 360.0


class URStateFromJointStates:
    """用 ROS /joint_states + Pinocchio 计算 tool0 的位姿与速度。

    输出格式尽量对齐 RTDE 的习惯：
    - tcp_pose:  [x, y, z, rx, ry, rz]（rx/ry/rz 为旋转向量）
    - tcp_speed: [vx, vy, vz, wx, wy, wz]（WORLD/base）
    """

    def __init__(self, joint_state_topic: str = "/joint_states"):
        self.pin_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.pin_data = pin.Data(self.pin_model)
        self.fid_tool0 = self.pin_model.getFrameId("tool0")

        self.joint_idx_mapping = None
        self.tcp_pose = None
        self.tcp_speed = None

        self._lock = threading.Lock()
        rospy.Subscriber(joint_state_topic, JointState, self._joint_state_cb, queue_size=1)

    def _joint_state_cb(self, msg: JointState):
        try:
            if self.joint_idx_mapping is None:
                name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
                mapping = []
                for joint_name in JOINT_NAMES:
                    if joint_name not in name_to_idx:
                        return
                    mapping.append(name_to_idx[joint_name])
                self.joint_idx_mapping = mapping
                rospy.loginfo(f"[URState] Joint index mapping: {self.joint_idx_mapping}")

            q = np.array(msg.position)[self.joint_idx_mapping]
            qd = np.array(msg.velocity)[self.joint_idx_mapping]

            pin.forwardKinematics(self.pin_model, self.pin_data, q, qd)
            pin.updateFramePlacements(self.pin_model, self.pin_data)

            H = self.pin_data.oMf[self.fid_tool0].homogeneous
            p = H[:3, 3]
            rotvec = R.from_matrix(H[:3, :3]).as_rotvec()

            v_world = pin.getFrameVelocity(
                self.pin_model,
                self.pin_data,
                self.fid_tool0,
                pin.pinocchio_pywrap_default.ReferenceFrame.WORLD,
            )
            tcp_speed = np.hstack((v_world.linear, v_world.angular))

            with self._lock:
                self.tcp_pose = np.hstack((p, rotvec))
                self.tcp_speed = tcp_speed

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[URState] joint_state_cb error: {e}")

    def get_tcp_pose_speed(self):
        with self._lock:
            if self.tcp_pose is None or self.tcp_speed is None:
                return None, None
            return self.tcp_pose.copy(), self.tcp_speed.copy()


class URMotorFeedforwardNode:
    def __init__(self):
        rospy.init_node("ur_motor_feedforward_test")

        self.ur_state = URStateFromJointStates()

        # 发布前馈角度（集成节点会订阅）
        self.feedforward_pub = rospy.Publisher("/afp/motor_target_angle", Float32, queue_size=1)

        # 订阅电机状态（用于验证）
        self.motor_pos = None
        self.motor_zero = None
        rospy.Subscriber("/motor1_status", Motor, self.motor_status_cb, queue_size=1)

        # 状态
        self.motor_target_angle = 0.0
        self.start_time = None
        self.motion_started = False

        rospy.loginfo(">>> UR→电机前馈节点已启动（使用 /joint_states 计算 TCP）<<<")

    def motor_status_cb(self, msg: Motor):
        self.motor_pos = msg.position
        if self.motor_zero is None:
            self.motor_zero = self.motor_pos
            rospy.loginfo(f"电机零点已记录: {self.motor_zero:.2f}°")

    def send_urscript_line_move(self):
        """发送URScript让机械臂走直线"""
        tcp_pose, _ = self.ur_state.get_tcp_pose_speed()
        if tcp_pose is None:
            raise RuntimeError("TCP pose not ready")

        x0, y0, z0 = tcp_pose[:3]
        rx, ry, rz = tcp_pose[3:]

        x1 = x0 + LINE_DISTANCE

        urscript = f"""
def test_line():
  set_tcp(p[0,0,0.221,0,0,0])
  p0 = p[{x0:.6f},{y0:.6f},{z0:.6f},{rx:.6f},{ry:.6f},{rz:.6f}]
  p1 = p[{x1:.6f},{y0:.6f},{z0:.6f},{rx:.6f},{ry:.6f},{rz:.6f}]
  movel(p0, a={LINE_ACCEL:.3f}, v={LINE_SPEED:.3f})
  movel(p1, a={LINE_ACCEL:.3f}, v={LINE_SPEED:.3f})
end

test_line()
"""

        rospy.loginfo(
            f"发送URScript: X {x0:.3f}→{x1:.3f} (距离={LINE_DISTANCE}m, 速度={LINE_SPEED}m/s)")

        with socket.create_connection((ROBOT_IP, 30002), timeout=2.0) as s:
            s.sendall(urscript.encode("utf-8"))

        self.start_time = time.time()
        self.motion_started = True

    def run(self):
        # 等待 motor zero + tcp ready
        rospy.loginfo("等待电机状态与 /joint_states ...")
        while not rospy.is_shutdown():
            tcp_pose, tcp_speed = self.ur_state.get_tcp_pose_speed()
            if self.motor_zero is not None and tcp_pose is not None and tcp_speed is not None:
                break
            rospy.sleep(0.05)

        rospy.sleep(0.5)
        self.send_urscript_line_move()

        rate = rospy.Rate(CONTROL_FREQUENCY)
        last_tcp_x = None
        last_loop_time = time.time()

        while not rospy.is_shutdown():
            tcp_pose, tcp_speed = self.ur_state.get_tcp_pose_speed()
            if tcp_pose is None or tcp_speed is None:
                rate.sleep()
                continue

            v_x = float(tcp_speed[0])
            tcp_x = float(tcp_pose[0])

            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now
            if dt <= 0.0 or dt > 0.2:
                dt = 1.0 / CONTROL_FREQUENCY

            motor_deg_s = linear_speed_to_output_deg_s(v_x) * FEEDFORWARD_GAIN
            delta_angle = motor_deg_s * dt
            self.motor_target_angle += delta_angle

            self.feedforward_pub.publish(Float32(data=self.motor_target_angle))

            if self.motion_started and abs(v_x) > 0.0005:
                elapsed = time.time() - self.start_time
                motor_actual_delta = (self.motor_pos - self.motor_zero) if (self.motor_pos is not None) else 0.0
                rospy.loginfo_throttle(
                    0.5,
                    f"[{elapsed:.1f}s] TCP_vx={v_x*1000:.1f}mm/s | "
                    f"Δ角度={delta_angle:.2f}°/step | "
                    f"目标={self.motor_target_angle:.1f}° | "
                    f"实际Δ={motor_actual_delta:.1f}° | "
                    f"误差={self.motor_target_angle - motor_actual_delta:.1f}°",
                )

            # 运动结束判定
            if self.motion_started and last_tcp_x is not None:
                if abs(v_x) < 0.0005 and abs(tcp_x - last_tcp_x) < 0.0001:
                    elapsed = time.time() - self.start_time
                    if elapsed > 5.0:
                        rospy.loginfo("=" * 60)
                        rospy.loginfo("运动结束！")
                        rospy.loginfo(f"总用时: {elapsed:.2f}s")
                        rospy.loginfo(f"电机目标累积角度(输出轴): {self.motor_target_angle:.1f}°")

                        expected_tape_mm = LINE_DISTANCE * 1000.0
                        target_tape_mm = self.motor_target_angle / 360.0 * ROLLER_CIRCUMFERENCE * 1000.0

                        if self.motor_pos is not None:
                            motor_actual_delta = self.motor_pos - self.motor_zero
                            actual_tape_mm = motor_actual_delta / 360.0 * ROLLER_CIRCUMFERENCE * 1000.0
                            rospy.loginfo(f"电机实际累积角度(输出轴): {motor_actual_delta:.1f}°")
                            rospy.loginfo(f"机械臂走了: {expected_tape_mm:.1f}mm")
                            rospy.loginfo(f"目标放带: {target_tape_mm:.1f}mm")
                            rospy.loginfo(f"实际放带: {actual_tape_mm:.1f}mm")
                            rospy.loginfo(f"放带误差: {target_tape_mm - actual_tape_mm:.1f}mm")
                        else:
                            rospy.loginfo(f"机械臂走了: {expected_tape_mm:.1f}mm")
                            rospy.loginfo(f"目标放带: {target_tape_mm:.1f}mm")

                        rospy.loginfo("=" * 60)
                        break

            last_tcp_x = tcp_x
            rate.sleep()

        rospy.loginfo("测试完成，节点退出")


if __name__ == "__main__":
    try:
        node = URMotorFeedforwardNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
