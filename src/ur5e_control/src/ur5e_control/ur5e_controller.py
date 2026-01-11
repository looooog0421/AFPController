#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import time # 引入 time 模块
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from dataclasses import dataclass
import pinocchio as pin
import os

# ... [保留原本的辅助函数 R2rotVec, rotVec2R, dataclasses 等] ...
# 为了节省篇幅，这里假设辅助函数和 DataClass 代码与你原文件一致，未改动
# 重点修改了 main 下面的循环

def R2rotVec(R):
    trace = np.trace(R)
    angle = np.arccos((trace - 1) / 2)
    if abs(angle) < 1e-6: return np.zeros(3)
    rx = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
    ry = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
    rz = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))
    return np.array([rx, ry, rz]) * angle

def rotVec2R(rotVec):
    angle = np.linalg.norm(rotVec)
    if abs(angle) < 1e-6: return np.eye(3)
    axis = rotVec / angle
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

@dataclass
class JointStates:
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray

@dataclass
class CartesianState:
    robot_trans: np.ndarray
    robot_vel: np.ndarray 
    ee_trans: np.ndarray
    ee_vel: np.ndarray

@dataclass
class RobotState:
    joint_state: JointStates
    ee_state: CartesianState

# 自动获取路径
MODEL_PATH = os.path.join(os.path.expanduser("~"), "AFPController/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf")

class UR5eController:
    def __init__(self):
        try:
            rospy.init_node("ur5e_controller", anonymous=True)
        except rospy.exceptions.ROSException:
            pass 
        self.pub = rospy.Publisher(
            "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=1
        )
        
        self.robot_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.robot_data_pin = pin.Data(self.robot_model)

        self.freq = 100.0  # 控制频率100Hz
        self.rate = rospy.Rate(self.freq)
        self.max_joint_vel = 2.0
        self.default_vel = 1.0
        
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        self.robot_state = RobotState(
            joint_state=JointStates(np.zeros(6), np.zeros(6), np.zeros(6)),
            ee_state=CartesianState(np.eye(4), np.zeros(3), np.eye(4), np.zeros(3))
        )

        self.robot_state_sub = rospy.Subscriber("/joint_states", JointState, self.update_robot_state)
        rospy.loginfo(f"UR5e Controller Initialized, freq={self.freq}Hz")

    def update_robot_state(self, joint_msg: JointState):
        order = [2, 1, 0, 3, 4, 5] # 根据你的实际映射调整
        try:
            self.robot_state.joint_state.position = np.array(joint_msg.position)[order]
            self.robot_state.joint_state.velocity = np.array(joint_msg.velocity)[order]
            self.robot_state.joint_state.effort = np.array(joint_msg.effort)[order]

            pin.forwardKinematics(self.robot_model, self.robot_data_pin, 
                                  self.robot_state.joint_state.position, 
                                  self.robot_state.joint_state.velocity)
            pin.updateFramePlacements(self.robot_model, self.robot_data_pin)
            
            flange_id = self.robot_model.getFrameId("flange")
            self.robot_state.ee_state.robot_trans = self.robot_data_pin.oMf[flange_id].homogeneous
            self.robot_state.ee_state.robot_vel = pin.getFrameVelocity(self.robot_model, self.robot_data_pin, flange_id).linear
        except Exception:
            pass

    # ... [保留 IK, move_to, move_line, move2default 等函数不变] ...
    # 这里的 IK 和运动函数不需要改动，直接略去以节省空间
    def IK(self, model, target_SE3, frame_name, q_init, max_iter=1000, eps=1e-4, damp=1e-6):
        data = model.createData()
        current_R = self.robot_state.ee_state.robot_trans[:3, :3]
        if q_init is None: q = self.robot_state.joint_state.position.copy()
        else: q = q_init.copy()
        frame_id = model.getFrameId(frame_name)
        for i in range(max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, frame_id)
            err = pin.log6(data.oMf[frame_id].inverse() * target_SE3).vector
            if np.linalg.norm(err) < eps: return q, True
            J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)
            JTJ = J.T.dot(J) + damp * np.eye(model.nv)
            d_q = np.linalg.solve(JTJ, J.T.dot(err))
            q = pin.integrate(model, q, d_q)
        return q, False

    def move2default(self, velocity=0.2):
        self.move_to([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0], velocity=velocity)

    def move_to_cartesian(self, target_pos, duration=None, velocity=None, wait4complete=True):
        target_pos_cartesian = np.array(target_pos)
        target_pos_joint, success = self.IK(self.robot_model, 
                                   pin.SE3(self.robot_state.ee_state.robot_trans[:3, :3], target_pos_cartesian), 
                                   "flange", 
                                   np.concatenate((self.robot_state.joint_state.position, np.zeros(self.robot_model.nq - 6)))
                                   )
        if success:
            self.move_to(target_pos_joint[:6], duration=duration, velocity=velocity, wait4complete=wait4complete)
        else:
            rospy.logwarn("IK Failed in move_to_cartesian")

    def move_to(self, target_pos, duration=None, velocity=None, wait4complete=True):
        target_pos = np.array(target_pos)
        try: current_pos = np.array(self.robot_state.joint_state.position)
        except: current_pos = np.zeros_like(target_pos)
        
        if velocity is None: velocity = self.max_joint_vel
        diffs = target_pos - current_pos
        max_delta = float(np.max(np.abs(diffs)))
        if duration is None:
            duration = 0.01 if max_delta <= 1e-6 else max(0.01, max_delta / float(velocity))
        
        vel_cmd = diffs / max(1e-6, float(duration))
        vel_cmd = np.clip(vel_cmd, -self.max_joint_vel, self.max_joint_vel)
        
        p0 = JointTrajectoryPoint()
        p0.positions = current_pos.astype(float).tolist(); p0.velocities = [0.0]*6; p0.time_from_start = rospy.Duration(0.0)
        p1 = JointTrajectoryPoint()
        p1.positions = target_pos.astype(float).tolist(); p1.velocities = vel_cmd.tolist(); p1.time_from_start = rospy.Duration(duration)
        
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = self.joint_names
        goal.goal.trajectory.points = [p0, p1]
        self.pub.publish(goal)
        if wait4complete: rospy.sleep(duration)

if __name__ == "__main__":
    controller = UR5eController()

    while controller.robot_state.joint_state.position.sum() == 0.0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot state update...")
        rospy.sleep(0.1)

    controller.move2default()
    rospy.sleep(1.0)
    
    # 机器人走一个sin曲线轨迹 z轴
    cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
    A = 0.1
    T = 10.0 
    freq = 100.0
    steps = int(T * freq)
    
    rospy.loginfo("Starting Sine Wave...")
    
    # ================= 修改开始：频率监测 =================
    loop_count = 0
    last_print_time = time.time()
    
    for step in range(steps):
        step_start_time = time.time() # 记录循环开始时间
        
        t = step / freq
        z_offset = A * np.sin(2 * np.pi * t / T)
        target_pos = cur_ee_pos + np.array([0.0, 0.0, z_offset]).T
        
        # 发送非阻塞命令
        controller.move_to_cartesian(target_pos, velocity=0.1, wait4complete=False)
        
        # 维持频率
        controller.rate.sleep()
        
        # 计算实际频率并打印
        loop_duration = time.time() - step_start_time
        loop_count += 1
        
        # 每隔 1 秒打印一次频率信息
        if time.time() - last_print_time >= 1.0:
            if loop_duration > 0:
                current_freq = 1.0 / loop_duration
                rospy.loginfo(f"UR5e Control Loop Freq: {current_freq:.2f} Hz")
            last_print_time = time.time()
    # ================= 修改结束 =================
    
    print(f"Completed sin wave trajectory.")
    rospy.spin()