#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from collections import deque
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped 
from std_msgs.msg import Float32MultiArray

MODEL_PATH = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"

class AdmittanceController:
    def __init__(self):
        rospy.init_node("admittance_controller", anonymous=True)
        
        # 1. Pinocchio 初始化
        self.robot_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.robot_data = self.robot_model.createData()
        self.ee_frame_name = "wrist_3_link" 
        self.ee_frame_id = self.robot_model.getFrameId(self.ee_frame_name)
        
        # 用于关节名映射的标准顺序
        self.sorted_joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # self.T_cam2base = np.eye(4)  # 如果需要，可以在这里设置相机到基座的变换

        # 2. 参数
        self.control_freq = 100.0
        self.reasoning_freq = 20.0
        self.interp_ratio = int(self.control_freq / self.reasoning_freq)
        
        self.rate = rospy.Rate(self.control_freq)
        
        
        
        self.admittance_gain_pos = 0.001 
        self.max_joint_vel = 2.0  # rad/s
        self.default_vel = 1.0  # rad/s



        # 状态
        self.current_q = np.zeros(6)
        self.current_pos = np.zeros(3)
        self.current_rot = np.eye(3)
        self.measured_wrench = np.zeros(6)

        # 目标
        self.target_pos = np.zeros(3)
        self.target_quat = np.array([0, 0, 0, 1.0]) 
        self.target_force = np.zeros(3)
        
        self.first_update = True 

        self.action_deque = deque(maxlen=100)

        # ================= 3. ROS 接口 (Sim2Real Ready) =================
        
        # A. 发送控制 -> 这里的名字是真机驱动监听的名字
        self.cmd_pub = rospy.Publisher(
            "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal", 
            FollowJointTrajectoryActionGoal,
            queue_size=1
        )

        # B. 接收状态 <- 这里的名字是真机驱动发布的名字
        self.joint_sub = rospy.Subscriber(
            "/joint_states", 
            JointState, 
            self.joint_callback
        )
        
        # C. 接收力 <- 无论真机还是仿真，都约定好发到这里
        self.wrench_sub = rospy.Subscriber("/mujoco/ee_wrench", WrenchStamped, self.wrench_callback)
        
        # D. 接收推理 <- 大脑发来的
        self.ai_sub = rospy.Subscriber("/il/action", Float32MultiArray, self.ai_callback)
        
        # E. 发送 Pose <- 给大脑用的 FK 结果
        self.pose_pub = rospy.Publisher("/robot/pose", PoseStamped, queue_size=10)

        rospy.loginfo("Admittance Controller Ready (Standard Interface)")

    def run(self):
        # 等待第一次数据
        while self.first_update and not rospy.is_shutdown():
            rospy.loginfo_throttle(2, "Waiting for /joint_states ...")
            self.rate.sleep()

        # 初始化目标为当前位置
        self.target_pos = self.current_pos.copy()
        self.target_quat = R.from_matrix(self.current_rot).as_quat()

        while not rospy.is_shutdown():
            if len(self.action_deque) > 0:
                next_action = self.action_deque.popleft()
                self.target_pos = next_action['pos']
                self.target_quat = next_action['quat']
                self.target_force = next_action['force']

            # 1. 导纳修正
            force_error_sensor = self.measured_wrench[:3] - self.target_force
            force_error_base = self.current_rot @ force_error_sensor
            pos_modification = self.admittance_gain_pos * force_error_base
            modified_target_pos = self.target_pos - pos_modification

            # 2. 构造目标 SE3
            R_des = R.from_quat(self.target_quat).as_matrix()
            target_se3 = pin.SE3(R_des, modified_target_pos)

            # 3. IK 解算
            q_sol, success = self.solve_ik_dls(target_se3, self.current_q)

            if success:
                self.publish_command(q_sol)
            
            self.rate.sleep()

    def joint_callback(self, msg):
        try:
            if len(msg.name) < 6: return
            
            # 鲁棒的字典映射 (防止乱序)
            joint_map = dict(zip(msg.name, msg.position))
            sorted_q = [joint_map[name] for name in self.sorted_joint_names]
            self.current_q = np.array(sorted_q)
            
            # 计算 FK 并发布给 Brain
            pin.forwardKinematics(self.robot_model, self.robot_data, self.current_q)
            pin.updateFramePlacements(self.robot_model, self.robot_data)
            
            frame_se3 = self.robot_data.oMf[self.ee_frame_id]
            self.current_pos = frame_se3.translation
            self.current_rot = frame_se3.rotation

            self.publish_pose(self.current_pos, self.current_rot)

            if self.first_update:
                self.first_update = False
        except Exception:
            pass

    def publish_pose(self, pos, rot_mat):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base"
        msg.pose.position.x = pos[0]; msg.pose.position.y = pos[1]; msg.pose.position.z = pos[2]
        q = R.from_matrix(rot_mat).as_quat()
        msg.pose.orientation.x = q[0]; msg.pose.orientation.y = q[1]; msg.pose.orientation.z = q[2]; msg.pose.orientation.w = q[3]
        self.pose_pub.publish(msg)

    def wrench_callback(self, msg):
        self.measured_wrench = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ])

    def ai_callback(self, msg):
        data = np.array(msg.data)
        if len(data) != 10 * 13 + 1:
            print(f"警告: 接收到的动作数据长度 {len(data)} 不匹配预期 {10*13+1}")
            return
        # 1. 提取时间戳
        obs_timestamp = data[0]
        action_flat = data[1:]

        # 2. 重塑为 (10, 13)
        action_seq = action_flat.reshape((10, 13))
        
        current_time = rospy.Time.now().to_sec()
        latency = current_time - obs_timestamp

        steps2drop = int(max(latency * self.control_freq, 0))

        dense_trajectory = self.interpolate_action_sequence(action_seq)

        if steps2drop < len(dense_trajectory):
            valid_actions = dense_trajectory[steps2drop:]
        else:
            rospy.logwarn(f"Latency too high ({latency:.3f}s), dropping all actions")
            return
        
        self.action_deque.clear()
        self.action_deque.extend(valid_actions)
    
    def interpolate_action_sequence(self, action_seq):
        dense_trajectory = []
        for i in range(len(action_seq)):
            current_action = action_seq[i]
            next_act = action_seq[i+1] if i < len(action_seq)-1 else current_action

            for j in range(self.interp_ratio):
                alpha = float(j) / float(self.interp_ratio)

                try:
                    slerp = Slerp([0, 1], R.from_quat([current_action[3:7], next_act[3:7]]))
                    interp_rot = slerp([alpha]).as_quat()[0]
                except Exception:
                    interp_rot = current_action[3:7]
                interp_pos = (1 - alpha) * current_action[0:3] + alpha * next_act[0:3]
                interp_force = (1 - alpha) * current_action[7:10] + alpha * next_act[7:10]
                interp_action = {
                    'pos': interp_pos,
                    'quat': interp_rot,
                    'force': interp_force
                }
                dense_trajectory.append(interp_action)
        print(f"Interpolated to {len(dense_trajectory)} actions")

        return dense_trajectory



    def solve_ik_dls(self, target_se3, q_init, max_iter=20, eps=1e-4, damp=1e-3):
        q = q_init.copy()
        for i in range(max_iter):
            pin.forwardKinematics(self.robot_model, self.robot_data, q)
            pin.updateFramePlacement(self.robot_model, self.robot_data, self.ee_frame_id)
            # 当前末端位姿
            current_SE3 = self.robot_data.oMf[self.ee_frame_id]
            # 计算误差
            err = pin.log6(current_SE3.inverse() * target_se3).vector
            if np.linalg.norm(err) < eps: return q, True
            J = pin.computeFrameJacobian(self.robot_model, self.robot_data, q, self.ee_frame_id, pin.LOCAL)
            dq = np.linalg.solve(J.T.dot(J) + damp * np.eye(6), J.T.dot(err))
            q = pin.integrate(self.robot_model, q, dq)
        return q, False

    def publish_command(self, target_pos, duration=None, velocity=None, wait4complete=False):
        target_pos = np.array(target_pos)

        # get current positions (safe fallback)
        try:
            current_pos = np.array(self.current_q)
        except Exception:
            current_pos = np.zeros_like(target_pos)

        # ensure shapes match
        if current_pos.shape != target_pos.shape:
            # try to broadcast or resize
            try:
                current_pos = np.resize(current_pos, target_pos.shape)
            except Exception:
                current_pos = np.zeros_like(target_pos)

        diffs = target_pos - current_pos
        max_delta = float(np.max(np.abs(diffs))) if diffs.size > 0 else 0.0

        # default velocity and duration
        if velocity is None:
            velocity = self.max_joint_vel

        if duration is None:
            # compute duration so that max_delta/velocity gives reasonable time
            duration = 0.01 if max_delta <= 1e-6 else max(0.01, max_delta / float(max(1e-6, float(velocity))))

        # compute per-joint velocity command and clamp to max_joint_vel
        vel_cmd = diffs / max(1e-6, float(duration))
        vel_cmd = np.clip(vel_cmd, -self.max_joint_vel, self.max_joint_vel)

        # Build a two-point trajectory: start at current_pos @ t=0, reach target_pos @ t=duration
        p0 = JointTrajectoryPoint()
        p0.positions = current_pos.astype(float).tolist()
        p0.velocities = [0.0] * len(self.sorted_joint_names)
        p0.time_from_start = rospy.Duration(0.0)

        p1 = JointTrajectoryPoint()
        p1.positions = target_pos.astype(float).tolist()
        p1.velocities = [float(v) for v in vel_cmd]
        p1.time_from_start = rospy.Duration(duration)

        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = list(self.sorted_joint_names)
        goal.goal.trajectory.points = [p0, p1]

        # rospy.loginfo(f"Publishing trajectory with duration={duration:.3f}s, max_delta={max_delta:.3f}")
        # set a valid start time so controllers know when to begin executing
        self.cmd_pub.publish(goal)
        if wait4complete:
            rospy.sleep(duration)

if __name__ == "__main__":
    try:
        AdmittanceController().run()
    except rospy.ROSInterruptException:
        pass