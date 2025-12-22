#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from dataclasses import dataclass
import pinocchio as pin

"""
测试ur5e机器人的末端姿态显示与旋转矩阵间对应关系
R(旋转矩阵)
rotVec(旋转向量)
"""
def R2rotVec(R):
    trace = np.trace(R)
    angle = np.arccos((trace - 1) / 2)

    if abs(angle) < 1e-6:
        return np.zeros(3)
    else:
        rx = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
        ry = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
        rz = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))
        axis = np.array([rx, ry, rz])
        return axis*angle

"""
将旋转向量转换为旋转矩阵
"""
def rotVec2R(rotVec):
    angle = np.linalg.norm(rotVec)
    if abs(angle) < 1e-6:
        return np.eye(3)
    axis = rotVec / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
    return R

@dataclass
class JointStates:
    position: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray

@dataclass
class CartesianState:
    # 都是基于机器人底座坐标系的
    robot_trans: np.ndarray
    robot_vel: np.ndarray 
    ee_trans: np.ndarray
    ee_vel: np.ndarray

@dataclass
class RobotState:
    joint_state: JointStates
    ee_state: CartesianState

    

MODEL_PATH = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"

class UR5eController:
    def __init__(self):
        rospy.init_node("ur5e_controller", anonymous=True)
        self.pub = rospy.Publisher(
            "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=1
        )
        
        self.robot_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.robot_data_pin = pin.Data(self.robot_model)

        self.freq = 100.0  # 控制频率100Hz
        self.rate = rospy.Rate(self.freq)

        self.max_joint_vel = 2.0  # rad/s
        self.default_vel = 1.0  # rad/s
        
        # 这是机器人按道理来说应该的关节名称顺序
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # # 实际上关节名称顺序是：
        # self.joint_names = [
        #     "elbow_joint",
        #     "shoulder_lift_joint",
        #     "shoulder_pan_joint",
        #     "wrist_1_joint",
        #     "wrist_2_joint",
        #     "wrist_3_joint"
        # ]

        self.current_command = None
        self.trajectory = None
        self.traj_index = 0

        rospy.loginfo(f"UR5e Controller Initialized, publish frquency set to {self.freq}Hz")

        self.robot_state = RobotState(
            joint_state=JointStates(
                position=np.zeros(6),
                velocity=np.zeros(6),
                effort=np.zeros(6)
            ),
            ee_state=CartesianState(
                robot_trans=np.eye(4),
                ee_trans=np.eye(4),
                robot_vel=np.zeros(3),
                ee_vel=np.zeros(3)
            )
        )

        self.robot_state_sub = rospy.Subscriber(
            "/joint_states",
            JointState,
            self.update_robot_state
        )

        

    def add_end_effector(self):
        """
        导入末端执行器，主要用于修改末端执行器的旋转矩阵
        """
        end_effector_path = "/home/lgx/Project/AFP/src/afp_mjc/env/AFPhead/urdf/AFPhead.urdf"
        ee_model = pin.buildModelFromUrdf(end_effector_path)
        self.robot_model = pin.appendModel(self.robot_model, ee_model, self.robot_model.getFrameId("flange"), pin.SE3.Identity())
        self.robot_data_pin = pin.Data(self.robot_model)
        # print(self.robot_model.nq)
        # q = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0, 0, 0, 0, 0])  # 6个关节+4个末端执行器关节
        # pin.forwardKinematics(self.robot_model, self.robot_data_pin, q)
        # pin.updateFramePlacements(self.robot_model, self.robot_data_pin)
        # ee_frame_id = self.robot_model.getFrameId("roll_link")
        # ee_pos = self.robot_data_pin.oMf[ee_frame_id]
        # print("End-Effector with added EE Position:", ee_pos)


    def update_robot_state(self, joint_msg: JointState):
        # 关节信息更新
        order = [2, 1, 0, 3, 4, 5]
        self.robot_state.joint_state.position = np.array(joint_msg.position)[order]
        self.robot_state.joint_state.velocity = np.array(joint_msg.velocity)[order]
        self.robot_state.joint_state.effort = np.array(joint_msg.effort)[order]

        # 末端执行器信息更新
        if self.robot_state.joint_state.position.shape[0] == self.robot_model.nq:
            # 说明此时没有末端执行器，直接计算
            pin.forwardKinematics(
                self.robot_model,
                self.robot_data_pin,
                self.robot_state.joint_state.position,
                self.robot_state.joint_state.velocity
            )
            flange_frame_id = self.robot_model.getFrameId("flange")
            ee_frame_id = self.robot_model.getFrameId("flange")

        elif self.robot_state.joint_state.position.shape[0] + 4 == self.robot_model.nq:
            # 如果末端执行器有4个关节，则补齐4个零值
            q_full = np.concatenate((self.robot_state.joint_state.position, np.zeros(4)))
            v_full = np.concatenate((self.robot_state.joint_state.velocity, np.zeros(4)))
            pin.forwardKinematics(
                self.robot_model,
                self.robot_data_pin,
                q_full,
                v_full
            )
            flange_frame_id = self.robot_model.getFrameId("flange")
            ee_frame_id = self.robot_model.getFrameId("roll_link")

        pin.updateFramePlacements(self.robot_model, self.robot_data_pin)
        self.robot_state.ee_state.robot_trans = self.robot_data_pin.oMf[flange_frame_id].homogeneous
        self.robot_state.ee_state.ee_trans = self.robot_data_pin.oMf[ee_frame_id].homogeneous

        self.robot_state.ee_state.robot_vel = pin.getFrameVelocity(self.robot_model, self.robot_data_pin, flange_frame_id).linear
        self.robot_state.ee_state.ee_vel = pin.getFrameVelocity(self.robot_model, self.robot_data_pin, ee_frame_id).linear

        # print("joint_pos:", self.robot_state.joint_state.position)
        # print("ee pos:", self.robot_state.ee_state.robot_trans[:3, 3])
        # print("joint vel:", self.robot_state.joint_state.velocity)
        # print("ee vel:", self.robot_state.ee_state.robot_vel)

    def test_pinocchio(self):
        q = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        pin.forwardKinematics(self.robot_model, self.robot_data_pin, q)
        pin.updateFramePlacements(self.robot_model, self.robot_data_pin)
        ee_frame_id = self.robot_model.getFrameId("flange")
        ee_pos = self.robot_data_pin.oMf[ee_frame_id]
        # print(type(ee_pos))
        # 将姿态用Rx Ry Rz表示
        # ee_pos = ee_pos.translation
        ee_rpy = pin.rpy.matrixToRpy(self.robot_data_pin.oMf[ee_frame_id].rotation)
        print("End-Effector RPY:", ee_rpy)
        # print(ee_pos.shape)
        print("End-Effector Position:", ee_pos)
        print(self.robot_data_pin.oMf[self.robot_model.getFrameId("base_link_inertia")].rotation)
        
        # 正确获取frame速度的方法
        frame_velocity = pin.getFrameVelocity(self.robot_model, self.robot_data_pin, ee_frame_id)
        ee_vel = frame_velocity.linear
        print("End-Effector Velocity:", ee_vel)

        print("joint state:", self.robot_state.joint_state)
        return ee_pos


    def move_line(self, target, velocity=None):
        """
        直线插补运动到目标位置
        target: 目标关节位置，list或np.ndarray
        velocity: 关节速度，float，单位rad/s
        """        
        if velocity is None:
            velocity = self.default_vel

        target = np.array(target)
        start_pos = np.array(self.robot_state.ee_state.robot_trans[:3, 3]).reshape(-1)
        next_pos = np.array(start_pos)
        print("next_pos:", next_pos)
        dis_start2target = np.linalg.norm(np.array(target) - start_pos)
        step_num = max(int(dis_start2target / velocity * self.freq), 1)
        step_vec = (np.array(target) - start_pos) / step_num
        for step in range(step_num):
            next_pos += step_vec
            # 计算逆运动学，得到关节位置
            ik_sol, success = self.IK(
                self.robot_model, 
                pin.SE3(self.robot_state.ee_state.robot_trans[:3, :3], next_pos), 
                "flange",
                np.concatenate((self.robot_state.joint_state.position, np.zeros(self.robot_model.nq - 6)))
            )
            if not success:
                rospy.logwarn("IK solver failed!")
            self.move_to(ik_sol[:6], velocity=velocity)
            self.rate.sleep()
        

    def IK(self, model: pin.Model, target_SE3, frame_name, q_init, max_iter=1000, eps=1e-4, damp=1e-6):
        data = model.createData()
        current_R = self.robot_state.ee_state.robot_trans[:3, :3]
        if q_init is None:
            q = self.robot_state.joint_state.position.copy()
        else:
            q = q_init.copy()
        
        frame_id = model.getFrameId(frame_name)
    

        for i in range(max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, frame_id)

            # 当前末端位姿
            current_SE3 = data.oMf[frame_id]

            # 计算误差
            err = pin.log6(current_SE3.inverse() * target_SE3).vector

            # 检查收敛
            if np.linalg.norm(err) < eps:
                return q, True
            
            # 计算雅可比矩阵
            J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)

            JTJ = J.T.dot(J) + damp * np.eye(model.nv)
            d_q = np.linalg.solve(JTJ, J.T.dot(err))

            q = pin.integrate(model, q, d_q)
        return q, False

    def move2default(self, velocity=0.2):
        self.move2point([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0], velocity=velocity)

    def move2point(self, target, velocity=None):
        if velocity is None:
            velocity = self.default_velocity

        start_pos = np.array(self.robot_state.joint_state.position)
        next_pos = np.array(start_pos)
        dis_start2target = np.linalg.norm(np.array(target) - start_pos)
        step_num = max(int(dis_start2target / velocity * self.freq), 1)
        step_vec = (np.array(target) - start_pos) / step_num
        for step in range(step_num):
            next_pos += step_vec
            self.move_to(next_pos, velocity=velocity)
            self.rate.sleep()

    def move_to_cartesian(self, target_pos, duration=None, velocity=None, wait4complete=True):
        target_pos_cartesian = np.array(target_pos)

        target_pos_joint = self.IK(self.robot_model, 
                                   pin.SE3(self.robot_state.ee_state.robot_trans[:3, :3], target_pos_cartesian), 
                                   "flange", 
                                   np.concatenate((self.robot_state.joint_state.position, np.zeros(self.robot_model.nq - 6)))
                                   )

        self.move_to(target_pos_joint[0][:6], duration=duration, velocity=velocity, wait4complete=wait4complete)

        
        


    def move_to(self, target_pos, duration=None, velocity=None, wait4complete=True):
        target_pos = np.array(target_pos)

        # get current positions (safe fallback)
        try:
            current_pos = np.array(self.robot_state.joint_state.position)
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
        p0.velocities = [0.0] * len(self.joint_names)
        p0.time_from_start = rospy.Duration(0.0)

        p1 = JointTrajectoryPoint()
        p1.positions = target_pos.astype(float).tolist()
        p1.velocities = [float(v) for v in vel_cmd]
        p1.time_from_start = rospy.Duration(duration)

        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = list(self.joint_names)
        goal.goal.trajectory.points = [p0, p1]

        # rospy.loginfo(f"Publishing trajectory with duration={duration:.3f}s, max_delta={max_delta:.3f}")
        # set a valid start time so controllers know when to begin executing
        self.pub.publish(goal)
        if wait4complete:
            rospy.sleep(duration)
        # print("Published move_to command (target positions):", p1.positions)
    

    
if __name__ == "__main__":
    controller = UR5eController()
    # ee_pos = controller.test_pinocchio()
    # test = R2rotVec(ee_pos.rotation)
    # print("RPY from transform2ur5e:", test)
    # test_R = rotVec2R(test)
    # print("Rotation matrix from R2rotVec:", test_R)
    # controller.add_end_effector()


    # while not rospy.is_shutdown():
    #     # controller.move_to([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0], velocity=1.0)
    #     controller.move_to([0, np.pi/2, -np.pi/2, 0, -np.pi/2, 0], velocity=1.0)
    #     controller.rate.sleep()

    # 等待机器人状态更新
    while controller.robot_state.joint_state.position.sum() == 0.0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot state update...")
        rospy.sleep(0.1)

    # 机器人回到默认位置
    controller.move2default()

    # # 机器人末端走直线运动， x方向前进0.2m， 然后回到原点
    # cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
    # print("Current EE Position:", cur_ee_pos)
    # target_pos = cur_ee_pos + np.array([0.2, 0.0, 0.0])
    # print("Target Position (up):", target_pos)
    # controller.move_line(target_pos, velocity=0.1)
    # target_pos = cur_ee_pos
    # print("Target Position (down):", target_pos)
    # controller.move_line(target_pos, velocity=0.1)

    # 延时等待
    print("Waiting for 1 second...")
    rospy.sleep(1.0)
    

    # 机器人走一个sin曲线轨迹 z轴
    cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
    A = 0.1
    T = 10.0 # 周期10秒
    freq = 100.0
    steps = int(T * freq)
    import time
    start_time = time.time()
    for step in range(steps):
        t = step / freq
        z_offset = A * np.sin(2 * np.pi * t / T)
        target_pos = cur_ee_pos + np.array([0.0, 0.0, z_offset]).T
        print("target_pos: ", target_pos)
        controller.move_to_cartesian(target_pos, velocity=0.1, wait4complete=False)
        controller.rate.sleep()
    end_time = time.time()
    print(f"Completed sin wave trajectory in {end_time - start_time:.2f} seconds.")
        
    rospy.spin()