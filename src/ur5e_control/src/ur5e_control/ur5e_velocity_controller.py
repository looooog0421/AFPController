#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UR5e 速度控制器基类
- 通过 joint_group_vel_controller 发送关节速度指令
- 大幅运动（初始定位）使用 scaled_pos_joint_traj_controller (action server)
- 高频伺服控制使用 /joint_group_vel_controller/command topic
- 同时兼容真实机器人和 MuJoCo 仿真（仿真侧订阅同一 topic 写入 ctrl_q）
"""

import numpy as np
import rospy
import time
import os
import sys

import pinocchio as pin
from scipy.spatial.transform import Rotation as R
import actionlib
import qpsolvers

from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from dataclasses import dataclass, field
import threading

# 工具类
sys.path.insert(0, os.path.dirname(__file__))
from lowpass_filter import LowPassOnlineFilter
from pinocchio_kinematic import Kinematics

# ------------------------------------ 数据类 ------------------------------------
@dataclass
class JointStates:
    position: np.ndarray = field(default_factory=lambda: np.zeros(6))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    effort:   np.ndarray = field(default_factory=lambda: np.zeros(6))

@dataclass
class CartesianState:
    robot_trans: np.ndarray = field(default_factory=lambda: np.eye(4))   # base frame
    robot_vel:   np.ndarray = field(default_factory=lambda: np.zeros(6))  # base frame [v, w]
    ee_trans:    np.ndarray = field(default_factory=lambda: np.eye(4))   # ee frame
    ee_vel:      np.ndarray = field(default_factory=lambda: np.zeros(6))  # ee frame   [v, w]

@dataclass
class RobotState:
    joint_state: JointStates   = field(default_factory=JointStates)
    ee_state:    CartesianState = field(default_factory=CartesianState)

# ------------------------------------ 常量 ------------------------------------

MODEL_PATH = os.path.join(
    os.path.expanduser("~"),
    "Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
)

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]

# /joint_states 中关节顺序到标准顺序的映射
# JOINT_ORDER = [2, 1, 0, 3, 4, 5]

DEFAULT_JOINT_POS = [np.pi/4, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/4]

POS_CTRL_ACTION  = "/scaled_pos_joint_traj_controller/follow_joint_trajectory"
VEL_CTRL_TOPIC   = "/joint_group_vel_controller/command"
POS_CTRL_NAME    = "scaled_pos_joint_traj_controller"
VEL_CTRL_NAME    = "joint_group_vel_controller"

# ------------------------------------ 控制器基类 ------------------------------------

class UR5eController:
    """
    UR5e 控制器基类，实现纯位置（速度）控制
    """

    def __init__(self,
                 node_name      = "ur5e_velocity_controller",
                 control_freq   = 200.0,
                 servo_kp       = 8.0,
                 servo_kd       = 0.5,
                 max_joint_vel  = [0.2, 0.2, 0.2, 0.5, 0.5, 0.5],
                 tracking_topic = "/reference_trajectory",
                 wrench_topic   = "/mujoco/ee_wrench",
                 filter_tau     = 0.05):
        """
        参数：
            node_name: ROS节点名称
            control_freq: 控制频率（Hz）
            servo_kp: 伺服控制的比例增益
            servo_kd: 伺服控制的微分增益
            max_joint_vel: 最大关节速度（rad/s）
            tracking_topic: 轨迹跟踪的输入 topic，类型为 JointTrajectoryPoint
            wrench_topic: 末端执行器力传感器数据的 topic，类型为 WrenchStamped
            filter_tau: 末端执行器力数据的低通滤波时间常数（秒）
        """
        # ── ROS 节点 ──
        try:
            rospy.init_node(node_name, anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        # ── 控制参数 ──
        self.freq = control_freq
        self.dt = 1.0 / control_freq
        self.rate = rospy.Rate(control_freq)

        self.servo_kp = servo_kp
        self.servo_kd = servo_kd
        self.max_joint_vel = max_joint_vel

        # ── 运动学模型 ──

        self.pin_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.pin_data = pin.Data(self.pin_model)

        # ── 机器人状态 ──
        self.robot_state = RobotState()
        self.joint_idx_mapping = None  
        self.kinematics_lock = threading.Lock()

        # —— 力传感器 ——
        self.current_wrench = np.zeros(6)  # [fx, fy, fz, tx, ty, tz]
        self.wrench_received = False
        self._wrench_filter = LowPassOnlineFilter(
            dimension=6,
            tau=filter_tau,
            dt=self.dt,
            initial_states=np.zeros(6)
        )

        # —— 轨迹跟踪 ——
        self.reference_pose = None
        self.tracking_topic = tracking_topic
        self._tracking_sub = None
        self.trajectory_tracking_enabled = False

        # —— Action Client ——（位置控制）
        rospy.loginfo(f"Waiting for action server {POS_CTRL_ACTION}...")
        self._pos_client = actionlib.SimpleActionClient(POS_CTRL_ACTION, FollowJointTrajectoryAction)
        connected = self._pos_client.wait_for_server(rospy.Duration(5.0))
        if connected:
            rospy.loginfo(f"Connected to action server {POS_CTRL_ACTION}")
        else:
            rospy.logwarn(f"Failed to connect to action server {POS_CTRL_ACTION}")
        
        # —— Publisher ——（速度控制）
        self._vel_pub = rospy.Publisher(VEL_CTRL_TOPIC, Float64MultiArray, queue_size=1)

        # —— 触发推理发布者 ——（外部触发器）
        self.trigger_pub = rospy.Publisher("/trigger_inference", Bool, queue_size=1)

        # —— 订阅末端执行器力传感器数据 ——
        rospy.Subscriber("/joint_states", JointState, self._joint_state_callback)
        rospy.Subscriber(wrench_topic, WrenchStamped, self._wrench_callback)

        rospy.on_shutdown(self._safe_shutdown_hook)


        rospy.loginfo(
            f"[{node_name}] Initialized | freq={control_freq}Hz | "
            f"kp={servo_kp} | kd={servo_kd} | max_vel={max_joint_vel} rad/s"
        )


    # ======================= ROS 回调函数 =======================
    
    def _joint_state_callback(self, msg: JointState):
        """
        关节状态回调函数，更新机器人状态中的关节位置、速度和力矩
        """
        try:
            if self.joint_idx_mapping is None:
                name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
                mapping = []
                for joint_name in JOINT_NAMES:
                    if joint_name not in name_to_idx:
                        return
                    mapping.append(name_to_idx[joint_name])
                self.joint_idx_mapping = mapping
                rospy.loginfo(f"Joint index mapping established: {self.joint_idx_mapping}")

            pos = np.array(msg.position)[self.joint_idx_mapping]
            vel = np.array(msg.velocity)[self.joint_idx_mapping]
            eff = np.array(msg.effort)[self.joint_idx_mapping]

            with self.kinematics_lock:
                self.robot_state.joint_state.position = pos
                self.robot_state.joint_state.velocity = vel
                self.robot_state.joint_state.effort   = eff

                # 正向运动学求解末端执行器位姿和速度
                pin.forwardKinematics(self.pin_model, self.pin_data, pos, vel)
                pin.updateFramePlacements(self.pin_model, self.pin_data)

                fid = self.pin_model.getFrameId("tool0")
                self.robot_state.ee_state.robot_trans = self.pin_data.oMf[fid].homogeneous

                ## 获取末端执行器在世界坐标系和局部坐标系下的速度
                v_world = pin.getFrameVelocity(
                    self.pin_model, self.pin_data, fid, pin.pinocchio_pywrap_default.ReferenceFrame.WORLD)
                self.robot_state.ee_state.robot_vel = np.hstack((v_world.linear, v_world.angular))

                v_ee = pin.getFrameVelocity(
                    self.pin_model, self.pin_data, fid, pin.pinocchio_pywrap_default.ReferenceFrame.LOCAL)
                self.robot_state.ee_state.local_vel = np.hstack((v_ee.linear, v_ee.angular))

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"joint_state_callback error: {e}")

    def _wrench_callback(self, msg: WrenchStamped):
        """
        力传感器回调函数，更新当前末端执行器的力数据，并进行低通滤波
        """
        try:
            raw_wrench = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ])
            self.current_wrench = self._wrench_filter.update(raw_wrench)
            self.wrench_received = True
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"wrench_callback error: {e}")
    
    def _reference_trajectory_callback(self, msg: PoseStamped):
        """
        轨迹跟踪回调函数，接收目标位姿并启用轨迹跟踪控制
        """
        self.reference_pose = msg

    # ======================= 切换控制器 =======================
    def _switch_controller(self, start_controllers, stop_controllers):
        
        try:
            rospy.wait_for_service('/controller_manager/switch_controller', timeout=3.0)
            switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = SwitchControllerRequest()
            req.start_controllers = start_controllers
            req.stop_controllers = stop_controllers
            req.strictness = SwitchControllerRequest.STRICT
            resp = switch(req)
            return resp.ok
        except Exception as e:
            rospy.logerr(f"Failed to switch controllers: {e}")
            return False
        
    def switch_to_position_control(self):
        ok = self._switch_controller(
            start_controllers=[POS_CTRL_NAME],
            stop_controllers=[VEL_CTRL_NAME]
        )
        if ok:
            rospy.loginfo("Switched to position control")
        else:
            rospy.logerr("Failed to switch to position control")
        return ok
    
    def switch_to_velocity_control(self):
        ok = self._switch_controller(
            start_controllers=[VEL_CTRL_NAME],
            stop_controllers=[POS_CTRL_NAME]
        )
        if ok:
            rospy.loginfo("Switched to velocity control")
        else:
            rospy.logerr("Failed to switch to velocity control")
        return ok
    
    # ======================= 位置控制器 =======================
    def move_to(self, target_joint_pos, velocity=0.2, duration=None, wait4complete=True):
        """
        使用位置控制器将机械臂移动到目标关节位置
        Parameters:
            target_joint_pos: 目标关节位置，长度为6的数组
            velocity: 运动速度（0-1之间），控制运动的快慢
            duration: 运动持续时间（秒），如果为None则由控制器自动规划
            wait4_completion: 是否等待运动完成后再返回
        """
        target = np.array(target_joint_pos, dtype=np.float64)
        current = self.robot_state.joint_state.position.copy()

        diff = target - current
        max_delta = np.max(np.abs(diff))

        if velocity is None: velocity = self.max_joint_vel

        if duration is None:
            duration = 0.01 if max_delta <= 1e-6 else max(0.01, max_delta / float(velocity))
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = JOINT_NAMES

        p0 = JointTrajectoryPoint()
        p0.positions = current.tolist()
        p0.velocities = [0.0] * len(current)
        p0.time_from_start = rospy.Duration(0.0)

        # 如果超速了，进行速度限制，等比例缩放速度，max_joint_val为长度为6的数组，分别对应每个关节的速度限制
        max_vel = np.array(self.max_joint_vel)
        vel_cmd = diff / max(duration, 1e-3)
        scale_factor = np.min(np.abs(max_vel / vel_cmd))
        if scale_factor < 1.0:
            vel_cmd *= scale_factor

        p1 = JointTrajectoryPoint()
        p1.positions = target.tolist()
        p1.velocities = vel_cmd.tolist()
        p1.time_from_start = rospy.Duration(duration)

        goal.trajectory.points = [p0, p1]
        self._pos_client.send_goal(goal)

        if wait4complete:
            done = self._pos_client.wait_for_result(rospy.Duration(duration + 1.0))
            if not done:
                rospy.logwarn("move_to: timeout!")
                return False
            return self._pos_client.get_state() == actionlib.GoalStatus.SUCCEEDED
        return True

    def move_to_cartesian(self, target_pos, target_rot, velocity=0.2, duration=None, wait4complete=True):
        
        
        if target_rot is None:
            target_rot = self.robot_state.ee_state.robot_trans[:3, :3]
        
        if isinstance(target_rot, np.ndarray) and target_rot.shape == (4,):
            q = target_rot
            target_rot = R.from_quat(q).as_matrix()
        elif isinstance(target_rot, np.ndarray) and target_rot.shape == (3,3):
            pass
        else:
            raise ValueError("target_rot must be a quaternion (4,) or rotation matrix (3,3)")
        
        target_pos_cartesian = np.array(target_pos)

        print(f"Reference Rot(quat):\n{R.from_matrix(target_rot).as_quat()}")
        target_pos_joint, success = self.IK(self.pin_model, 
                                   pin.SE3(target_rot, target_pos_cartesian), 
                                   "tool0", 
                                   np.concatenate((self.robot_state.joint_state.position, np.zeros(self.pin_model.nq - 6)))
                                   )
        if success:
            self.move_to(target_pos_joint[:6], duration=duration, velocity=velocity, wait4complete=wait4complete)
        else:
            rospy.logwarn("IK Failed in move_to_cartesian")

    def move2default(self, velocity=0.3, wait4complete=True):
        rospy.loginfo("Moving to default pose...")
        return self.move_to(DEFAULT_JOINT_POS, velocity=velocity, wait4complete=wait4complete)

    # ======================== 速度控制器 ========================
    
    def servo_joint_pos(self, target_joint_pos):
        """
        伺服控制器，计算并发布关节位置指令
        Parameters:
            target_joint_pos: 目标关节位置，长度为6的数组
        """
        q = self.robot_state.joint_state.position
        # dq = self.robot_state.joint_state.velocity
        error = np.asanyarray(target_joint_pos) - q

        vel_cmd = self.servo_kp * error

        deadband = 1e-4
        vel_cmd[np.abs(error) < deadband] = 0.0
        
        self.servo_joint_command(vel_cmd)


    def servo_joint_command(self, target_joint_vel):
        
        scale_factor = np.min(np.abs(np.array(self.max_joint_vel) / target_joint_vel))
        if scale_factor < 1.0:
            vel_cmd = scale_factor * target_joint_vel
        else:
            vel_cmd = target_joint_vel

        # print(f"max_joint_vel: {self.max_joint_vel}")
        # print(f"Servo Command: {vel_cmd}")
        msg = Float64MultiArray()
        msg.data = vel_cmd.tolist()
        self._vel_pub.publish(msg)

    def stop_servo(self):
        self.servo_joint_command(np.zeros(6))

    def _safe_shutdown_hook(self):
        rospy.loginfo("Shutting down, stopping servo...")
        self.stop_servo()

        rospy.sleep(0.5)  # 确保命令发送出去

        self.switch_to_position_control()

        rospy.loginfo("Shutdown complete.")

    def servo_cartesian_pos(self, target_pos, target_rot):
        
        with self.kinematics_lock:
            current_pos = self.robot_state.ee_state.robot_trans[:3, 3]
            current_rot = self.robot_state.ee_state.robot_trans[:3, :3]
            q = self.robot_state.joint_state.position

            pin.computeJointJacobians(self.pin_model, self.pin_data, q)
            pin.updateFramePlacement(self.pin_model, self.pin_data, self.pin_model.getFrameId("tool0"))
            J = pin.computeFrameJacobian(self.pin_model, self.pin_data, q, self.pin_model.getFrameId("tool0"), pin.LOCAL_WORLD_ALIGNED)

        err_pos =  target_pos - current_pos

        err_rot_matrix = current_rot.T @ target_rot
        err_rot = current_rot @ pin.log3(err_rot_matrix)

        err_twist = np.hstack((err_pos, err_rot))

        v_cartesian = self.servo_kp * err_twist
        
        max_linear_vel = 0.1
        max_angular_vel = 0.5
        v_cartesian[:3] = np.clip(v_cartesian[:3], -max_linear_vel, max_linear_vel)
        v_cartesian[3:] = np.clip(v_cartesian[3:], -max_angular_vel, max_angular_vel)

        damp = 1e-4
        JTT = J.T @ J + damp * np.eye(self.pin_model.nv)
        vel_cmd = np.linalg.solve(JTT, J.T @ v_cartesian)

        self.servo_joint_command(vel_cmd[:6])

    # ======================= IK =======================

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
            q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)
        return q, False
    
    def IK_QP(self, model, target_SE3, frame_name, q_init, max_iter=1000, eps=1e-4, damp=1e-6):
        data = model.createData()
        if q_init is None: q = self.robot_state.joint_state.position.copy()
        else: q = q_init.copy()

        frame_id = model.getFrameId(frame_name)

        q_min = model.lowerPositionLimit
        q_max = model.upperPositionLimit

        max_step = 0.1

        for i in range(max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, frame_id)

            err = pin.log6(data.oMf[frame_id].inverse() * target_SE3).vector
            if np.linalg.norm(err) < eps: return q, True

            J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)

            P = J.T.dot(J) + damp * np.eye(model.nv)
            q_vec = -J.T.dot(err)

            lb = np.maximum(q_min - q, -max_step)
            ub = np.minimum(q_max - q, max_step)

            try:
                d_q = qpsolvers.solve_qp(P, q_vec, lb=lb, ub=ub, solver="quadprog")
            except Exception as e:
                print(f"QP solver error: {e}")
                return q, False
            
            if d_q is None:
                print(f"QP Optimization failed (Infeasible) at iter {i}.")
                break

            q = pin.integrate(model, q, d_q)
        return q, False

    # ======================= 参考轨迹跟踪 =======================

    def enable_trajectory_tracking(self, topic_name=None):
        """
        启用轨迹跟踪控制，订阅目标位姿 topic
        """
        if topic_name is None:
            topic_name = self.tracking_topic
        
        if not self.trajectory_tracking_enabled:
            self._tracking_sub = rospy.Subscriber(
                topic_name, PoseStamped, self._reference_trajectory_callback, queue_size=1)
            self.trajectory_tracking_enabled = True
            rospy.loginfo(f"Trajectory tracking enabled, subscribed to {topic_name}")
    
    def disable_trajectory_tracking(self):
        """
        禁用轨迹跟踪控制，取消订阅目标位姿 topic
        """
        if self.trajectory_tracking_enabled and self._tracking_sub is not None:
            self._tracking_sub.unregister()
            self._tracking_sub = None
            self.trajectory_tracking_enabled = False
            rospy.loginfo("Trajectory tracking disabled")

    def compute_servo_command(self, target_pos, target_rot):
        """
        计算伺服控制命令，基于当前位姿和目标位姿的误差
        Parameters:
            target_pos: (3, ) 目标位置
            target_rot: (3, 3) 目标旋转矩阵
        
        """
        self.servo_cartesian_pos(target_pos=target_pos, target_rot=target_rot)

        return target_pos, target_rot
    
    def run_tracking_loop(self):
        """
        轨迹跟踪控制循环，持续计算并发布控制命令
        """
        rospy.loginfo("Starting trajectory tracking loop...")
        while self.reference_pose is None and not rospy.is_shutdown():
            self.trigger_pub.publish(Bool(data=True))  # 触发外部推理
            self.rate.sleep()
        
        loop_count = 0
        last_log_time = time.time()
        last_loop_time = time.time()

        while not rospy.is_shutdown():
            loop_start_time = time.time()

            try:
                if self.reference_pose is not None:
                    p = self.reference_pose.pose.position
                    o = self.reference_pose.pose.orientation
                    ref_pos = np.array([p.x, p.y, p.z])

                    quat_norm = np.linalg.norm([o.x, o.y, o.z, o.w])

                    if quat_norm > 0.9:
                        ref_rot = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
                    else:
                        # 姿态无效，保持当前
                        ref_rot = self.robot_state.ee_state.robot_trans[:3, :3]

                    
                    self.compute_servo_command(ref_pos, ref_rot)
                    
                    loop_count += 1

                    now = time.time()
                    if now - last_log_time >= 1.0:
                        dt = loop_start_time - last_loop_time
                        freq = 1.0 / dt if dt > 1e-6 else float('inf')
                        ee_pos = self.robot_state.ee_state.robot_trans[:3, 3]
                        rospy.loginfo(
                            f"Freq: {freq:.1f}Hz | "
                            f"EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] | "
                            f"Ref: [{ref_pos[0]:.3f}, {ref_pos[1]:.3f}, {ref_pos[2]:.3f}]"
                        )
                        last_log_time = now

                    last_loop_time = loop_start_time

                else:
                    self.trigger_pub.publish(Bool(data=True))  # 触发外部推理

            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Error in tracking loop: {e}")

            self.rate.sleep()

    # ═══════════════════════════════════════════════════════════
    #  工具函数
    # ═══════════════════════════════════════════════════════════

    def wait_for_robot_state(self, timeout: float = 5.0) -> bool:
        """等待关节状态非零"""
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if np.any(self.robot_state.joint_state.position != 0.0):
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn(f"Timeout waiting for robot state ({timeout}s)")
                return False
            rospy.sleep(0.05)
        return False
    

if __name__ == "__main__":
    controller = UR5eController(
        node_name="ur5e_vel_controller",
        control_freq=200.0,
        tracking_topic="/reference_trajectory"
    )

    if not controller.wait_for_robot_state(timeout=3.0):
        rospy.logerr("Failed to get initial robot state, exiting.")
        sys.exit(1)

    # 1. 切换到位置控制，移动到初始位置
    controller.switch_to_position_control()
    controller.move2default(velocity=0.5, wait4complete=True)
    rospy.sleep(1.0)


    rospy.loginfo("Moving to above the mold...")
    controller.move_to_cartesian(
        # target_pos=np.array([-0.54936, -0.20258, 0.00463]),
        target_pos=np.array([-0.3, -0.3, 0.4]),
        
        target_rot=R.from_euler('xyz', [0, 180, 0], degrees=True).as_matrix(),
        wait4complete=True
    )

    # 2. 切换到速度控制，启用轨迹跟踪
    controller.switch_to_velocity_control()
    controller.enable_trajectory_tracking("/reference_trajectory")
    controller.run_tracking_loop()