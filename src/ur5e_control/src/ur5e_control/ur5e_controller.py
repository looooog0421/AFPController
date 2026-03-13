#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import time # 引入 time 模块
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from dataclasses import dataclass
import pinocchio as pin
import os
from scipy.spatial.transform import Rotation as R
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
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
MODEL_PATH = os.path.join(os.path.expanduser("~"), "Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf")

class UR5eController:
    """UR5e控制器基类
    
    提供基础功能：
    - 机器人状态更新
    - IK求解
    - 运动控制（move_to, move_to_cartesian）
    - 轨迹跟踪
    
    可被子类继承以实现不同控制策略（位置控制、阻抗控制等）
    """
    
    def __init__(self, 
                 node_name="ur5e_controller",
                 control_freq=100.0,
                 enable_tracking=False, 
                 tracking_topic="/reference_trajectory", 
                 tracking_velocity=0.5):
        """初始化UR5e控制器
        
        Args:
            node_name: ROS节点名称
            control_freq: 控制频率 (Hz)
            enable_tracking: 是否启用轨迹跟踪模式 (True=自动跟踪, False=过程化编程)
            tracking_topic: 轨迹跟踪话题名称
            tracking_velocity: 轨迹跟踪速度
        """
        try:
            rospy.init_node(node_name, anonymous=True)
        except rospy.exceptions.ROSException:
            pass 
        
        # self.pub = rospy.Publisher(
        #     "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
        #     FollowJointTrajectoryActionGoal,
        #     queue_size=1
        # )

        self.trigger_pub = rospy.Publisher('trigger_inference', Bool, queue_size=1)

        # 1. 定义 Action Client
        # 注意 topic 名字：去掉末尾的 "/goal"。
        # Client 会自动去找 /goal, /result, /feedback 等子话题
        action_topic = "/scaled_pos_joint_traj_controller/follow_joint_trajectory"
        
        self.client = actionlib.SimpleActionClient(
            action_topic, 
            FollowJointTrajectoryAction
        )
        
        # 2. 等待服务器上线 (这是解决“机器人不动”的最关键一步)
        rospy.loginfo(f"Waiting for action server: {action_topic}")
        server_connected = self.client.wait_for_server(timeout=rospy.Duration(5.0))
        
        if not server_connected:
            rospy.logerr("Could not connect to trajectory controller! Please check if the robot simulation is running.")
            # 这里你可以选择抛出异常或者继续
        else:
            rospy.loginfo("Connected to trajectory controller successfully!")
        
        self.robot_model = pin.buildModelFromUrdf(MODEL_PATH)
        self.robot_data_pin = pin.Data(self.robot_model)

        self.freq = control_freq
        self.rate = rospy.Rate(self.freq)
        self.dt = 1.0 / self.freq
        self.max_joint_vel = 2.0
        self.default_vel = 1.0
        
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        self.robot_state = RobotState(
            joint_state=JointStates(np.zeros(6), np.zeros(6), np.zeros(6)),
            ee_state=CartesianState(np.eye(4), np.zeros(3), np.eye(4), np.zeros(3))
        )

        # 参考轨迹跟踪相关
        self.reference_pose = None
        self.trajectory_tracking_enabled = False
        self.auto_tracking_mode = enable_tracking
        self.tracking_topic = tracking_topic
        self.tracking_velocity = tracking_velocity

        self.robot_state_sub = rospy.Subscriber("/joint_states", JointState, self.update_robot_state)
        rospy.loginfo(f"{node_name} Initialized, freq={self.freq}Hz, auto_tracking={enable_tracking}")

    def update_robot_state(self, joint_msg: JointState):
        """更新机器人状态（可被子类重写以添加额外处理）"""
        order = [2, 1, 0, 3, 4, 5] # 根据你的实际映射调整
        try:
            self.robot_state.joint_state.position = np.array(joint_msg.position)[order]
            self.robot_state.joint_state.velocity = np.array(joint_msg.velocity)[order]
            self.robot_state.joint_state.effort = np.array(joint_msg.effort)[order]

            pin.forwardKinematics(self.robot_model, self.robot_data_pin, 
                                  self.robot_state.joint_state.position, 
                                  self.robot_state.joint_state.velocity)
            pin.updateFramePlacements(self.robot_model, self.robot_data_pin)
            
            tool0_id = self.robot_model.getFrameId("tool0")
            self.robot_state.ee_state.robot_trans = self.robot_data_pin.oMf[tool0_id].homogeneous
            vel_base = pin.getFrameVelocity(
                                    self.robot_model, 
                                    self.robot_data_pin, 
                                    tool0_id, 
                                    pin.pinocchio_pywrap_default.ReferenceFrame.WORLD)
            self.robot_state.ee_state.robot_vel = np.hstack((vel_base.linear, vel_base.angular))

            vel_ee = pin.getFrameVelocity(
                                    self.robot_model, 
                                    self.robot_data_pin, 
                                    tool0_id, 
                                    pin.pinocchio_pywrap_default.ReferenceFrame.LOCAL)
            self.robot_state.ee_state.ee_vel = np.hstack((vel_ee.linear, vel_ee.angular))
        except Exception:
            pass

    def reference_trajectory_callback(self, msg: PoseStamped):
        """参考轨迹回调函数"""
        self.reference_pose = msg

    def enable_trajectory_tracking(self, topic_name="/reference_trajectory"):
        """启用轨迹跟踪模式"""
        if not self.trajectory_tracking_enabled:
            self.ref_traj_sub = rospy.Subscriber(topic_name, PoseStamped, self.reference_trajectory_callback, queue_size=1)
            self.trajectory_tracking_enabled = True
            rospy.loginfo(f"Trajectory tracking enabled, subscribing to {topic_name}")

    def disable_trajectory_tracking(self):
        """禁用轨迹跟踪模式"""
        if self.trajectory_tracking_enabled:
            self.ref_traj_sub.unregister()
            self.trajectory_tracking_enabled = False
            self.reference_pose = None
            rospy.loginfo("Trajectory tracking disabled")

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
        """移动到默认位置"""
        self.move_to([np.pi/4, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/4], velocity=velocity, duration=5.0, wait4complete=True)
        # rospy.sleep(1.0)
    
    def wait_for_robot_state(self, timeout=5.0):
        """等待机器人状态更新
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否成功接收到状态
        """
        start_time = rospy.Time.now()
        while self.robot_state.joint_state.position.sum() == 0.0 and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"Timeout waiting for robot state after {timeout}s")
                return False
            rospy.sleep(0.1)
        return True

    def run_trajectory_tracking_loop(self, velocity=0.5):
        """运行轨迹跟踪循环，持续跟随参考轨迹"""
        if not self.trajectory_tracking_enabled:
            rospy.logwarn("Trajectory tracking not enabled. Call enable_trajectory_tracking() first.")
            return
        
        rospy.loginfo("Starting trajectory tracking loop...")
        loop_count = 0
        last_print_time = time.time()
        last_loop_time = time.time()
        
        while not rospy.is_shutdown():
            loop_start_time = time.time()
            
            # 检查是否收到参考轨迹
            
            if self.reference_pose is not None:
                print(f"Reference Pose: {self.reference_pose.pose.position}")
                # 提取目标位置
                target_pos = np.array([
                    self.reference_pose.pose.position.x,
                    self.reference_pose.pose.position.y,
                    self.reference_pose.pose.position.z
                ])

                # 判断有没有姿态信息
                if (
                    self.reference_pose.pose.orientation.x ** 2 +
                    self.reference_pose.pose.orientation.y ** 2 +
                    self.reference_pose.pose.orientation.z ** 2 +
                    self.reference_pose.pose.orientation.w ** 2
                ) < 0.9:
                    target_orin = np.array([
                        self.reference_pose.pose.orientation.x,
                        self.reference_pose.pose.orientation.y,
                        self.reference_pose.pose.orientation.z,
                        self.reference_pose.pose.orientation.w
                    ])
                else:
                    target_orin = None  # 保持当前姿态

                self.reference_pose = None  # 清除参考轨迹，等待下一次更新
                print(f"Reference Pose: {self.reference_pose}")
                # 发送运动命令（非阻塞）
                self.move_to_cartesian(target_pos, target_orin=target_orin, velocity=velocity, wait4complete=False)

                loop_count += 1
                
                # 每隔1秒打印一次频率信息
                if time.time() - last_print_time >= 1.0:
                    loop_duration = loop_start_time - last_loop_time
                    if loop_duration > 0:
                        current_freq = 1.0 / loop_duration
                        rospy.loginfo(f"Tracking Loop Freq: {current_freq:.2f} Hz, Target: {target_pos}")
                    last_print_time = time.time()
                
                last_loop_time = loop_start_time
            else:
                # 等待参考轨迹
                if time.time() - last_print_time >= 2.0:
                    rospy.loginfo("Waiting for reference trajectory...")
                    last_print_time = time.time()
            
            # 维持控制频率
            self.rate.sleep()

    def move_to_cartesian(self, target_pos, target_orin=None, duration=None, velocity=None, wait4complete=True):
        """
        move_to_cartesian 的 Docstring
        
        :param self: 
        :param target_pos: 目标位置
        :param target_orin: 目标姿态， 四元数或旋转矩阵, 若为None则保持当前姿态
        :param duration: 运动时间
        :param velocity: 运动速度
        :param wait4complete: 是否等待运动完成
        """
        if target_orin is None:
            target_orin = self.robot_state.ee_state.robot_trans[:3, :3]
        
        if isinstance(target_orin, np.ndarray) and target_orin.shape == (4,):
            q = target_orin
            target_orin = R.from_quat(q).as_matrix()
        elif isinstance(target_orin, np.ndarray) and target_orin.shape == (3,3):
            pass
        else:
            raise ValueError("target_orin must be a quaternion (4,) or rotation matrix (3,3)")
        
        target_pos_cartesian = np.array(target_pos)
        target_pos_joint, success = self.IK(self.robot_model, 
                                   pin.SE3(target_orin, target_pos_cartesian), 
                                   "tool0", 
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

        goal = FollowJointTrajectoryGoal()

        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = self.joint_names

        p0 = JointTrajectoryPoint()
        p0.positions = current_pos.astype(float).tolist()
        p0.velocities = [0.0]*6
        p0.time_from_start = rospy.Duration(0.0)

        p1 = JointTrajectoryPoint()
        p1.positions = target_pos.astype(float).tolist()
        p1.velocities = vel_cmd.tolist()
        p1.time_from_start = rospy.Duration(duration)

        goal.trajectory.points = [p0, p1]

        # 3. 发送目标
        self.client.send_goal(goal)
        # rospy.loginfo(f"Moving to {p1.positions}")

        if wait4complete:
            # 4. 等待运动完成
            done = self.client.wait_for_result(rospy.Duration(duration + 0.5))
            if not done:
                rospy.logwarn("Move timeout or failed!")
                return False
            else:
                # 检查action状态
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    # rospy.loginfo("Move completed successfully!")
                    return True
                else:
                    rospy.logwarn(f"Move finished with state: {state}")
                    return False
        return None  # 非阻塞模式不返回状态

if __name__ == "__main__":
    # ============ 配置参数 ============
    # 修改这里来切换模式：
    # - enable_tracking=True: 自动监听/reference_trajectory话题并跟踪
    # - enable_tracking=False: 过程化编程模式
    ENABLE_TRACKING = True  # 改为False可使用过程化编程
    
    # ============ 初始化控制器 ============
    controller = UR5eController(
        enable_tracking=ENABLE_TRACKING,
        tracking_topic="/reference_trajectory",
        tracking_velocity=0.5
    )

    # 等待机器人状态更新
    while controller.robot_state.joint_state.position.sum() == 0.0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot state update...")
        rospy.sleep(0.1)

    # 移动到初始位置
    rospy.loginfo("Moving to default position...")
    controller.move2default()
    rospy.sleep(2.0)
    rospy.loginfo("Robot is ready.")
    
    # ============ 根据模式选择执行 ============
    if ENABLE_TRACKING:
        # 轨迹跟踪模式
        rospy.loginfo("=" * 60)
        rospy.loginfo("TRAJECTORY TRACKING MODE")
        rospy.loginfo(f"Subscribing to topic: {controller.tracking_topic}")
        rospy.loginfo("Publish PoseStamped messages to control the robot")
        rospy.loginfo("Example:")
        rospy.loginfo("  rostopic pub /reference_trajectory geometry_msgs/PoseStamped \\")
        rospy.loginfo("    '{header: {frame_id: \"base_link\"}, \\")
        rospy.loginfo("      pose: {position: {x: 0.3, y: 0.2, z: 0.5}}}'")
        rospy.loginfo("=" * 60)
        
        controller.enable_trajectory_tracking(topic_name=controller.tracking_topic)
        controller.run_trajectory_tracking_loop(velocity=controller.tracking_velocity)
    
    else:
        # 过程化编程模式
        rospy.loginfo("=" * 60)
        rospy.loginfo("PROCEDURAL PROGRAMMING MODE")
        rospy.loginfo("Running example: sine wave trajectory in z-axis")
        rospy.loginfo("=" * 60)
        
        # 机器人走一个sin曲线轨迹 z轴
        cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
        A = 0.1
        T = 10.0 
        freq = 100.0
        steps = int(T * freq)
        
        rospy.loginfo("Starting Sine Wave...")
        loop_count = 0
        last_print_time = time.time()
        
        for step in range(steps):
            loop_start_time = time.time()
            t = step / freq
            z_offset = A * np.sin(2 * np.pi * t / T)
            target_pos = cur_ee_pos + np.array([0.0, 0.0, z_offset]).T
            controller.move_to_cartesian(target_pos, velocity=0.1, wait4complete=False)
            controller.rate.sleep()
            
            loop_count += 1
            if time.time() - last_print_time >= 1.0:
                rospy.loginfo(f"Progress: {step}/{steps} ({step/steps*100:.1f}%)")
                last_print_time = time.time()
        
        rospy.loginfo("Completed sin wave trajectory.")
        rospy.spin()

