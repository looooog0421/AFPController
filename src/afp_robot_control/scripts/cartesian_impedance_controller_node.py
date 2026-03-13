#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
笛卡尔空间阻抗控制ROS节点
200Hz实时控制循环
"""
import rospy
import numpy as np
import os
import sys
from collections import deque
import pinocchio as pin

# ROS消息类型
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, Header
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
# from afp_robot_control.msg import ImpedanceParams, JointPositionCommand  # 自定义消息（需要编译后使用）

# 阻抗控制模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))
from impedance_control import (
    CartesianState,
    WrenchData,
    ImpedanceParams,
    ControlOutput,
    StandardStrategy,
    AFPStrategy,
    CoordinateTransformer,
    CartesianImpedanceController
)


class ImpedanceControllerNode:
    """笛卡尔阻抗控制ROS节点"""
    
    def __init__(self):
        rospy.init_node("cartesian_impedance_controller", anonymous=False)
        
        # ============ 参数加载 ============
        self.load_parameters()
        
        # ============ 初始化策略 ============
        if self.task_type == 'afp':
            rospy.loginfo(f"Using AFP Strategy with target contact force: {self.afp_target_force}N")
            self.strategy = AFPStrategy(
                target_contact_force=self.afp_target_force,
                y_target_force=self.afp_y_target_force,
                target_torque=np.array(self.afp_target_torque)
            )
        elif self.task_type == 'hybrid':
            from impedance_control.task_strategy import HybridStrategy
            rospy.loginfo(f"Using Hybrid Strategy with force-controlled DOFs: {self.hybrid_force_dofs}")
            self.strategy = HybridStrategy(
                force_controlled_dofs=self.hybrid_force_dofs,
                target_wrench=np.array(self.hybrid_target_wrench)
            )
        else:  # standard
            rospy.loginfo("Using Standard Strategy")
            self.strategy = StandardStrategy(
                target_wrench=np.array(self.standard_target_wrench)
            )
        
        # ============ 初始化控制器 ============
        rospy.loginfo(f"Loading URDF from: {self.urdf_path}")
        self.controller = CartesianImpedanceController(
            urdf_path=self.urdf_path,
            strategy=self.strategy,
            end_effector_frame=self.ee_frame,
            control_frequency=self.control_freq
        )
        self.controller.enable_debug(self.debug_enabled)
        
        # ============ 坐标转换器 ============
        self.coord_transformer = CoordinateTransformer(
            rotation_axis=self.sensor_rotation_axis,
            angle_deg=self.sensor_rotation_angle
        )
        rospy.loginfo(f"Sensor frame rotation: {self.sensor_rotation_axis} axis, {self.sensor_rotation_angle} deg")
        
        # ============ 状态变量 ============
        self.current_joint_state = None
        self.current_joint_velocity = None
        self.current_wrench_sensor = None  # 传感器坐标系
        self.reference_pose = None
        self.impedance_params = self._create_default_impedance_params()
        
        # Home position: [0, -90, 90, -90, -90, 90] degrees
        self.home_position = np.deg2rad([0, -90, 90, -90, -90, 90])
        self.at_home_position = False
        
        # Pinocchio模型用于IK计算
        self.robot_model_ik = pin.buildModelFromUrdf(self.urdf_path)
        self.robot_data_ik = self.robot_model_ik.createData()
        
        # 关节名称
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.data_received = {
            'joint_state': False,
            'wrench': False,
            'reference': False
        }
        
        # 力传感器零点标定
        self.wrench_bias = np.zeros(6)
        self.bias_calibration_samples = []
        self.bias_calibration_num = 100
        self.bias_calibrated = False
        
        # ============ ROS订阅 ============
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        rospy.Subscriber('/mujoco/ee_wrench', WrenchStamped, self.wrench_callback, queue_size=1)
        rospy.Subscriber('/reference_trajectory', PoseStamped, self.reference_callback, queue_size=1)
        
        # 动态阻抗参数订阅（使用Float32MultiArray临时替代自定义消息）
        rospy.Subscriber('/impedance_params_dynamic', Float32MultiArray, self.impedance_params_callback, queue_size=1)
        
        # ============ ROS发布 ============
        # 发布UR轨迹命令到MuJoCo仿真或真实机器人
        self.joint_cmd_pub = rospy.Publisher(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal',
            FollowJointTrajectoryActionGoal,
            queue_size=1
        )
        
        # 调试用：发布Float32MultiArray格式
        self.joint_debug_pub = rospy.Publisher('/joint_position_command', Float32MultiArray, queue_size=1)
        
        # 调试信息发布
        if self.debug_enabled:
            self.debug_pub = rospy.Publisher('/impedance_debug', Float32MultiArray, queue_size=1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Cartesian Impedance Controller Node Initialized")
        rospy.loginfo(f"Task Type: {self.task_type}")
        rospy.loginfo(f"Control Frequency: {self.control_freq} Hz")
        rospy.loginfo(f"Home Position: {np.rad2deg(self.home_position)} deg")
        rospy.loginfo(f"Waiting for sensor data...")
        rospy.loginfo("=" * 60)
        
        # ============ 控制循环 ============
        self.rate = rospy.Rate(self.control_freq)
        self.move_to_home_position()
        # self.control_loop()
    
    def load_parameters(self):
        """从ROS参数服务器加载参数"""
        # 机器人配置
        self.urdf_path = rospy.get_param('~urdf_path', 
                                         '/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf')
        self.ee_frame = rospy.get_param('~end_effector_frame', 'flange')
        
        # 控制参数
        self.control_freq = rospy.get_param('~control_frequency', 200.0)
        self.debug_enabled = rospy.get_param('~debug_enabled', True)
        
        # 任务策略
        self.task_type = rospy.get_param('~task_type', 'standard')  # 'standard', 'afp', 'hybrid'
        
        # AFP策略参数
        self.afp_target_force = rospy.get_param('~afp/target_contact_force', 20.0)
        self.afp_y_target_force = rospy.get_param('~afp/y_target_force', 0.0)
        self.afp_target_torque = rospy.get_param('~afp/target_torque', [0.0, 0.0, 0.0])
        
        # 标准策略参数
        self.standard_target_wrench = rospy.get_param('~standard/target_wrench', [0.0]*6)
        
        # 混合策略参数
        self.hybrid_force_dofs = rospy.get_param('~hybrid/force_controlled_dofs', [2])  # 默认Z轴力控
        self.hybrid_target_wrench = rospy.get_param('~hybrid/target_wrench', [0.0]*6)
        
        # 默认阻抗参数
        self.default_stiffness = rospy.get_param('~default_stiffness', [500.0]*6)
        self.default_damping = rospy.get_param('~default_damping', [50.0]*6)
        self.default_mass = rospy.get_param('~default_mass', [1.0]*6)
        self.use_mass = rospy.get_param('~use_mass', False)
        
        # 传感器配置
        self.sensor_rotation_axis = rospy.get_param('~sensor/rotation_axis', 'z')
        self.sensor_rotation_angle = rospy.get_param('~sensor/rotation_angle', 0.0)
        self.enable_bias_calibration = rospy.get_param('~sensor/enable_bias_calibration', True)
        
        # 运动控制参数
        self.max_joint_vel = rospy.get_param('~max_joint_velocity', 0.2)  # rad/s
    
    def _create_default_impedance_params(self) -> ImpedanceParams:
        """创建默认阻抗参数"""
        return ImpedanceParams(
            stiffness=np.array(self.default_stiffness),
            damping=np.array(self.default_damping),
            mass=np.array(self.default_mass) if self.use_mass else None,
            use_mass=self.use_mass
        )
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        order = [2, 1, 0, 3, 4, 5] # 根据你的实际映射调整
        try:
            self.current_joint_state = np.array(msg.position)[order]
            self.current_joint_velocity = np.array(msg.velocity)[order]
            self.data_received['joint_state'] = True
        except Exception as e:
            rospy.logwarn(f"Error processing joint state: {e}")
    
    def wrench_callback(self, msg: WrenchStamped):
        """力/力矩传感器回调"""
        raw_wrench = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])
        
        # 零点标定
        if self.enable_bias_calibration and not self.bias_calibrated:
            self.bias_calibration_samples.append(raw_wrench)
            if len(self.bias_calibration_samples) >= self.bias_calibration_num:
                self.wrench_bias = np.mean(self.bias_calibration_samples, axis=0)
                self.bias_calibrated = True
                rospy.loginfo(f"Wrench bias calibrated: {self.wrench_bias}")
            return
        
        # 零点补偿
        calibrated_wrench = raw_wrench - self.wrench_bias
        
        # 保存传感器坐标系数据
        self.current_wrench_sensor = WrenchData.from_array(
            calibrated_wrench,
            frame='sensor',
            timestamp=msg.header.stamp.to_sec()
        )
        self.data_received['wrench'] = True
    
    def reference_callback(self, msg: PoseStamped):
        """参考轨迹回调"""
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        orientation = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])
        
        self.reference_pose = CartesianState(
            position=position,
            orientation=orientation,
            timestamp=msg.header.stamp.to_sec()
        )
        self.data_received['reference'] = True
        print(f"position: {position}, orientation: {orientation}")
    
    def impedance_params_callback(self, msg: Float32MultiArray):
        """动态阻抗参数回调"""
        # 期望格式: [K1...K6, D1...D6] 或 [K1...K6, D1...D6, M1...M6, use_mass]
        data = np.array(msg.data)
        
        if len(data) == 12:
            # 只有刚度和阻尼
            self.impedance_params = ImpedanceParams(
                stiffness=data[:6],
                damping=data[6:12],
                use_mass=False
            )
        elif len(data) == 18:
            # 刚度、阻尼和质量
            self.impedance_params = ImpedanceParams(
                stiffness=data[:6],
                damping=data[6:12],
                mass=data[12:18],
                use_mass=True
            )
        elif len(data) == 19:
            # 刚度、阻尼、质量和use_mass标志
            self.impedance_params = ImpedanceParams(
                stiffness=data[:6],
                damping=data[6:12],
                mass=data[12:18],
                use_mass=bool(data[18])
            )
        else:
            rospy.logwarn(f"Invalid impedance params length: {len(data)}, expected 12, 18 or 19")
    
    def move_to_home_position(self):
        """移动到home位置 [0, -90, -90, -90, 90, -90] degrees"""
        rospy.loginfo("Moving to home position...")
        
        # 等待关节状态
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and self.current_joint_state is None:
            if rospy.Time.now() > timeout:
                rospy.logwarn("Timeout waiting for joint states, skipping home position")
                return
            rospy.sleep(0.01)
        
        rospy.loginfo(f"Start: {np.rad2deg(self.current_joint_state)} deg")
        rospy.loginfo(f"Target: {np.rad2deg(self.home_position)} deg")
        
        # 直接调用move_to
        self.move_to(self.home_position, velocity=self.max_joint_vel, wait4complete=False)
        
        self.at_home_position = True
        rospy.loginfo("Home position initialization complete")
        rospy.sleep(5.0)
    
    def IK(self, target_position, target_orientation, q_init=None, max_iter=1000, eps=1e-4, damp=1e-6):
        """逆运动学计算
        
        Args:
            target_position: [x, y, z] 位置
            target_orientation: [w, x, y, z] 四元数
            q_init: 初始关节角度
        
        Returns:
            q: 关节角度
            success: 是否成功
        """
        # 构造目标SE3
        quat = pin.Quaternion(target_orientation[0], target_orientation[1], 
                             target_orientation[2], target_orientation[3])
        target_SE3 = pin.SE3(quat.toRotationMatrix(), np.array(target_position))
        
        if q_init is None:
            q = self.current_joint_state.copy() if self.current_joint_state is not None else np.zeros(6)
        else:
            q = q_init.copy()
        
        frame_id = self.robot_model_ik.getFrameId(self.ee_frame)
        
        for i in range(max_iter):
            pin.forwardKinematics(self.robot_model_ik, self.robot_data_ik, q)
            pin.updateFramePlacement(self.robot_model_ik, self.robot_data_ik, frame_id)
            err = pin.log6(self.robot_data_ik.oMf[frame_id].inverse() * target_SE3).vector
            
            if np.linalg.norm(err) < eps:
                return q, True
            
            J = pin.computeFrameJacobian(self.robot_model_ik, self.robot_data_ik, q, frame_id, pin.LOCAL)
            JTJ = J.T.dot(J) + damp * np.eye(self.robot_model_ik.nv)
            d_q = np.linalg.solve(JTJ, J.T.dot(err))
            q = pin.integrate(self.robot_model_ik, q, d_q)
        
        return q, False
    
    def control_loop(self):
        """主控制循环 - 200Hz"""
        rospy.loginfo("Starting impedance control loop...")
        
        while not rospy.is_shutdown():
            # 检查数据是否就绪
            if not all(self.data_received.values()):
                missing = [k for k, v in self.data_received.items() if not v]
                if rospy.get_time() % 5 < 0.01:  # 每5秒提示一次
                    rospy.logwarn(f"Waiting for data: {missing}")
                self.rate.sleep()
                continue
            
            try:
                # 1. IK: 将笛卡尔目标转换为关节角度
                target_joint_positions, ik_success = self.IK(
                    self.reference_pose.position,
                    self.reference_pose.orientation,
                    q_init=self.current_joint_state
                )
                
                if not ik_success:
                    rospy.logwarn_throttle(1.0, "IK failed")
                    self.rate.sleep()
                    continue
                
                # 2. 坐标转换：传感器 -> 末端
                wrench_ee = self.coord_transformer.transform_wrench_to_ee(
                    self.current_wrench_sensor)
                
                # 3. 控制计算
                output = self.controller.compute_control(
                    current_joint_state=self.current_joint_state,
                    current_joint_velocity=self.current_joint_velocity,
                    reference_cartesian=self.reference_pose,
                    current_wrench=wrench_ee,
                    impedance_params=self.impedance_params
                )
                
                # 4. 发布关节指令
                if output.success:
                    # 使用move_to发布命令，不等待完成
                    self.move_to(output.joint_positions, velocity=self.max_joint_vel, wait4complete=False)
                else:
                    rospy.logwarn_throttle(1.0, f"Control failed: {output.error_msg}")
                
                # 5. 发布调试信息
                if self.debug_enabled and output.debug_info:
                    self.publish_debug_info(output.debug_info)
            
            except Exception as e:
                rospy.logerr_throttle(1.0, f"Control loop error: {str(e)}")
            
            self.rate.sleep()
    
    def move_to(self, target_pos, duration=None, velocity=None, wait4complete=False):
        """移动到目标关节位置
        
        Args:
            target_pos: 目标关节角度 [6]
            duration: 运动持续时间，如果None则根据velocity计算
            velocity: 最大关节速度 (rad/s)
            wait4complete: 是否等待运动完成
        """
        target_pos = np.array(target_pos)
        try: 
            current_pos = np.array(self.current_joint_state)
        except: 
            current_pos = np.zeros_like(target_pos)
        
        if velocity is None: 
            velocity = self.max_joint_vel
        
        diffs = target_pos - current_pos
        max_delta = float(np.max(np.abs(diffs)))
        
        if duration is None:
            duration = 0.01 if max_delta <= 1e-6 else max(0.01, max_delta / float(velocity))
        
        vel_cmd = diffs / max(1e-6, float(duration))
        vel_cmd = np.clip(vel_cmd, -self.max_joint_vel, self.max_joint_vel)
        
        p0 = JointTrajectoryPoint()
        p0.positions = current_pos.astype(float).tolist()
        p0.velocities = [0.0]*6
        p0.time_from_start = rospy.Duration(0.0)
        
        p1 = JointTrajectoryPoint()
        p1.positions = target_pos.astype(float).tolist()
        p1.velocities = vel_cmd.tolist()
        p1.time_from_start = rospy.Duration(duration)
        
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = self.joint_names
        goal.goal.trajectory.points = [p0, p1]
        self.joint_cmd_pub.publish(goal)
        
        if wait4complete: 
            rospy.sleep(duration)

        rospy.sleep(5)  # 确保命令发送出去
    
    def publish_debug_info(self, debug_info: dict):
        """发布调试信息"""
        # 简化版：打包关键信息到Float32MultiArray
        # 格式: [error_norm, delta_F_norm, delta_x_norm, ...]
        debug_data = [
            debug_info.get('error_6d_norm', 0.0),
            debug_info.get('delta_F_norm', 0.0),
            debug_info.get('delta_x_norm', 0.0),
        ]
        
        msg = Float32MultiArray()
        msg.data = debug_data
        self.debug_pub.publish(msg)
        
        # 详细信息记录到日志
        if rospy.get_time() % 1.0 < (1.0 / self.control_freq):  # 每秒打印一次
            rospy.loginfo(f"[Debug] Error: {debug_info.get('error_6d_norm', 0):.4f}, "
                         f"ΔF: {debug_info.get('delta_F_norm', 0):.4f}, "
                         f"Δx: {debug_info.get('delta_x_norm', 0):.4f}")

    # def move_to(self, target_pos, duration=None, velocity=None, wait4complete=False):
    #     target_pos = np.array(target_pos)
    #     try: current_pos = np.array(self.current_joint_state)
    #     except: current_pos = np.zeros_like(target_pos)
        
    #     if velocity is None: velocity = self.max_joint_vel
    #     diffs = target_pos - current_pos
    #     max_delta = float(np.max(np.abs(diffs)))
    #     if duration is None:
    #         duration = 0.01 if max_delta <= 1e-6 else max(0.01, max_delta / float(velocity))
        
    #     vel_cmd = diffs / max(1e-6, float(duration))
    #     vel_cmd = np.clip(vel_cmd, -self.max_joint_vel, self.max_joint_vel)
        
    #     p0 = JointTrajectoryPoint()
    #     p0.positions = current_pos.astype(float).tolist(); p0.velocities = [0.0]*6; p0.time_from_start = rospy.Duration(0.0)
    #     p1 = JointTrajectoryPoint()
    #     p1.positions = target_pos.astype(float).tolist(); p1.velocities = vel_cmd.tolist(); p1.time_from_start = rospy.Duration(duration)
        
    #     goal = FollowJointTrajectoryActionGoal()
    #     goal.goal.trajectory.joint_names = self.joint_names
    #     goal.goal.trajectory.points = [p0, p1]
    #     self.joint_cmd_pub.publish(goal)
    #     if wait4complete: rospy.sleep(duration)

if __name__ == '__main__':
    try:
        node = ImpedanceControllerNode()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Node failed: {str(e)}")
        import traceback
        traceback.print_exc()
