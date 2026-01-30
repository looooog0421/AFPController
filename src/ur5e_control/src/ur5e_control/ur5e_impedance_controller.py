#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UR5e阻抗控制器
基于力传感器反馈的笛卡尔空间阻抗控制
继承自UR5eController基类
"""

import numpy as np
import rospy
import time
import sys
import os
import pinocchio as pin
from collections import deque
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, WrenchStamped
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm

# 导入基类
sys.path.insert(0, os.path.dirname(__file__))
from ur5e_controller import UR5eController, JointStates, CartesianState, RobotState, R2rotVec, rotVec2R


@dataclass
class ImpedanceParams:
    """阻抗参数"""
    stiffness: np.ndarray  # 刚度 [Kx, Ky, Kz, Krx, Kry, Krz]
    damping: np.ndarray    # 阻尼 [Dx, Dy, Dz, Drx, Dry, Drz]
    mass: np.ndarray = None  # 可选的惯性项


class MovingAverageFilter:
    """移动平均滤波器"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value):
        self.buffer.append(value)
        return np.mean(self.buffer, axis=0)
    
    def reset(self):
        self.buffer.clear()


class ExponentialFilter:
    """指数滤波器（一阶低通滤波）"""
    def __init__(self, alpha=0.2):
        self.alpha = alpha  # 平滑系数 (0-1)，越小越平滑
        self.filtered_value = None
    
    def update(self, value):
        if self.filtered_value is None:
            self.filtered_value = value
        else:
            self.filtered_value = self.alpha * value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        self.filtered_value = None


class UR5eImpedanceController(UR5eController):
    """UR5e阻抗控制器
    
    继承自UR5eController，添加力传感器反馈和阻抗控制逻辑
    """
    
    def __init__(self, 
                 enable_tracking=False,
                 control_freq=500.0,
                 impedance_params=None,
                 filter_type='exponential',
                 filter_alpha=0.3):
        """初始化UR5e阻抗控制器
        
        Args:
            enable_tracking: 是否启用轨迹跟踪模式
            control_freq: 控制频率 (Hz)，建议200-500Hz
            impedance_params: ImpedanceParams对象，包含刚度和阻尼
            target_wrench: 目标力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
            filter_type: 滤波器类型 'exponential' or 'moving_average'
            filter_alpha: 指数滤波器平滑系数 (0-1)
        """
        # 调用基类构造函数
        super().__init__(
            node_name="ur5e_impedance_controller",
            control_freq=control_freq,
            enable_tracking=enable_tracking,
            tracking_topic="/reference_trajectory"
        )
        
        
        # 阻抗特有参数
        # 阻抗参数（默认值）
        if impedance_params is None:
            self.impedance_params = ImpedanceParams(
                stiffness=np.array([500.0, 500.0, 500.0, 50.0, 50.0, 50.0]),
                damping=np.array([50.0, 50.0, 50.0, 5.0, 5.0, 5.0]),
                mass=np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
            )
        else:
            self.impedance_params = impedance_params
        
        # 内部状态变量
        self.xc = np.zeros(6)  # 参考位置/姿态
        self.dxc = np.zeros(6)  # 参考速度/角速度
        
        self.target_wrench = np.zeros(6)  # 目标力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
        self.target_wrench[2] = 10.0  # 默认Z轴10N力
        
        # 当前力/力矩
        self.current_wrench = np.zeros(6)
        self.wrench_received = False
        
        # 力传感器零点标定
        self.wrench_bias = np.zeros(6)
        self.bias_calibration_samples = []
        self.bias_calibration_num = 100
        self.bias_calibrated = False
        
        # 滤波器
        self.filter_type = filter_type
        if filter_type == 'exponential':
            self.force_filter = ExponentialFilter(alpha=filter_alpha)
            self.delta_x_filter = ExponentialFilter(alpha=filter_alpha)
        else:  # moving_average
            window_size = int(self.freq * 0.05)  # 50ms窗口
            self.force_filter = MovingAverageFilter(window_size=window_size)
            self.delta_x_filter = MovingAverageFilter(window_size=window_size)
        
        # 位置积分限制（防止积分饱和）
        self.max_position_correction = 0.05  # 5cm
        
        self.contact_force_threshold = 3.0  # N，接触力阈值
        self.in_contact = False  # 是否接触
        self.release_decay = 0.95
        
        # 订阅力传感器
        self.wrench_sub = rospy.Subscriber("/mujoco/ee_wrench", WrenchStamped, self.wrench_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Impedance Controller Extensions:")
        rospy.loginfo(f"Filter Type: {filter_type}")
        rospy.loginfo(f"Stiffness: {self.impedance_params.stiffness[:3]}")
        rospy.loginfo(f"Damping: {self.impedance_params.damping[:3]}")
        if self.impedance_params.mass is not None:
            rospy.loginfo(f"Mass: {self.impedance_params.mass[:3]}")
        rospy.loginfo("=" * 60)
    
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
        if not self.bias_calibrated:
            self.bias_calibration_samples.append(raw_wrench)
            if len(self.bias_calibration_samples) >= self.bias_calibration_num:
                self.wrench_bias = np.mean(self.bias_calibration_samples, axis=0)
                self.bias_calibrated = True
                rospy.loginfo(f"Wrench bias calibrated: {self.wrench_bias}")
            return
        
        # 零点补偿
        calibrated_wrench = raw_wrench - self.wrench_bias
        
        # 滤波
        self.current_wrench = self.force_filter.update(calibrated_wrench)
        self.wrench_received = True
    
    def compute_impedance_correction(self, target_wrench=None):
        """
        阻抗控制器，
        计算在当前姿态的末端坐标系下的位移修正量
        Args:
            reference_position: 参考位置 [x, y, z] # 基坐标系下
            reference_orientation: 参考姿态 四元数 [x, y, z, w] # 基坐标系下
            target_wrench: 目标力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz] # 末端坐标系下 
        Returns:
            delta_x: 位移修正量 [dx, dy, dz, drx, dry, drz] # 末端坐标系下
        """
        dt = 1.0 / self.freq

        # 力误差
        if target_wrench is None:
            target_wrench = self.target_wrench

        # 外力 环境对机器人的作用力
        F_ext = self.current_wrench
        F_err = - (F_ext - target_wrench)  # 力误差 负号是因为力传感器测量的是环境对机器人的力

        # 阻抗参数
        K = np.diag(self.impedance_params.stiffness)
        D = np.diag(self.impedance_params.damping)
        if self.impedance_params.mass is not None:
            M = np.diag(self.impedance_params.mass)
        else:
            M = np.diag(np.ones(6))

        # 导纳模型计算加速度
        ddxc = np.linalg.solve(M, F_err - D @ self.dxc - K @ self.xc)

        # ========= 接触判断 =========
        force_norm = np.linalg.norm(F_ext[:3] - target_wrench[:3])

        if force_norm > self.contact_force_threshold:
            self.in_contact = True
            self.dxc += ddxc * dt
            self.xc += self.dxc * dt
        else:
            if self.in_contact:
                # 刚刚脱离接触，开始衰减
                self.in_contact = False
            # 衰减位置和速度
            self.dxc *= self.release_decay
            self.xc *= self.release_decay
        
        delta_x = self.xc.copy()
        
        # 滤波修正量
        delta_x_filtered = self.delta_x_filter.update(delta_x)
        # TODO: 根据需要屏蔽某些方向的阻抗控制
        delta_x_filtered[:2] = 0.0  # 仅Z轴方向阻抗控制
        delta_x_filtered[3:] = 0.0  # 不进行姿态阻抗控制
        return delta_x_filtered
    
    def run_impedance_control_loop(self):
        """运行阻抗控制循环"""
        rospy.loginfo("Starting impedance control loop...")
        
        # 等待力传感器标定完成
        while not self.bias_calibrated and not rospy.is_shutdown():
            rospy.loginfo("Calibrating force sensor bias...")
            rospy.sleep(0.5)
        
        # 等待参考轨迹
        if self.auto_tracking_mode:
            while self.reference_pose is None and not rospy.is_shutdown():
                rospy.loginfo("Waiting for reference trajectory...")
                rospy.sleep(1.0)
        else:
            # 使用当前位置作为参考，创建一个PoseStamped
            from geometry_msgs.msg import Pose
            self.reference_pose = PoseStamped()
            self.reference_pose.pose.position.x = self.robot_state.ee_state.robot_trans[0, 3]
            self.reference_pose.pose.position.y = self.robot_state.ee_state.robot_trans[1, 3]
            self.reference_pose.pose.position.z = self.robot_state.ee_state.robot_trans[2, 3]
            
            quat = R.from_matrix(self.robot_state.ee_state.robot_trans[:3, :3]).as_quat()
            self.reference_pose.pose.orientation.w = quat[3]
            self.reference_pose.pose.orientation.x = quat[0]
            self.reference_pose.pose.orientation.y = quat[1]
            self.reference_pose.pose.orientation.z = quat[2]
            
            rospy.loginfo(f"Using current position as reference: [{self.reference_pose.pose.position.x:.3f}, {self.reference_pose.pose.position.y:.3f}, {self.reference_pose.pose.position.z:.3f}]")
        
        loop_count = 0
        last_print_time = time.time()
        last_loop_time = time.time()
        
        while not rospy.is_shutdown():
            loop_start_time = time.time()
            
            try:
                if self.reference_pose is not None:
                    # 1. 从reference_pose提取位置和姿态
                    reference_position = np.array([
                        self.reference_pose.pose.position.x,
                        self.reference_pose.pose.position.y,
                        self.reference_pose.pose.position.z
                    ])
                    reference_orientation = np.array([
                        self.reference_pose.pose.orientation.w,
                        self.reference_pose.pose.orientation.x,
                        self.reference_pose.pose.orientation.y,
                        self.reference_pose.pose.orientation.z
                    ])

                    # self.reference_pose = None  # 清除参考轨迹，等待下一次更新

                    if np.linalg.norm(reference_orientation) < 0.5:
                        reference_orientation = R.from_matrix(self.robot_state.ee_state.robot_trans[:3, :3]).as_quat()
                    # 2. 计算阻抗修正
                    delta_x_ee = self.compute_impedance_correction(self.target_wrench)
                    delta_pos_base = self.robot_state.ee_state.robot_trans[:3, :3] @ delta_x_ee[:3]
                    delta_rot_base = self.robot_state.ee_state.robot_trans[:3, :3] @ delta_x_ee[3:]
                
                    # 3. 计算修正后的目标位置
                    corrected_position = reference_position + delta_pos_base
                    corrected_orientation = reference_orientation  # 暂时不修正姿态

                    corrected_orientation = (R.from_rotvec(delta_rot_base) * R.from_quat(corrected_orientation)).as_matrix()
                    # 4. IK求解
                    target_joint_pos, ik_success = self.IK(
                                                    self.robot_model, 
                                                    pin.SE3(corrected_orientation, corrected_position),
                                                    "tool0",
                                                    self.robot_state.joint_state.position)
                    
                    if not ik_success:
                        rospy.logwarn_throttle(1.0, "IK failed in impedance control")
                        self.rate.sleep()
                        continue
                    
                    # 限制单关节跳变
                    dq = target_joint_pos - self.robot_state.joint_state.position
                    max_step = 0.01  # rad
                    dq = np.clip(dq, -max_step, max_step)
                    target_joint_pos = self.robot_state.joint_state.position + dq

                    # 5. 发送命令
                    # done = self.move_to(target_joint_pos, wait4complete=True)
                    self.servo_joint_command(target_joint_pos)
                    
                    # print(f"Move to target joint position done: {done}")

                    loop_count += 1
                    
                    # 6. 打印状态
                    if time.time() - last_print_time >= 1.0:
                        loop_duration = loop_start_time - last_loop_time
                        if loop_duration > 0:
                            current_freq = 1.0 / loop_duration
                            force_error = self.current_wrench - self.target_wrench
                            rospy.loginfo(f"Freq: {current_freq:.1f}Hz | "
                                        f"Force Error: [{force_error[0]:.2f}, {force_error[1]:.2f}, {force_error[2]:.2f}]N | "
                                        f"Delta_x: [{delta_pos_base[0]*1000:.2f}, {delta_pos_base[1]*1000:.2f}, {delta_pos_base[2]*1000:.2f}]mm"
                                        f"Stiffness: {self.impedance_params.stiffness[:3]} | ")
                        last_print_time = time.time()
                    
                    last_loop_time = loop_start_time
                else:
                    self.trigger_pub.publish(Bool(data=True))
                    # rospy.loginfo("Waiting for reference trajectory...")
                
            except Exception as e:
                rospy.logerr_throttle(1.0, f"Error in impedance control loop: {e}")
            
            # 维持控制频率
            self.rate.sleep()


if __name__ == "__main__":
    # ============ 配置参数 ============
    ENABLE_TRACKING = True  # True=跟踪参考轨迹, False=保持当前位置
    CONTROL_FREQ = 500.0    # 控制频率 (Hz)
    
    # 阻抗参数
    impedance_params = ImpedanceParams(
        stiffness=np.array([800.0, 800.0, 1.0, 80.0, 80.0, 80.0]),  # 刚度
        damping=np.array([80.0, 80.0, 80.0, 8.0, 8.0, 8.0])           # 阻尼
    )
    
    # 目标力 (N)
    target_wrench = np.array([0.0, 0.0, -10.0, 0.0, 0.0, 0.0])  # Z轴-10N力
    
    # ============ 初始化控制器 ============
    controller = UR5eImpedanceController(
        enable_tracking=ENABLE_TRACKING,
        control_freq=CONTROL_FREQ,
        impedance_params=impedance_params,
        filter_type='exponential',
        filter_alpha=0.3
    )
    
    # 等待机器人状态
    if not controller.wait_for_robot_state(timeout=3.0):
        rospy.logerr("Failed to receive robot state")
        sys.exit(1)
    print("robot_state received")
    print("robot_state.ee_state.robot_trans:", controller.robot_state.ee_state.robot_trans)
    # 移动到初始位置
    rospy.loginfo("Moving to default position...")
    controller.move2default()
    rospy.sleep(1.0)
    
    # 移动到模具上方
    rospy.loginfo("Moving to above the mold...")
    controller.move_to_cartesian(
        target_pos=np.array([-0.54936, -0.20258, 0.00463]),
        target_orin=R.from_euler('xyz', [0, 180, 0], degrees=True).as_quat(),
        wait4complete=True
    )

    # 启用轨迹跟踪（如果需要）
    if ENABLE_TRACKING:
        controller.enable_trajectory_tracking("/reference_trajectory")
    
    # 运行阻抗控制
    controller.run_impedance_control_loop()
