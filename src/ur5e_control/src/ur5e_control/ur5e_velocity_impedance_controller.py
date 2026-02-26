#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UR5e 导纳控制器
继承自 ur5e_velocity_controller.py中的UR5eController
"""
import pinocchio as pin
import numpy as np
import rospy
import time
import sys
import os
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from ur5e_velocity_controller import UR5eController

@dataclass
class AdmittanceParams:
    """导纳控制参数

    Attributes
    ----------
    stiffness : (6,) 位移刚度 [Kx, Ky, Kz, Krx, Kry, Krz]
    damping   : (6,) 速度阻尼 [Dx, Dy, Dz, Drx, Dry, Drz]
    mass      : (6,) 惯性项   [Mx, My, Mz, Mrx, Mry, Mrz]，None 则忽略
    enabled_axes : (6,) bool，True 表示该轴启用导纳控制
                   默认只开 Z 轴平动
    coordinate_frame : 'base' 或 'ee'，表示力/力矩的坐标系
    """
    stiffness: np.ndarray = field(default_factory=lambda: np.array([800.0, 800.0, 800.0, 80.0, 80.0, 80.0]))
    damping:   np.ndarray = field(default_factory=lambda: np.array([60.0, 60.0, 60.0, 20.0, 20.0, 20.0]))
    mass:      np.ndarray = None
    enabled_axes: np.ndarray = field(default_factory=lambda: np.array([False, False, True, False, False, False]))
    coordinate_frame: str = 'ee'  # 'base' 或 'ee'

class UR5eAdmittanceController(UR5eController):

    def __init__(self,
                 control_freq: float = 200.0,
                 admittance_params: AdmittanceParams = None,
                 max_pos_correction: float = 0.4,
                 contact_threshold: float = 3.0,
                 release_decay: float = 0.95,
                 **kwargs
                 ):
        super().__init__(
            node_name="ur5e_admittance_controller",
            servo_kp=8.0,
            servo_kd=0.5,
            **kwargs
        )
        
        # 导纳控制参数
        self.params = admittance_params if admittance_params is not None else AdmittanceParams()

        self._xc = np.zeros(6)  # 当前的位移/姿态修正量 [dx, dy, dz, drx, dry, drz]
        self._dxc = np.zeros(6) # 当前的位移/姿态修正速度


        self.target_wrench = np.zeros(6)  # 期望的末端力/力矩 [Fx, Fy, Fz, Mx, My, Mz]

        self.max_pos_correction = max_pos_correction
        self.contact_threshold = contact_threshold
        self.release_decay = release_decay
        self._in_contact = False

        self._last_delta_pos_base = np.zeros(6)  # 上一时刻的位移修正（基坐标系）
        self._last_force_error = np.zeros(6)  # 上一时刻的力/力矩误差

        rospy.loginfo("=" * 60)
        rospy.loginfo("Admittance Controller Initialized")
        rospy.loginfo(f"  Stiffness : {self.params.stiffness}")
        rospy.loginfo(f"  Damping   : {self.params.damping}")
        rospy.loginfo(f"  Target F  : {self.target_wrench[:3]} N")
        rospy.loginfo(f"  Axes      : {self.params.enabled_axes}")
        rospy.loginfo("=" * 60)

    def compute_admittance_correction(self, target_wrench=None):
        """根据当前的末端力/力矩误差计算位移/姿态修正量
        
        Args:
            target_wrench: 期望的末端力/力矩 (Fx, Fy, Fz, Mx, My, Mz)，如果为 None 则使用 self.target_wrench
        Returns:
            delta_pos: 位移/姿态修正量 [dx, dy, dz, drx, dry, drz]
        """
        if target_wrench is None:
            target_wrench = self.target_wrench

        F_ext = self.current_wrench.copy()  # 当前末端力/力矩
        F_err = target_wrench - F_ext  # 力/力矩误差

        K = np.diag(self.params.stiffness)
        D = np.diag(self.params.damping)
        
        if self.params.mass is not None:
            M = np.diag(self.params.mass)
        else:
            M = np.eye(6)

        # 导纳模型加速度
        ddxc = np.linalg.solve(M, F_err - D @ self._dxc - K @ self._xc)

        force_norm =  np.linalg.norm(F_err[:3])

        if force_norm > self.contact_threshold:
            self._in_contact = True
            self._dxc += ddxc * self.dt
            self._xc += self._dxc * self.dt
        else:
            self._in_contact = False
            self._dxc *= self.release_decay
            self._xc *= self.release_decay

        self._xc = np.clip(
                    self._xc, -self.max_pos_correction, self.max_pos_correction)

        delta_x = self._xc.copy()
        delta_x[~self.params.enabled_axes] = 0.0

        return delta_x

    def reset_admittance_state(self):
        """重置导纳控制状态"""
        self._xc = np.zeros(6)
        self._dxc = np.zeros(6)
        self._in_contact = False
        rospy.loginfo("Admittance state reset")

    def compute_servo_command(self, target_pos, target_rot, target_wrench=None):
        """计算末端位置/姿态目标对应的关节速度指令
        Args:
            target_pos: 末端目标位置 [x, y, z]
            target_rot: 末端目标姿态 (四元数 [x, y, z, w])
            target_wrench: 期望的末端力/力矩 (Fx, Fy, Fz, Mx, My, Mz)，如果为 None 则使用 self.target_wrench
        Returns:
            joint_vel_cmd: 关节速度指令 [v1, v2, v3, v4, v5, v6]
        """
        # 计算导纳修正
        

        # 将末端修正转换到基坐标系
        if self.params.coordinate_frame == 'ee':
            delta_x_ee = self.compute_admittance_correction(target_wrench)

            with self.kinematics_lock:
                R_base = self.robot_state.ee_state.robot_trans[:3, :3]  # 获取旋转矩阵

            delta_pos_base = R_base @ delta_x_ee[:3]
            delta_rot_base = R_base @ delta_x_ee[3:]
        else:
            delta_x_base = self.compute_admittance_correction(target_wrench)
            delta_pos_base = delta_x_base[:3]
            delta_rot_base = delta_x_base[3:]

        self._last_delta_pos_base = delta_pos_base
        self._last_force_error = self.target_wrench - self.current_wrench

        corrected_pos = target_pos + delta_pos_base
        corrected_rot = (R.from_matrix(target_rot) * R.from_rotvec(delta_rot_base)).as_matrix()

        self.servo_cartesian_pos(corrected_pos, corrected_rot)

        return corrected_pos, corrected_rot
    
    def run_tracking_loop(self):
        """
        轨迹跟踪控制循环，持续计算并发布控制命令
        """
        rospy.loginfo("Starting admittance trajectory tracking loop...")
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

                    
                    corrected_pos, corrected_rot = self.compute_servo_command(ref_pos, ref_rot)
                    
                    loop_count += 1

                    now = time.time()
                    if now - last_log_time >= 1.0:
                        dt = loop_start_time - last_loop_time
                        freq = 1.0 / dt if dt > 1e-6 else float('inf')
                        ee_pos = self.robot_state.ee_state.robot_trans[:3, 3]
                        rospy.loginfo(
                            f"Freq: {freq:.1f}Hz | "
                            f"EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] | "
                            f"Target: [{corrected_pos[0]:.3f}, {corrected_pos[1]:.3f}, {corrected_pos[2]:.3f}] | "
                            f"Ref: [{ref_pos[0]:.3f}, {ref_pos[1]:.3f}, {ref_pos[2]:.3f}]"
                            f"wrench: [{self.current_wrench[0]:.2f}, {self.current_wrench[1]:.2f}, {self.current_wrench[2]:.2f}] N"
                        )
                        last_log_time = now

                    last_loop_time = loop_start_time

                else:
                    self.trigger_pub.publish(Bool(data=True))  # 触发外部推理

            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Error in tracking loop: {e}")

            self.rate.sleep()
    

if __name__ == "__main__":
    
    params = AdmittanceParams(
        stiffness=np.array([800.0, 800.0, 800.0, 80.0, 80.0, 80.0]),
        damping=np.array([60.0, 60.0, 60.0, 20.0, 20.0, 20.0]),
        mass=None,
        enabled_axes=np.array([False, False, True, False, False, False]),
        coordinate_frame='ee',
    )

    target_wrench = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0])  # 期望末端力：沿Z轴向下5N

    controller = UR5eAdmittanceController(
        control_freq=200.0,
        admittance_params=params,
        max_pos_correction=0.05,
        contact_threshold=3.0,
        release_decay=0.95
    )

    if not controller.wait_for_robot_state(timeout=5.0):
        rospy.logerr("Failed to get initial robot state. Exiting.")
        sys.exit(1)

    # 1. 切换到位置控制，移动到初始位置
    controller.switch_to_position_control()
    controller.move2default(velocity=0.5, wait4complete=True)
    rospy.sleep(1.0)


    rospy.loginfo("Moving to above the mold...")
    controller.move_to_cartesian(
        target_pos=np.array([-0.54936, -0.20258, 0.00463]),
        # target_pos=np.array([-0.3, -0.3, 0.4]),
        
        target_rot=R.from_euler('xyz', [0, 180, 0], degrees=True).as_matrix(),
        wait4complete=True
    )

    controller.target_wrench = target_wrench
    controller.switch_to_velocity_control()
    controller.enable_trajectory_tracking()
    controller.run_tracking_loop()