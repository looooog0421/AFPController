#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人运动学封装
基于Pinocchio实现正运动学、逆运动学、雅可比等
"""
import numpy as np
import pinocchio as pin
from typing import Optional, Tuple
from .impedance_types import CartesianState


class RobotKinematicsWrapper:
    """机器人运动学包装类"""
    
    def __init__(self, urdf_path: str, end_effector_frame: str = "flange", use_mujoco_frame: bool = False):
        """
        初始化运动学模块
        
        Args:
            urdf_path: URDF文件路径
            end_effector_frame: 末端执行器框架名称
            use_mujoco_frame: 是否转换到MuJoCo坐标系（x和y取反）
        """
        # 加载模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.use_mujoco_frame = use_mujoco_frame
        
        # 末端执行器框架
        if self.model.existFrame(end_effector_frame):
            self.ee_frame_id = self.model.getFrameId(end_effector_frame)
        else:
            # 如果指定框架不存在，使用最后一个框架
            self.ee_frame_id = self.model.nframes - 1
            print(f"Warning: Frame '{end_effector_frame}' not found, using frame {self.ee_frame_id}")
        
        self.n_joints = self.model.nq
        
        # IK求解器参数
        self.ik_eps = 1e-4
        self.ik_max_iter = 100
        self.ik_damping = 1e-6
        self.position_only_ik = False  # 是否只做位置IK（忽略姿态）
    
    def forward_kinematics(self, q: np.ndarray) -> CartesianState:
        """
        正运动学：关节角度 -> 笛卡尔位姿
        
        Args:
            q: 关节角度 (n_joints,)
        
        Returns:
            CartesianState: 笛卡尔位姿
        """
        assert q.shape == (self.n_joints,), f"Expected joint state of shape ({self.n_joints},), got {q.shape}"
        
        # 计算正运动学
        pin.framesForwardKinematics(self.model, self.data, q)
        
        # 获取末端位姿
        ee_pose = self.data.oMf[self.ee_frame_id]  # SE3 object
        
        # 提取位置和姿态
        position = ee_pose.translation.copy()
        rotation_matrix = ee_pose.rotation.copy()
        
        # 坐标系转换：MuJoCo和Pinocchio的基座坐标系x,y轴方向相反
        if self.use_mujoco_frame:
            position[0] = -position[0]
            position[1] = -position[1]
            # 旋转矩阵也需要相应调整（绕z轴旋转180度）
            rotation_matrix[:, 0] = -rotation_matrix[:, 0]  # x轴取反
            rotation_matrix[:, 1] = -rotation_matrix[:, 1]  # y轴取反
        
        # 旋转矩阵转四元数 [qw, qx, qy, qz]
        from scipy.spatial.transform import Rotation as R
        quat_xyzw = R.from_matrix(rotation_matrix).as_quat()  # [qx, qy, qz, qw]
        orientation = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # [qw, qx, qy, qz]
        
        return CartesianState(
            position=position,
            orientation=orientation,
            velocity=np.zeros(6)
        )
    
    def inverse_kinematics(self, 
                          target_cartesian: CartesianState, 
                          q_init: np.ndarray,
                          joint_limits: bool = True) -> Tuple[np.ndarray, bool]:
        """
        逆运动学：笛卡尔位姿 -> 关节角度
        使用数值迭代方法（基于雅可比伪逆）
        
        Args:
            target_cartesian: 目标笛卡尔位姿
            q_init: 初始关节角度（通常使用当前关节角度）
            joint_limits: 是否考虑关节限位
        
        Returns:
            (q_solution, success): 关节角度解和成功标志
        """
        # 目标位姿转换为SE3
        target_pose = target_cartesian.to_pose_matrix()
        target_position = target_pose[:3, 3].copy()
        target_rotation = target_pose[:3, :3].copy()
        
        # 坐标系转换：如果输入是MuJoCo坐标系，转换到Pinocchio坐标系
        if self.use_mujoco_frame:
            target_position[0] = -target_position[0]
            target_position[1] = -target_position[1]
            target_rotation[:, 0] = -target_rotation[:, 0]
            target_rotation[:, 1] = -target_rotation[:, 1]
        
        target_se3 = pin.SE3(target_rotation, target_position)
        
        q = q_init.copy()
        
        for i in range(self.ik_max_iter):
            # 正运动学
            pin.framesForwardKinematics(self.model, self.data, q)
            current_pose = self.data.oMf[self.ee_frame_id]
            
            # Position-only IK: 直接使用笛卡尔位置误差
            if self.position_only_ik:
                # 直接计算位置误差（3维）
                pos_error = target_position - current_pose.translation
                error_norm = np.linalg.norm(pos_error)
                
                # 检查收敛
                if error_norm < self.ik_eps:
                    return q, True
                
                # 计算位置雅可比（前3行）
                J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
                J_pos = J[:3, :]
                
                # 阻尼最小二乘法求解
                J_damped = J_pos.T @ J_pos + self.ik_damping * np.eye(self.n_joints)
                dq = np.linalg.solve(J_damped, J_pos.T @ pos_error)
                
            else:
                # 完整6D IK: 使用SE3 log（参考ur5e_controller实现）
                # 误差 = log(current^-1 * target)，在当前坐标系下
                error_se3_log = pin.log6(current_pose.inverse() * target_se3)
                error_6d = error_se3_log.vector  # [v_x, v_y, v_z, w_x, w_y, w_z]
                
                # 检查收敛
                if np.linalg.norm(error_6d) < self.ik_eps:
                    return q, True
                
                # 计算局部雅可比矩阵（LOCAL坐标系）
                J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.LOCAL)
                
                # 阻尼最小二乘法求解
                J_damped = J.T @ J + self.ik_damping * np.eye(self.n_joints)
                dq = np.linalg.solve(J_damped, J.T @ error_6d)
            
            # 更新关节角度
            q = pin.integrate(self.model, q, dq)
            
            # 关节限位检查
            if joint_limits:
                q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        
        # 未收敛 - 报告适当的误差
        if self.position_only_ik:
            final_error = np.linalg.norm(target_position - current_pose.translation)
            print(f"Warning: Position-only IK did not converge after {self.ik_max_iter} iterations, error: {final_error*1000:.2f}mm")
        else:
            final_error = np.linalg.norm(error_6d)
            print(f"Warning: IK did not converge after {self.ik_max_iter} iterations, error: {final_error}")
        return q, False
    
    def compute_jacobian(self, q: np.ndarray, frame: str = 'world') -> np.ndarray:
        """
        计算雅可比矩阵
        
        Args:
            q: 关节角度 (n_joints,)
            frame: 参考坐标系 ('world', 'local', 'local_world_aligned')
        
        Returns:
            J: 雅可比矩阵 (6, n_joints)
        """
        assert q.shape == (self.n_joints,), f"Expected joint state of shape ({self.n_joints},), got {q.shape}"
        
        # 更新运动学
        pin.framesForwardKinematics(self.model, self.data, q)
        
        # 选择参考系
        if frame == 'world':
            ref_frame = pin.ReferenceFrame.WORLD
        elif frame == 'local':
            ref_frame = pin.ReferenceFrame.LOCAL
        elif frame == 'local_world_aligned':
            ref_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        else:
            raise ValueError(f"Invalid frame: {frame}")
        
        # 计算雅可比
        J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, ref_frame)
        
        return J
    
    def compute_cartesian_velocity(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """
        计算笛卡尔空间速度
        
        Args:
            q: 关节角度 (n_joints,)
            dq: 关节速度 (n_joints,)
        
        Returns:
            cart_vel: 笛卡尔速度 (6,) [vx, vy, vz, wx, wy, wz]
        """
        J = self.compute_jacobian(q, frame='world')
        cart_vel = J @ dq
        return cart_vel
    
    def get_end_effector_transform(self, q: np.ndarray) -> np.ndarray:
        """
        获取末端执行器变换矩阵
        
        Args:
            q: 关节角度 (n_joints,)
        
        Returns:
            T: 4x4齐次变换矩阵
        """
        pin.framesForwardKinematics(self.model, self.data, q)
        ee_pose = self.data.oMf[self.ee_frame_id]
        return ee_pose.homogeneous
    
    def get_end_effector_rotation(self, q: np.ndarray) -> np.ndarray:
        """
        获取末端执行器旋转矩阵
        
        Args:
            q: 关节角度 (n_joints,)
        
        Returns:
            R: 3x3旋转矩阵
        """
        pin.framesForwardKinematics(self.model, self.data, q)
        ee_pose = self.data.oMf[self.ee_frame_id]
        return ee_pose.rotation
    
    def set_ik_parameters(self, eps: float = None, max_iter: int = None, 
                         damping: float = None, position_only: bool = None):
        """设置IK求解器参数"""
        if eps is not None:
            self.ik_eps = eps
        if max_iter is not None:
            self.ik_max_iter = max_iter
        if damping is not None:
            self.ik_damping = damping
        if position_only is not None:
            self.position_only_ik = position_only
