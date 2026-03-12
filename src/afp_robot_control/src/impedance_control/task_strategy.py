#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务策略模块
定义不同任务场景下的误差计算和期望力计算策略
"""
import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation as R
from .impedance_types import CartesianState, WrenchData


class TaskStrategy(ABC):
    """任务策略抽象基类"""
    
    @abstractmethod
    def compute_error(self, 
                      current_state: CartesianState,
                      reference_state: CartesianState,
                      current_wrench: Optional[WrenchData] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算位姿误差
        
        Args:
            current_state: 当前笛卡尔状态
            reference_state: 参考笛卡尔状态
            current_wrench: 当前力/力矩（可选，某些策略需要）
        
        Returns:
            (position_error (3,), orientation_error (3,))
            位置误差: [ex, ey, ez] (m)
            姿态误差: [erx, ery, erz] 轴角表示 (rad)
        """
        pass
    
    @abstractmethod
    def compute_desired_wrench(self,
                               current_wrench: WrenchData,
                               reference_wrench: Optional[WrenchData] = None,
                               error_6d: Optional[np.ndarray] = None) -> np.ndarray:
        """
        计算期望力/力矩
        
        Args:
            current_wrench: 当前力/力矩
            reference_wrench: 参考力/力矩（可选）
            error_6d: 6D位姿误差（可选，某些策略需要）
        
        Returns:
            desired_wrench (6,): [fx, fy, fz, tx, ty, tz]
        """
        pass
    
    def get_strategy_name(self) -> str:
        """返回策略名称"""
        return self.__class__.__name__


class StandardStrategy(TaskStrategy):
    """
    标准策略：直接位姿误差
    适用于常规阻抗控制任务
    """
    
    def __init__(self, target_wrench: np.ndarray = None):
        """
        Args:
            target_wrench: 目标力/力矩 (6,) [fx, fy, fz, tx, ty, tz]，默认为零
        """
        self.target_wrench = target_wrench if target_wrench is not None else np.zeros(6)
    
    def compute_error(self, 
                      current_state: CartesianState,
                      reference_state: CartesianState,
                      current_wrench: Optional[WrenchData] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        标准位姿误差计算
        
        位置误差: e_pos = ref_pos - curr_pos
        姿态误差: 四元数误差转轴角
        """
        # 位置误差
        position_error = reference_state.position - current_state.position
        
        # 姿态误差（四元数）
        # q_error = q_ref * q_curr^(-1)
        q_curr = current_state.orientation  # [qw, qx, qy, qz]
        q_ref = reference_state.orientation
        
        # 四元数逆: q^(-1) = [qw, -qx, -qy, -qz] / |q|^2 (归一化后直接共轭)
        q_curr_inv = np.array([q_curr[0], -q_curr[1], -q_curr[2], -q_curr[3]])
        
        # 四元数乘法
        q_error = self._quaternion_multiply(q_ref, q_curr_inv)
        
        # 转轴角表示
        orientation_error = self._quaternion_to_axis_angle(q_error)
        
        return position_error, orientation_error
    
    def compute_desired_wrench(self,
                               current_wrench: WrenchData,
                               reference_wrench: Optional[WrenchData] = None,
                               error_6d: Optional[np.ndarray] = None) -> np.ndarray:
        """
        标准期望力：直接返回目标力
        """
        if reference_wrench is not None:
            return reference_wrench.to_array()
        return self.target_wrench
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        四元数乘法 q1 * q2
        q = [qw, qx, qy, qz]
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def _quaternion_to_axis_angle(self, q: np.ndarray) -> np.ndarray:
        """
        四元数转轴角表示
        q = [qw, qx, qy, qz]
        返回: axis_angle (3,) [theta*nx, theta*ny, theta*nz]
        """
        qw, qx, qy, qz = q
        
        # 处理接近单位四元数的情况
        if abs(qw) > 0.999999:
            return np.zeros(3)
        
        # theta = 2 * arccos(qw)
        theta = 2.0 * np.arccos(np.clip(qw, -1.0, 1.0))
        
        # axis = [qx, qy, qz] / sin(theta/2)
        sin_half_theta = np.sqrt(1.0 - qw**2)
        if sin_half_theta < 1e-6:
            return np.zeros(3)
        
        axis = np.array([qx, qy, qz]) / sin_half_theta
        
        # axis_angle = theta * axis
        return theta * axis
    
    def set_target_wrench(self, target_wrench: np.ndarray):
        """动态设置目标力/力矩"""
        assert target_wrench.shape == (6,), "Target wrench must be (6,) array"
        self.target_wrench = target_wrench


class AFPStrategy(TaskStrategy):
    """
    AFP特殊策略：x-z平面合力控制
    适用于纤维铺放任务，保持恒定接触力
    
    特点：
    - 末端坐标系x-z平面内的合力保持恒定: sqrt(fx^2 + fz^2) = F_target
    - 力的分配由当前接触状态决定: Δfx/Δfz = fx/fz
    - y方向和转矩使用标准阻抗控制
    """
    
    def __init__(self, 
                 target_contact_force: float,
                 y_target_force: float = 0.0,
                 target_torque: np.ndarray = None):
        """
        Args:
            target_contact_force: 目标接触力大小 (N) sqrt(fx^2 + fz^2)
            y_target_force: y方向目标力 (N)，默认0
            target_torque: 目标力矩 (3,) [tx, ty, tz]，默认零
        """
        self.F_target = target_contact_force
        self.y_target_force = y_target_force
        self.target_torque = target_torque if target_torque is not None else np.zeros(3)
        
        # 用于平滑过渡的最小力阈值
        self.min_force_threshold = 0.5  # N
    
    def compute_error(self, 
                      current_state: CartesianState,
                      reference_state: CartesianState,
                      current_wrench: Optional[WrenchData] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        AFP位姿误差计算
        
        位置误差：使用标准计算（参考轨迹 - 当前位置）
        姿态误差：使用标准四元数误差
        """
        # 使用标准策略的位姿误差计算
        standard_strategy = StandardStrategy()
        return standard_strategy.compute_error(current_state, reference_state, current_wrench)
    
    def compute_desired_wrench(self,
                               current_wrench: WrenchData,
                               reference_wrench: Optional[WrenchData] = None,
                               error_6d: Optional[np.ndarray] = None) -> np.ndarray:
        """
        AFP期望力计算：x-z平面合力控制
        
        核心逻辑:
        1. 当前合力: F_curr = sqrt(fx^2 + fz^2)
        2. 力误差: ΔF = F_target - F_curr
        3. 按比例分配: Δfx/Δfz = fx/fz
        """
        # 确保力数据在末端坐标系
        if current_wrench.frame != 'endeffector':
            raise ValueError(f"AFPStrategy requires wrench in 'endeffector' frame, got '{current_wrench.frame}'")
        
        fx = current_wrench.force[0]
        fy = current_wrench.force[1]
        fz = current_wrench.force[2]
        
        # 计算x-z平面的当前合力
        F_current = np.sqrt(fx**2 + fz**2)
        
        # 力误差（标量）
        delta_F_magnitude = self.F_target - F_current
        
        # 计算力的分配比例
        if F_current > self.min_force_threshold:
            # 有显著接触力时，按当前力比例分配
            ratio_x = fx / F_current
            ratio_z = fz / F_current
        else:
            # 接触力很小时，使用默认分配（主要在z方向）
            ratio_x = 0.0
            ratio_z = 1.0
        
        # 计算x和z方向的力调整量
        delta_fx = delta_F_magnitude * ratio_x
        delta_fz = delta_F_magnitude * ratio_z
        
        # 期望力（相对于参考力的增量）
        desired_fx = delta_fx
        desired_fz = delta_fz
        desired_fy = self.y_target_force - fy  # y方向标准控制
        
        # 组装6D期望力
        desired_wrench = np.array([
            desired_fx,
            desired_fy,
            desired_fz,
            self.target_torque[0] - current_wrench.torque[0],
            self.target_torque[1] - current_wrench.torque[1],
            self.target_torque[2] - current_wrench.torque[2]
        ])
        
        return desired_wrench
    
    def set_target_contact_force(self, force: float):
        """动态设置目标接触力"""
        assert force >= 0, "Target contact force must be non-negative"
        self.F_target = force
    
    def set_y_target_force(self, force: float):
        """动态设置y方向目标力"""
        self.y_target_force = force
    
    def set_target_torque(self, torque: np.ndarray):
        """动态设置目标力矩"""
        assert torque.shape == (3,), "Target torque must be (3,) array"
        self.target_torque = torque
    
    def get_current_contact_force(self, wrench: WrenchData) -> float:
        """获取当前接触力大小"""
        if wrench.frame != 'endeffector':
            raise ValueError("Wrench must be in endeffector frame")
        return np.sqrt(wrench.force[0]**2 + wrench.force[2]**2)


class HybridStrategy(TaskStrategy):
    """
    混合策略：某些自由度使用力控制，其他自由度使用位置控制
    可以灵活配置每个自由度的控制模式
    """
    
    def __init__(self, 
                 force_controlled_dofs: list,  # [0-5] 哪些自由度使用力控制
                 target_wrench: np.ndarray = None):
        """
        Args:
            force_controlled_dofs: 力控制自由度索引列表 [0-5] 
                                  0-2: x,y,z平移, 3-5: rx,ry,rz旋转
            target_wrench: 目标力/力矩 (6,)
        """
        self.force_controlled_dofs = force_controlled_dofs
        self.position_controlled_dofs = [i for i in range(6) if i not in force_controlled_dofs]
        self.target_wrench = target_wrench if target_wrench is not None else np.zeros(6)
    
    def compute_error(self, 
                      current_state: CartesianState,
                      reference_state: CartesianState,
                      current_wrench: Optional[WrenchData] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        混合误差计算：位置控制自由度计算位姿误差，力控制自由度误差为0
        """
        # 使用标准策略计算完整误差
        standard_strategy = StandardStrategy()
        pos_error, ori_error = standard_strategy.compute_error(current_state, reference_state)
        
        error_6d = np.concatenate([pos_error, ori_error])
        
        # 力控制自由度的误差设为0（不进行位置修正）
        for dof in self.force_controlled_dofs:
            error_6d[dof] = 0.0
        
        return error_6d[:3], error_6d[3:]
    
    def compute_desired_wrench(self,
                               current_wrench: WrenchData,
                               reference_wrench: Optional[WrenchData] = None,
                               error_6d: Optional[np.ndarray] = None) -> np.ndarray:
        """
        混合期望力：力控制自由度使用目标力，位置控制自由度期望力为0
        """
        desired_wrench = np.zeros(6)
        
        # 力控制自由度使用目标力
        for dof in self.force_controlled_dofs:
            if reference_wrench is not None:
                desired_wrench[dof] = reference_wrench.to_array()[dof]
            else:
                desired_wrench[dof] = self.target_wrench[dof]
        
        return desired_wrench
