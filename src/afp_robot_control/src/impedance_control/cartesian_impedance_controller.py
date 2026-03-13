#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
笛卡尔空间阻抗控制器核心模块
位置型阻抗控制：根据力偏差调整位置
"""
import numpy as np
from typing import Optional
from .impedance_types import CartesianState, WrenchData, ImpedanceParams, ControlOutput
from .task_strategy import TaskStrategy
from .robot_kinematics_wrapper import RobotKinematicsWrapper


class CartesianImpedanceController:
    """
    笛卡尔空间阻抗控制器
    
    控制律:
    1. 计算位姿误差: e = x_ref - x_curr
    2. 阻抗控制: F_desired = K*e + D*ė + M*ë
    3. 力误差: ΔF = F_desired - F_measured
    4. 位置修正: Δx = (K + λI)^(-1) * ΔF
    5. 修正目标: x_target = x_ref + Δx
    6. 逆运动学: q_target = IK(x_target)
    """
    
    def __init__(self, 
                 urdf_path: str,
                 strategy: TaskStrategy,
                 end_effector_frame: str = "flange",
                 control_frequency: float = 200.0,
                 use_mujoco_frame: bool = False):
        """
        初始化控制器
        
        Args:
            urdf_path: URDF文件路径
            strategy: 任务策略
            end_effector_frame: 末端执行器框架名称
            control_frequency: 控制频率 (Hz)
            use_mujoco_frame: 是否使用MuJoCo坐标系（x和y取反）
        """
        self.robot_kin = RobotKinematicsWrapper(urdf_path, end_effector_frame, use_mujoco_frame)
        self.strategy = strategy
        self.dt = 1.0 / control_frequency
        
        # 位置修正阻尼参数（用于数值稳定）
        self.correction_damping = 0.1
        
        # 上一时刻的笛卡尔状态（用于速度估计）
        self.last_cartesian_state: Optional[CartesianState] = None
        self.last_time = 0.0
        
        # 调试信息
        self.debug_enabled = True
    
    def compute_control(self,
                       current_joint_state: np.ndarray,
                       current_joint_velocity: np.ndarray,
                       reference_cartesian: CartesianState,
                       current_wrench: WrenchData,
                       impedance_params: ImpedanceParams,
                       reference_wrench: Optional[WrenchData] = None
                       ) -> ControlOutput:
        """
        主控制循环
        
        Args:
            current_joint_state: 当前关节角度 (n_joints,)
            current_joint_velocity: 当前关节速度 (n_joints,)
            reference_cartesian: 参考笛卡尔位姿
            current_wrench: 当前力/力矩（末端坐标系）
            impedance_params: 阻抗参数
            reference_wrench: 参考力/力矩（可选）
        
        Returns:
            ControlOutput: 控制输出（关节角度指令）
        """
        debug_info = {}
        
        try:
            # 1. 正运动学：关节 -> 笛卡尔当前位姿
            current_cartesian = self.robot_kin.forward_kinematics(current_joint_state)
            debug_info['current_cartesian_pos'] = current_cartesian.position.copy()
            debug_info['current_cartesian_ori'] = current_cartesian.orientation.copy()
            
            # 2. 计算笛卡尔速度
            current_cartesian.velocity = self.robot_kin.compute_cartesian_velocity(
                current_joint_state, current_joint_velocity)
            debug_info['cartesian_velocity'] = current_cartesian.velocity.copy()
            
            # 3. 策略层：计算位姿误差
            pos_error, ori_error = self.strategy.compute_error(
                current_cartesian, reference_cartesian, current_wrench)
            error_6d = np.concatenate([pos_error, ori_error])
            debug_info['position_error'] = pos_error.copy()
            debug_info['orientation_error'] = ori_error.copy()
            debug_info['error_6d_norm'] = np.linalg.norm(error_6d)
            
            # 4. 阻抗控制律：F_desired = K*e + D*ė (+ M*ë)
            F_impedance = impedance_params.stiffness * error_6d + \
                         impedance_params.damping * current_cartesian.velocity
            
            if impedance_params.use_mass and impedance_params.mass is not None:
                # 计算加速度（数值微分，简化）
                if self.last_cartesian_state is not None:
                    cart_accel = (current_cartesian.velocity - self.last_cartesian_state.velocity) / self.dt
                else:
                    cart_accel = np.zeros(6)
                F_impedance += impedance_params.mass * cart_accel
                debug_info['cartesian_acceleration'] = cart_accel.copy()
            
            debug_info['F_impedance'] = F_impedance.copy()
            
            # 5. 策略层：计算期望力
            F_desired = self.strategy.compute_desired_wrench(
                current_wrench, reference_wrench, error_6d)
            debug_info['F_desired'] = F_desired.copy()
            
            # 6. 力误差（仅考虑期望外力与实际外力的差异）
            # 注意：在位置型阻抗控制中，位置误差已经在步骤8通过参考位姿体现
            # 这里的力误差只用于力控制场景
            F_measured = current_wrench.to_array()
            
            if reference_wrench is not None:
                # 有期望力时：根据力误差修正位置
                F_ref = reference_wrench.to_array()
                delta_F = F_ref - F_measured
            else:
                # 无期望力时：不做力修正，delta_F = 0
                # 这样Δx = 0，直接用IK求解参考位置
                delta_F = np.zeros(6)
            
            debug_info['F_measured'] = F_measured.copy()
            debug_info['delta_F'] = delta_F.copy()
            debug_info['delta_F_norm'] = np.linalg.norm(delta_F)
            
            # 7. 位置修正：Δx = (K + λI)^(-1) * ΔF
            delta_x = self._compute_position_correction(delta_F, impedance_params)
            debug_info['delta_x'] = delta_x.copy()
            debug_info['delta_x_norm'] = np.linalg.norm(delta_x)
            
            # 8. 修正目标位姿
            corrected_cartesian = self._apply_correction(reference_cartesian, delta_x)
            debug_info['corrected_pos'] = corrected_cartesian.position.copy()
            debug_info['corrected_ori'] = corrected_cartesian.orientation.copy()
            
            # 9. 逆运动学：修正后笛卡尔位姿 -> 关节角度
            target_joints, ik_success = self.robot_kin.inverse_kinematics(
                corrected_cartesian, current_joint_state)
            
            if not ik_success:
                debug_info['warning'] = 'IK did not converge'
            
            debug_info['target_joints'] = target_joints.copy()
            debug_info['strategy_name'] = self.strategy.get_strategy_name()
            
            # 10. 更新历史状态
            self.last_cartesian_state = current_cartesian.copy()
            
            return ControlOutput(
                joint_positions=target_joints,
                success=ik_success,
                error_msg="" if ik_success else "IK convergence failed",
                debug_info=debug_info if self.debug_enabled else {}
            )
        
        except Exception as e:
            return ControlOutput(
                joint_positions=current_joint_state,  # 保持当前位置
                success=False,
                error_msg=f"Control computation failed: {str(e)}",
                debug_info=debug_info
            )
    
    def _compute_position_correction(self, 
                                    delta_F: np.ndarray, 
                                    impedance_params: ImpedanceParams) -> np.ndarray:
        """
        计算位置修正量
        
        使用阻尼最小二乘法: Δx = (K + λI)^(-1) * ΔF
        
        Args:
            delta_F: 力误差 (6,)
            impedance_params: 阻抗参数
        
        Returns:
            delta_x: 位置修正量 (6,) [Δx, Δy, Δz, Δrx, Δry, Δrz]
        """
        K = impedance_params.stiffness
        
        # 构造阻尼矩阵
        K_damped = np.diag(K) + self.correction_damping * np.eye(6)
        
        # 求解：K_damped * Δx = ΔF
        try:
            delta_x = np.linalg.solve(K_damped, delta_F)
        except np.linalg.LinAlgError:
            # 如果矩阵奇异，使用伪逆
            delta_x = np.linalg.pinv(K_damped) @ delta_F
        
        return delta_x
    
    def _apply_correction(self, 
                         reference_cartesian: CartesianState, 
                         delta_x: np.ndarray) -> CartesianState:
        """
        应用位置修正到参考位姿
        
        Args:
            reference_cartesian: 参考笛卡尔位姿
            delta_x: 位置修正量 (6,) [Δx, Δy, Δz, Δrx, Δry, Δrz]
        
        Returns:
            corrected_cartesian: 修正后的笛卡尔位姿
        """
        # 位置修正（简单相加）
        corrected_position = reference_cartesian.position + delta_x[:3]
        
        # 姿态修正（使用四元数）
        delta_rotation = delta_x[3:]  # 轴角表示
        
        # 轴角转四元数
        from scipy.spatial.transform import Rotation as R
        delta_angle = np.linalg.norm(delta_rotation)
        
        if delta_angle > 1e-6:
            delta_axis = delta_rotation / delta_angle
            delta_quat_scipy = R.from_rotvec(delta_rotation).as_quat()  # [qx, qy, qz, qw]
            delta_quat = np.array([delta_quat_scipy[3], delta_quat_scipy[0], 
                                  delta_quat_scipy[1], delta_quat_scipy[2]])  # [qw, qx, qy, qz]
        else:
            delta_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 单位四元数
        
        # 四元数乘法：q_corrected = q_ref * delta_q
        corrected_orientation = self._quaternion_multiply(
            reference_cartesian.orientation, delta_quat)
        
        return CartesianState(
            position=corrected_position,
            orientation=corrected_orientation,
            velocity=reference_cartesian.velocity.copy()
        )
    
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
        
        result = np.array([w, x, y, z])
        return result / np.linalg.norm(result)  # 归一化
    
    def set_strategy(self, strategy: TaskStrategy):
        """动态切换任务策略"""
        self.strategy = strategy
    
    def set_correction_damping(self, damping: float):
        """设置位置修正阻尼参数"""
        assert damping > 0, "Damping must be positive"
        self.correction_damping = damping
    
    def enable_debug(self, enable: bool = True):
        """启用/禁用调试信息"""
        self.debug_enabled = enable
    
    def reset(self):
        """重置控制器状态"""
        self.last_cartesian_state = None
        self.last_time = 0.0
