#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pinocchio as pin
import numpy as np
import os
from dataclasses import dataclass, field
from typing import Optional, Tuple, Union
from enum import Enum

# ==================== 数据结构定义 ============================
class RotationType(Enum):
    """旋转表示类型"""
    QUATERNION = "quaternion"  # [x, y, z, w]
    EULER_XYZ = "euler_xyz"    # [rx, ry, rz] 内旋XYZ
    EULER_ZYX = "euler_zyx"    # [rz, ry, rx] 外旋ZYX (Roll-Pitch-Yaw)
    ROTATION_MATRIX = "rotation_matrix"  # 3x3矩阵


# ================== 运动学模块 ===================
class RobotKinematics:
    """机器人运动学模块"""
    def __init__(self, urdf_path: str, tcp_offset: np.ndarray = None):
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        self.ee_frame_id = (self.model.getFrameId("flange")
                            if self.model.existFrame("flange")
                            else self.model.nframes - 1)
        
        self.tcp_offset = tcp_offset if tcp_offset is not None else np.zeros(3)
        self.tcp_offset_se3 = pin.SE3(np.eye(3), self.tcp_offset)
    
        # 逆运动学求解
        self.ik_eps = 1e-4
        self.ik_max_iter = 1000
        self.ik_damp = 1e-6
        self.ik_dt = 0.01

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """计算正运动学，返回TCP位置和旋转矩阵"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        flange_se3 = self.data.oMf[self.ee_frame_id]
        tcp_se3 = flange_se3 * self.tcp_offset_se3
        
        return tcp_se3.translation.copy(), tcp_se3.rotation.copy()

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """计算雅可比矩阵"""
        return pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
    
    # =================== 旋转表示转换 ===================
    @staticmethod
    def rotation_to_matrix(
            rotation: np.ndarray,
            rotation_type: RotationType
            ) -> np.ndarray:
        """将各种旋转表示转换为旋转矩阵"""
        if rotation_type == RotationType.ROTATION_MATRIX:
            assert rotation.shape == (3, 3), "Rotation matrix must be 3x3"
            return rotation
        
        elif rotation_type == RotationType.QUATERNION:
            # Pinocchio使用 [x, y, z, w] 格式
            assert len(rotation) == 4, "Quaternion must have 4 elements [x,y,z,w]"
            quat = pin.Quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
            return quat.toRotationMatrix()
        
        elif rotation_type == RotationType.EULER_XYZ:
            # 内旋XYZ (Body-fixed)
            assert len(rotation) == 3, "Euler angles must have 3 elements"
            return pin.rpy.rpyToMatrix(rotation[0], rotation[1], rotation[2])
        
        elif rotation_type == RotationType.EULER_ZYX:
            # 外旋ZYX (Space-fixed, Roll-Pitch-Yaw)
            # 等价于内旋XYZ的逆序
            assert len(rotation) == 3, "Euler angles must have 3 elements"
            return pin.rpy.rpyToMatrix(rotation[2], rotation[1], rotation[0])
        
        else:
            raise ValueError(f"Unknown rotation type: {rotation_type}")

    @staticmethod
    def rotation_to_matrix(
            rotation: np.ndarray,
            rotation_type: RotationType
            ) -> np.ndarray:
        """
        将旋转矩阵转换为指定表示
        
        Args:
            R: 3x3旋转矩阵
            rotation_type: 目标旋转类型
            
        Returns:
            rotation: 指定格式的旋转表示
        """
        if rotation_type == RotationType.ROTATION_MATRIX:
            return rotation
        
        elif rotation_type == RotationType.QUATERNION:
            quat = pin.Quaternion(rotation)
            # 返回 [x, y, z, w]
            return np.array([quat.x, quat.y, quat.z, quat.w])
        
        elif rotation_type == RotationType.EULER_XYZ:
            rpy = pin.rpy.matrixToRpy(rotation)
            return np.array(rpy)
        
        elif rotation_type == RotationType.EULER_ZYX:
            rpy = pin.rpy.matrixToRpy(rotation)
            # 反转顺序
            return np.array([rpy[2], rpy[1], rpy[0]])
        
        else:
            raise ValueError(f"Unknown rotation type: {rotation_type}")
        
    def inverse_kinematics(
            self,
            target: Union[np.ndarray, pin.SE3],
            target_rotation: Union[np.ndarray, None] = None,
            rotation_type: RotationType = RotationType.ROTATION_MATRIX,
            q_init: Optional[np.ndarray] = None,
            position_only: bool = False,
            max_iter: Optional[int] = None,
            eps: Optional[float] = None
        ) -> Tuple[np.ndarray, bool, int]:
        """
        统一的逆运动学接口
        
        支持三种调用方式：
        1. inverse_kinematics(target_position, target_rotation, ...)
        - 指定位置和姿态
        2. inverse_kinematics(target_se3_object, ...)
        - 直接传入pin.SE3对象
        3. inverse_kinematics(target_position, None, ...)
        - 仅指定位置，姿态保持当前不变（需要q_init）
        
        Args:
            target: 目标位置 (3,) 或 pin.SE3 对象
            target_rotation: 目标旋转（格式由rotation_type指定），None表示保持姿态不变
            rotation_type: 旋转表示类型
            q_init: 初始关节角（默认使用中性位姿）
            position_only: 强制仅考虑位置，完全忽略姿态约束
            max_iter: 最大迭代次数
            eps: 收敛阈值
            
        Returns:
            q_solution: 关节角解 (6,)
            success: 是否成功收敛
            iterations: 实际迭代次数
            
        Examples:
            >>> # 方式1：位置+姿态
            >>> q, ok, _ = kin.inverse_kinematics(
            ...     np.array([0.5, 0.2, 0.4]),
            ...     np.array([0, 0, 0, 1]),  # 四元数
            ...     RotationType.QUATERNION
            ... )
            
            >>> # 方式2：SE3对象
            >>> target_se3 = pin.SE3(R, p)
            >>> q, ok, _ = kin.inverse_kinematics(target_se3)
            
            >>> # 方式3：仅位置，保持当前姿态
            >>> q, ok, _ = kin.inverse_kinematics(
            ...     np.array([0.5, 0.2, 0.4]),
            ...     target_rotation=None,
            ...     q_init=current_q
            ... )
            
            >>> # 方式4：仅位置，完全忽略姿态
            >>> q, ok, _ = kin.inverse_kinematics(
            ...     np.array([0.5, 0.2, 0.4]),
            ...     position_only=True
            ... )
        """

        # 参数设置
        max_iter = max_iter if max_iter is not None else self.ik_max_iter
        eps = eps if eps is not None else self.ik_eps

        # 初始关节角
        if q_init is None:
            q = pin.neutral(self.model)
        else:
            q = q_init.copy()

        # 统一转换为 target_SE3
        target_se3 = None

        if isinstance(target, pin.SE3):
            # 方式2：直接传入SE3对象
            target_se3 = target
            target_position = target.translation
            target_R = target.rotation
            solve_orientation = not position_only

        elif isinstance(target, np.ndarray) and target.shape == (3,):
            target_position = target
        
            if target_rotation is not None:
                # 方式1：位置+姿态
                target_R = self.rotation_to_matrix(target_rotation, rotation_type)
                target_se3 = pin.SE3(target_R, target_position)
                solve_orientation = not position_only
            
            elif position_only:
                # 方式4：position_only=True，完全忽略姿态
                target_se3 = pin.SE3(np.eye(3), target_position)
                solve_orientation = False
                
            else:
                # 方式3：target_rotation=None 且 position_only=False
                # 保持当前姿态不变
                if q_init is None:
                    raise ValueError(
                        "When target_rotation=None and position_only=False, "
                        "q_init must be provided to preserve current orientation"
                    )
                
                # 获取当前姿态
                _, current_R = self.forward_kinematics(q)
                target_R = current_R
                target_se3 = pin.SE3(target_R, target_position)
                solve_orientation = True
        else:
            raise TypeError(
                f"target must be np.ndarray of shape (3,) or pin.SE3, "
                f"got {type(target)} with shape {getattr(target, 'shape', 'N/A')}"
            )
        
        # 逆运动学迭代
        for i in range(max_iter):
            # 计算当前tcp位姿
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data)

            flange_se3 = self.data.oMf[self.ee_frame_id]
            current_tcp_se3 = flange_se3 * self.tcp_offset_se3

            # 计算误差
            if solve_orientation:
                error_se3 = target_se3.actInv(current_tcp_se3)
                err = pin.log(error_se3).vector
            else:
                error_pos = target_position - current_tcp_se3.translation
                err = np.concatenate((error_pos, np.zeros(3)))
            
            # 给位置和姿态加权
            w_pos = 1.0
            w_ori = 0.5
            err_weighted = np.hstack([
                err[:3] * w_pos,
                err[3:] * w_ori
            ])

            # 检查收敛
            if solve_orientation:
                err_norm = np.linalg.norm(err_weighted)
            else:
                err_norm = np.linalg.norm(err[:3])

            if err_norm < eps:
                return q, True, i + 1
        
            J = self.jacobian(q)

            if not solve_orientation:
                # 仅使用位置部分的雅可比 (3x6)
                J = J[:3, :]
                err = err[:3]

            # 阻尼最小二乘法求解
            # (J_T J + λ^2 I) Δq = J_T err
            H = J.T @ J + self.ik_damp * np.eye(J.shape[1])
            dq = np.linalg.solve(H, J.T @ err)
            q = pin.integrate(self.model, q, dq * self.ik_dt)
        
        # 迭代结束仍未收敛
        return q, False, max_iter