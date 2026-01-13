#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阻抗控制数据结构定义
"""
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from scipy.spatial.transform import Rotation as R


@dataclass
class CartesianState:
    """笛卡尔空间状态"""
    position: np.ndarray        # [x, y, z] 位置 (3,)
    orientation: np.ndarray     # 四元数 [qw, qx, qy, qz] (4,)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))  # 线速度 + 角速度 (6,)
    timestamp: float = 0.0
    
    def __post_init__(self):
        """验证数据格式"""
        assert self.position.shape == (3,), "Position must be (3,) array"
        assert self.orientation.shape == (4,), "Orientation must be (4,) quaternion [qw,qx,qy,qz]"
        assert self.velocity.shape == (6,), "Velocity must be (6,) array"
        
        # 归一化四元数
        self.orientation = self.orientation / np.linalg.norm(self.orientation)
    
    def to_pose_matrix(self) -> np.ndarray:
        """转换为4x4齐次变换矩阵"""
        T = np.eye(4)
        T[:3, 3] = self.position
        T[:3, :3] = R.from_quat([self.orientation[1], self.orientation[2], 
                                  self.orientation[3], self.orientation[0]]).as_matrix()
        return T
    
    @classmethod
    def from_pose_matrix(cls, T: np.ndarray, velocity: np.ndarray = None, timestamp: float = 0.0):
        """从4x4齐次变换矩阵创建"""
        position = T[:3, 3]
        quat_xyzw = R.from_matrix(T[:3, :3]).as_quat()  # [qx, qy, qz, qw]
        orientation = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # [qw, qx, qy, qz]
        vel = velocity if velocity is not None else np.zeros(6)
        return cls(position=position, orientation=orientation, velocity=vel, timestamp=timestamp)
    
    def copy(self):
        """深拷贝"""
        return CartesianState(
            position=self.position.copy(),
            orientation=self.orientation.copy(),
            velocity=self.velocity.copy(),
            timestamp=self.timestamp
        )


@dataclass
class WrenchData:
    """力/力矩数据"""
    force: np.ndarray           # [fx, fy, fz] (3,)
    torque: np.ndarray          # [tx, ty, tz] (3,)
    frame: str = 'sensor'       # 'sensor' or 'endeffector' or 'base'
    timestamp: float = 0.0
    
    def __post_init__(self):
        """验证数据格式"""
        assert self.force.shape == (3,), "Force must be (3,) array"
        assert self.torque.shape == (3,), "Torque must be (3,) array"
        assert self.frame in ['sensor', 'endeffector', 'base'], "Invalid frame"
    
    def to_array(self) -> np.ndarray:
        """转换为6D数组 [fx, fy, fz, tx, ty, tz]"""
        return np.concatenate([self.force, self.torque])
    
    @classmethod
    def from_array(cls, wrench_6d: np.ndarray, frame: str = 'sensor', timestamp: float = 0.0):
        """从6D数组创建"""
        assert wrench_6d.shape == (6,), "Wrench must be (6,) array"
        return cls(
            force=wrench_6d[:3].copy(),
            torque=wrench_6d[3:].copy(),
            frame=frame,
            timestamp=timestamp
        )
    
    def copy(self):
        """深拷贝"""
        return WrenchData(
            force=self.force.copy(),
            torque=self.torque.copy(),
            frame=self.frame,
            timestamp=self.timestamp
        )


@dataclass
class ImpedanceParams:
    """阻抗参数（6自由度）"""
    stiffness: np.ndarray       # 刚度 K (6,) [Kx, Ky, Kz, Krx, Kry, Krz]
    damping: np.ndarray         # 阻尼 D (6,) [Dx, Dy, Dz, Drx, Dry, Drz]
    mass: Optional[np.ndarray] = None  # 质量 M (6,) [Mx, My, Mz, Mrx, Mry, Mrz] - 可选
    use_mass: bool = False      # 是否使用质量项
    
    def __post_init__(self):
        """验证数据格式"""
        assert self.stiffness.shape == (6,), "Stiffness must be (6,) array"
        assert self.damping.shape == (6,), "Damping must be (6,) array"
        if self.mass is not None:
            assert self.mass.shape == (6,), "Mass must be (6,) array"
        if self.use_mass and self.mass is None:
            raise ValueError("use_mass=True but mass is None")
    
    @classmethod
    def create_uniform(cls, stiffness: float, damping: float, mass: Optional[float] = None):
        """创建各向同性的阻抗参数"""
        K = np.ones(6) * stiffness
        D = np.ones(6) * damping
        M = np.ones(6) * mass if mass is not None else None
        return cls(stiffness=K, damping=D, mass=M, use_mass=(mass is not None))
    
    @classmethod
    def create_anisotropic(cls, 
                          trans_stiffness: np.ndarray,  # (3,) [Kx, Ky, Kz]
                          rot_stiffness: np.ndarray,    # (3,) [Krx, Kry, Krz]
                          trans_damping: np.ndarray,    # (3,)
                          rot_damping: np.ndarray,      # (3,)
                          trans_mass: Optional[np.ndarray] = None,  # (3,)
                          rot_mass: Optional[np.ndarray] = None):   # (3,)
        """创建各向异性的阻抗参数"""
        K = np.concatenate([trans_stiffness, rot_stiffness])
        D = np.concatenate([trans_damping, rot_damping])
        M = None
        use_mass = False
        if trans_mass is not None and rot_mass is not None:
            M = np.concatenate([trans_mass, rot_mass])
            use_mass = True
        return cls(stiffness=K, damping=D, mass=M, use_mass=use_mass)
    
    def copy(self):
        """深拷贝"""
        return ImpedanceParams(
            stiffness=self.stiffness.copy(),
            damping=self.damping.copy(),
            mass=self.mass.copy() if self.mass is not None else None,
            use_mass=self.use_mass
        )


@dataclass
class ControlOutput:
    """控制输出"""
    joint_positions: np.ndarray  # 关节角度指令 (n_joints,)
    success: bool
    error_msg: str = ""
    debug_info: dict = field(default_factory=dict)  # 调试信息
    
    def __post_init__(self):
        """验证数据格式"""
        if self.success:
            assert len(self.joint_positions) > 0, "Joint positions cannot be empty"
