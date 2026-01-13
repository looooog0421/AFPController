#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple
import yaml

# ==================== 数据结构定义 =============================
@dataclass
class ForceTorqueData:
    """力/力矩传感器数据"""
    force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    torque: np.ndarray = field(default_factory=lambda: np.zeros(3))
    timestamp: float = 0.0

# =================== 传感器处理模块 ===================
@dataclass
class GravityCompensator:
    """重力补偿模块"""
    mass: float # 物体质量(kg)
    center_of_mass: np.ndarray # 重心位置(相对于TCP坐标系)
    static_force_offset: np.ndarray # 静态力偏置
    static_torque_offset: np.ndarray # 静态力矩偏置

    @classmethod
    def from_yaml(cls, filepath: str) -> 'GravityCompensator':
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            return cls(
                mass = float(data["mass_kg"]),
                center_of_mass=np.array(data['center_of_mass_m'], dtype=float),
                static_force_offset=np.array(data['static_force_offset_N'], dtype=float),
                static_torque_offset=np.array(data['static_torque_offset_Nm'], dtype=float),
            )
        except Exception as e:
            rospy.logerr(f"Failed to load gravity compensation from {filepath}: {e}")
            raise
    
    def __str__(self):
        return (f"GravityCompensation:\n"
                f"  Mass: {self.mass:.5f} kg\n"
                f"  CoM: [{self.center_of_mass[0]:.5f}, "
                f"{self.center_of_mass[1]:.5f}, "
                f"{self.center_of_mass[2]:.5f}] m\n"
                f"  Force Offset: [{self.static_force_offset[0]:.3f}, "
                f"{self.static_force_offset[1]:.3f}, "
                f"{self.static_force_offset[2]:.3f}] N\n"
                f"  Torque Offset: [{self.static_torque_offset[0]:.5f}, "
                f"{self.static_torque_offset[1]:.5f}, "
                f"{self.static_torque_offset[2]:.5f}] Nm")

class ForceSensorFilter:
    """力传感器滤波器"""
    
    def __init__(
            self,
            alpha: float = 0.1,
            gravity_comp_yaml: Optional[str] = None
            ):
        self.alpha = alpha
        self.gravity_comp = Optional[GravityCompensator] = None
        
        # 加载重力补偿参数
        if gravity_comp_yaml:
            self.load_gravity_compensation(gravity_comp_yaml)
        
        # 滤波器
        self.filtered_force = np.zeros(3)
        self.filtered_torque = np.zeros(3)

    def load_gravity_compensation(self, filepath: str):
        try:
            rospy.loginfo(f"Loaded gravity compensation:\n{self.gravity_comp}")
            self.gravity_comp = GravityCompensator.from_yaml(filepath)
        except Exception as e:
            rospy.logerr(f"Error loading gravity compensation: {e}")
            self.gravity_comp = None

    def update(
            self,
            raw_force: np.ndarray,
            raw_torque: np.ndarray
            ) -> Tuple[np.ndarray, np.ndarray]:
        """更新滤波器并返回补偿后的力/力矩"""

        # 重力补偿, TODO: 这里只对传感器的静态偏置进行了补偿，没有考虑重力补偿，重力补偿需要根据当前姿态计算，后续可考虑在机械臂层面实现
        if self.gravity_comp is not None:
            compensated_force = raw_force - self.gravity_comp.static_force_offset
            compensated_torque = raw_torque - self.gravity_comp.static_torque_offset
        else:
            compensated_force = raw_force
            compensated_torque = raw_torque
        
        # 低通滤波
        self.filtered_force = (self.alpha * compensated_force + 
                               (1 - self.alpha) * self.filtered_force)
        self.filtered_torque = (self.alpha * compensated_torque + 
                                (1 - self.alpha) * self.filtered_torque)

        return self.filtered_force, self.filtered_torque

    def update_force_only(
            self,
            raw_force: np.ndarray
            ) -> np.ndarray:
        """仅更新力的滤波器并返回补偿后的力"""
        filtered_force, _ = self.update(raw_force, np.zeros(3))
        return filtered_force

    def reset(self):
        """重置滤波器状态"""
        self.filtered_force = np.zeros(3)
        self.filtered_torque = np.zeros(3)

    @property
    def is_ready(self) -> bool:
        """检查重力补偿参数是否已加载"""
        return self.gravity_comp is not None