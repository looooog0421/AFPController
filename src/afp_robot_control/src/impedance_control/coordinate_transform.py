#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
坐标系转换模块
处理力传感器坐标系到机器人末端坐标系的转换
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from .impedance_types import WrenchData


class CoordinateTransformer:
    """力传感器坐标系转换器"""
    
    def __init__(self, sensor_to_ee_rotation: np.ndarray = None, rotation_axis: str = None, angle_deg: float = None):
        """
        初始化坐标转换器
        
        Args:
            sensor_to_ee_rotation: 3x3旋转矩阵，传感器坐标系到末端坐标系
            rotation_axis: 旋转轴 ('x', 'y', 'z')，与angle_deg配合使用
            angle_deg: 旋转角度（度），与rotation_axis配合使用
            
        示例:
            # 方式1: 直接提供旋转矩阵
            tf = CoordinateTransformer(sensor_to_ee_rotation=R_matrix)
            
            # 方式2: 指定旋转轴和角度
            tf = CoordinateTransformer(rotation_axis='z', angle_deg=90)  # 绕z轴旋转90度
        """
        if sensor_to_ee_rotation is not None:
            assert sensor_to_ee_rotation.shape == (3, 3), "Rotation matrix must be 3x3"
            self.R_sensor_to_ee = sensor_to_ee_rotation
        elif rotation_axis is not None and angle_deg is not None:
            self.R_sensor_to_ee = self._create_rotation_matrix(rotation_axis, angle_deg)
        else:
            # 默认：无旋转（单位矩阵）
            self.R_sensor_to_ee = np.eye(3)
    
    def _create_rotation_matrix(self, axis: str, angle_deg: float) -> np.ndarray:
        """
        创建绕指定轴的旋转矩阵
        
        Args:
            axis: 旋转轴 ('x', 'y', 'z')
            angle_deg: 旋转角度（度）
        
        Returns:
            3x3旋转矩阵
        """
        axis_map = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
        if axis.lower() not in axis_map:
            raise ValueError(f"Invalid axis: {axis}. Must be 'x', 'y', or 'z'")
        
        axis_vec = np.array(axis_map[axis.lower()])
        angle_rad = np.deg2rad(angle_deg)
        rotation = R.from_rotvec(angle_rad * axis_vec)
        return rotation.as_matrix()
    
    def transform_wrench_to_ee(self, wrench_sensor: WrenchData) -> WrenchData:
        """
        将传感器坐标系的力/力矩转换到末端坐标系
        
        Args:
            wrench_sensor: 传感器坐标系下的力/力矩数据
        
        Returns:
            末端坐标系下的力/力矩数据
        """
        if wrench_sensor.frame != 'sensor':
            raise ValueError(f"Expected frame='sensor', got '{wrench_sensor.frame}'")
        
        # 力的转换: F_ee = R * F_sensor
        force_ee = self.R_sensor_to_ee @ wrench_sensor.force
        
        # 力矩的转换: T_ee = R * T_sensor
        torque_ee = self.R_sensor_to_ee @ wrench_sensor.torque
        
        return WrenchData(
            force=force_ee,
            torque=torque_ee,
            frame='endeffector',
            timestamp=wrench_sensor.timestamp
        )
    
    def transform_wrench_to_base(self, wrench_ee: WrenchData, ee_rotation: np.ndarray) -> WrenchData:
        """
        将末端坐标系的力/力矩转换到基坐标系
        
        Args:
            wrench_ee: 末端坐标系下的力/力矩数据
            ee_rotation: 末端坐标系相对于基坐标系的旋转矩阵 (3x3)
        
        Returns:
            基坐标系下的力/力矩数据
        """
        if wrench_ee.frame != 'endeffector':
            raise ValueError(f"Expected frame='endeffector', got '{wrench_ee.frame}'")
        
        assert ee_rotation.shape == (3, 3), "ee_rotation must be 3x3 matrix"
        
        # 力的转换: F_base = R_base_ee * F_ee
        force_base = ee_rotation @ wrench_ee.force
        
        # 力矩的转换: T_base = R_base_ee * T_ee
        torque_base = ee_rotation @ wrench_ee.torque
        
        return WrenchData(
            force=force_base,
            torque=torque_base,
            frame='base',
            timestamp=wrench_ee.timestamp
        )
    
    def set_rotation_matrix(self, rotation_matrix: np.ndarray):
        """动态设置旋转矩阵"""
        assert rotation_matrix.shape == (3, 3), "Rotation matrix must be 3x3"
        self.R_sensor_to_ee = rotation_matrix
    
    def set_rotation_from_axis_angle(self, axis: str, angle_deg: float):
        """动态设置旋转（通过轴角）"""
        self.R_sensor_to_ee = self._create_rotation_matrix(axis, angle_deg)
    
    def get_rotation_matrix(self) -> np.ndarray:
        """获取当前旋转矩阵"""
        return self.R_sensor_to_ee.copy()


class GravityCompensator:
    """重力补偿器（可选功能）"""
    
    def __init__(self, tool_mass: float, tool_cog: np.ndarray, gravity_vector: np.ndarray = np.array([0, 0, -9.81])):
        """
        初始化重力补偿器
        
        Args:
            tool_mass: 工具质量 (kg)
            tool_cog: 工具质心相对于传感器坐标系的位置 (3,) [x, y, z] (m)
            gravity_vector: 重力加速度向量 (3,) 基坐标系下 (m/s^2)
        """
        self.tool_mass = tool_mass
        self.tool_cog = tool_cog
        self.gravity_vector = gravity_vector
    
    def compensate(self, wrench: WrenchData, ee_rotation: np.ndarray) -> WrenchData:
        """
        补偿重力影响
        
        Args:
            wrench: 原始力/力矩数据（末端坐标系）
            ee_rotation: 末端坐标系相对于基坐标系的旋转矩阵 (3x3)
        
        Returns:
            补偿后的力/力矩数据
        """
        # 重力在末端坐标系下的表示
        gravity_ee = ee_rotation.T @ self.gravity_vector
        
        # 重力产生的力
        gravity_force = self.tool_mass * gravity_ee
        
        # 重力产生的力矩（由于质心偏移）
        gravity_torque = np.cross(self.tool_cog, gravity_force)
        
        # 补偿
        compensated_force = wrench.force - gravity_force
        compensated_torque = wrench.torque - gravity_torque
        
        return WrenchData(
            force=compensated_force,
            torque=compensated_torque,
            frame=wrench.frame,
            timestamp=wrench.timestamp
        )
