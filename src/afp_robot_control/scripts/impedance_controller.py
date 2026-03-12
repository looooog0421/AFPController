#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用化阻抗控制器设计
支持：任意轴、任意坐标系、灵活组合位置控制和力控制
"""
import rospy
import pinocchio as pin
import numpy as np
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Tuple, Union, List
from enum import Enum
from collections import deque
import yaml
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped 
from std_msgs.msg import Float32, String

from force_sensor_filter import ForceSensorFilter, GravityCompensator, ForceTorqueData
from afp_robot_control.src.utils.robot_kinimatics import RobotKinematics, RotationType

# ==================== 数据结构定义 =============================
@dataclass
# class RobotState:
#     """机器人状态"""
#     joint_positions: np.ndarray
#     joint_velocities: np.ndarray = field(default_factory=lambda: np.zeros(6))
#     tcp_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
#     tcp_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
#     tcp_velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
#     timestamp: float = 0.0

# @dataclass
# class ControlCommand:
#     """控制指令"""
#     velocity: np.ndarray  # 笛卡尔空间速度 [vx, vy, vz, wx, wy, wz]
#     acceleration: float = 3.0
#     time: float = 0.02


# @dataclass
# class ImpedanceParams:
#     """阻抗控制参数"""
#     target_force: float = 0.0
#     k_lift: float = 0.05  # 抬起刚度
#     k_drop: float = 0.02  # 下压刚度
#     gravity_compensation: float = -0.001
#     max_velocity: float = 0.20
#     min_velocity: float = -0.08

# @dataclass
# class MotionParams:
#     """运动参数"""
#     target_range: float = 0.04
#     hard_limit: float = 0.08
#     cruise_speed: float = 0.02
#     y_axis_invert: float = -1.0

class ControlMode(Enum):
    """控制模式"""
    POSITION = "position" # 位置控制
    FORCE = "force"    # 力控制
    IMPEDANCE = "impedance" # 阻抗控制
    FREE = "free"    # 自由运动

class CoordinateFrame(Enum):
    """坐标系"""
    WORLD = "world"           # 世界坐标系
    TOOL = "tool"             # 工具坐标系
    BASE = "base"             # 基座坐标系

@dataclass
class AxisConfig:
    """单轴配置"""
    axis_index: int                  # 轴索引 (0-5, 对应 x,y,z,rx,ry,rz)
    mode: ControlMode            # 控制模式
    frame: CoordinateFrame       # 坐标系

    # 位置控制参数
    target_position: Optional[float] = None
    position_gain: float = 1.0
    max_velocity: float = 0.1

    # 力控制参数
    target_force: float = 0.0
    force_gain_positive: float = 0.05  # 抬起刚度
    force_gain_negative: float = 0.02  # 下压刚度

    # 限位
    soft_limit_min: Optional[float] = None
    soft_limit_max: Optional[float] = None

@dataclass
class ImpedanceControllerConfig:
    """阻抗控制器配置"""
    axes: List[AxisConfig]      # 各轴配置列表
    reference_position: Optional[np.ndarray] = None
    reference_rotation: Optional[np.ndarray] = None
    enable_gravity_compensation: bool = True
    gravity_drift: float = -0.001
 

# =================== 控制器基类 ===================
class CartesianImpedanceController(ABC):
    """
    通用阻抗控制器基类
    """

    def __init__(
            self, 
            config: ImpedanceControllerConfig,
            kinematics: Optional[RobotKinematics] = None  # 可选的运动学模块
            ):
        self.config = config
        self.kinematics = kinematics

        # 状态变量
        self.reference_