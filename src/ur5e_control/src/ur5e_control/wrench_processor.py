#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wrench processor for real-robot force control.

Pipeline:
1. raw wrench ingest
2. optional static bias removal (from calibration file)
3. runtime tare / bias removal
4. low-pass filter
5. sensor -> ee rotation
6. gravity / tool compensation
7. optional point-shift for torque at control point
"""

from dataclasses import dataclass, field
import os
import time
import yaml
import numpy as np

try:
    from .lowpass_filter import LowPassOnlineFilter
except ImportError:
    from lowpass_filter import LowPassOnlineFilter


@dataclass
class WrenchProcessorConfig:
    dt: float
    filter_tau: float = 0.05
    tare_sample_count: int = 100
    sensor_to_ee_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    tool_mass: float = 0.0
    tool_cog: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gravity_vector: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    static_force_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    static_torque_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    sensor_to_control_point: np.ndarray = field(default_factory=lambda: np.zeros(3))
    enable_point_shift: bool = False
    wrench_timeout: float = 0.2

    def __post_init__(self):
        self.sensor_to_ee_rotation = np.array(self.sensor_to_ee_rotation, dtype=float)
        self.tool_cog = np.array(self.tool_cog, dtype=float)
        self.gravity_vector = np.array(self.gravity_vector, dtype=float)
        self.static_force_offset = np.array(self.static_force_offset, dtype=float)
        self.static_torque_offset = np.array(self.static_torque_offset, dtype=float)
        self.sensor_to_control_point = np.array(self.sensor_to_control_point, dtype=float)

        if self.sensor_to_ee_rotation.shape != (3, 3):
            raise ValueError("sensor_to_ee_rotation must be (3, 3)")
        if self.tool_cog.shape != (3,):
            raise ValueError("tool_cog must be (3,)")
        if self.gravity_vector.shape != (3,):
            raise ValueError("gravity_vector must be (3,)")
        if self.static_force_offset.shape != (3,):
            raise ValueError("static_force_offset must be (3,)")
        if self.static_torque_offset.shape != (3,):
            raise ValueError("static_torque_offset must be (3,)")
        if self.sensor_to_control_point.shape != (3,):
            raise ValueError("sensor_to_control_point must be (3,)")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.tare_sample_count <= 0:
            raise ValueError("tare_sample_count must be positive")
        if self.wrench_timeout <= 0.0:
            raise ValueError("wrench_timeout must be positive")

    @property
    def static_wrench_offset(self) -> np.ndarray:
        return np.concatenate([self.static_force_offset, self.static_torque_offset])

    @classmethod
    def from_yaml(cls,
                  file_path: str,
                  dt: float,
                  filter_tau: float = 0.05,
                  tare_sample_count: int = 100,
                  wrench_timeout: float = 0.2,
                  enable_point_shift: bool = False):
        config = cls(
            dt=dt,
            filter_tau=filter_tau,
            tare_sample_count=tare_sample_count,
            wrench_timeout=wrench_timeout,
            enable_point_shift=enable_point_shift,
        )

        if not file_path or not os.path.exists(file_path):
            return config

        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if "T_sensor_to_ee" in data:
            T_sensor_to_ee = np.array(data["T_sensor_to_ee"], dtype=float)
            if T_sensor_to_ee.shape == (4, 4):
                config.sensor_to_ee_rotation = T_sensor_to_ee[:3, :3]
                sensor_origin_in_ee = T_sensor_to_ee[:3, 3]
            else:
                sensor_origin_in_ee = np.zeros(3)
        else:
            sensor_origin_in_ee = np.zeros(3)

        if "mass_kg" in data:
            config.tool_mass = float(data["mass_kg"])

        if "center_of_mass_m" in data:
            config.tool_cog = np.array(data["center_of_mass_m"], dtype=float)

        if "static_force_offset_N" in data:
            config.static_force_offset = np.array(data["static_force_offset_N"], dtype=float)

        if "static_torque_offset_Nm" in data:
            config.static_torque_offset = np.array(data["static_torque_offset_Nm"], dtype=float)

        if "T_ee_to_tool" in data:
            T_ee_to_tool = np.array(data["T_ee_to_tool"], dtype=float)
            if T_ee_to_tool.shape == (4, 4):
                tool_origin_in_ee = T_ee_to_tool[:3, 3]
                config.sensor_to_control_point = tool_origin_in_ee - sensor_origin_in_ee

        return config


@dataclass
class ProcessedWrenchState:
    raw_sensor: np.ndarray = field(default_factory=lambda: np.zeros(6))
    static_bias_removed_sensor: np.ndarray = field(default_factory=lambda: np.zeros(6))
    tared_sensor: np.ndarray = field(default_factory=lambda: np.zeros(6))
    filtered_sensor: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ee_wrench: np.ndarray = field(default_factory=lambda: np.zeros(6))
    compensated_ee_wrench: np.ndarray = field(default_factory=lambda: np.zeros(6))
    tare_bias: np.ndarray = field(default_factory=lambda: np.zeros(6))
    timestamp: float = 0.0
    tare_done: bool = False
    ready: bool = False
    stale: bool = True
    fault: str = ""


class WrenchProcessor:
    def __init__(self, config: WrenchProcessorConfig):
        self.config = config
        self.state = ProcessedWrenchState()
        self._tare_samples = []
        self._filter = LowPassOnlineFilter(
            dimension=6,
            tau=self.config.filter_tau,
            dt=self.config.dt,
            initial_states=np.zeros(6)
        )

    def reset_tare(self):
        self._tare_samples = []
        self.state.tare_bias = np.zeros(6)
        self.state.tare_done = False
        self.state.ready = False
        self.state.stale = True
        self.state.fault = ""
        self._filter = LowPassOnlineFilter(
            dimension=6,
            tau=self.config.filter_tau,
            dt=self.config.dt,
            initial_states=np.zeros(6)
        )

    def is_ready(self, now: float = None) -> bool:
        if now is None:
            now = time.time()
        if not self.state.tare_done:
            return False
        if self.state.fault:
            return False
        if self.state.timestamp <= 0.0:
            return False
        if (now - self.state.timestamp) > self.config.wrench_timeout:
            return False
        return True

    def update(self, raw_wrench: np.ndarray, ee_rotation: np.ndarray = None, timestamp: float = None) -> ProcessedWrenchState:
        if timestamp is None:
            timestamp = time.time()

        raw_wrench = np.array(raw_wrench, dtype=float)
        if raw_wrench.shape != (6,):
            raise ValueError("raw_wrench must be (6,)")

        self.state.raw_sensor = raw_wrench.copy()
        self.state.timestamp = float(timestamp)
        self.state.stale = False
        self.state.fault = ""

        static_bias_removed = raw_wrench - self.config.static_wrench_offset
        self.state.static_bias_removed_sensor = static_bias_removed.copy()

        if not self.state.tare_done:
            self._tare_samples.append(static_bias_removed.copy())
            if len(self._tare_samples) >= self.config.tare_sample_count:
                self.state.tare_bias = np.mean(self._tare_samples, axis=0)
                self.state.tare_done = True
            self.state.ready = False
            self.state.tared_sensor = np.zeros(6)
            self.state.filtered_sensor = np.zeros(6)
            self.state.ee_wrench = np.zeros(6)
            self.state.compensated_ee_wrench = np.zeros(6)
            return self.state

        tared = static_bias_removed - self.state.tare_bias
        filtered = self._filter.update(tared)
        self.state.tared_sensor = tared.copy()
        self.state.filtered_sensor = filtered.copy()

        force_ee = self.config.sensor_to_ee_rotation @ filtered[:3]
        torque_ee = self.config.sensor_to_ee_rotation @ filtered[3:]
        ee_wrench = np.concatenate([force_ee, torque_ee])
        self.state.ee_wrench = ee_wrench.copy()

        compensated = ee_wrench.copy()
        if self.config.tool_mass > 0.0:
            if ee_rotation is None:
                self.state.compensated_ee_wrench = compensated
                self.state.ready = False
                self.state.fault = "missing_ee_rotation"
                return self.state

            ee_rotation = np.array(ee_rotation, dtype=float)
            if ee_rotation.shape != (3, 3):
                self.state.compensated_ee_wrench = compensated
                self.state.ready = False
                self.state.fault = "invalid_ee_rotation"
                return self.state

            gravity_ee = ee_rotation.T @ self.config.gravity_vector
            gravity_force = self.config.tool_mass * gravity_ee
            gravity_torque = np.cross(self.config.tool_cog, gravity_force)
            compensated[:3] -= gravity_force
            compensated[3:] -= gravity_torque

        if self.config.enable_point_shift:
            compensated[3:] -= np.cross(self.config.sensor_to_control_point, compensated[:3])

        self.state.compensated_ee_wrench = compensated
        self.state.ready = True
        return self.state
