#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os
import rospy
import tf.transformations as tf_trans

class GravityCompensator:
    """
    重力补偿计算器
    """
    def __init__(self, config_path):
        self.mass = None
        self.com_position = None  # 重心位置
        self.f_bias = np.zeros(3)  # 力传感器零偏
        self.t_bias = np.zeros(3)  # 力传感器零偏
        self.is_calibrated = False
        self.gravity = 9.81  # 重力加速度,单位:m/s^2

        # 工具到力传感器的变换矩阵
        self.R_tool_sensor = tf_trans.rotation_matrix(0, [0, 1, 0])[:3, :3]

        self.load_config(config_path)

    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                self.mass = data['mass_kg']
                self.com = np.array(data['center_of_mass_m'])
                self.f_bias = np.array(data['static_force_offset_N'])
                self.t_bias = np.array(data['static_torque_offset_Nm'])
                self.is_calibrated = True
                rospy.loginfo("Gravity calibration loaded successfully.")
        except Exception as e:
            rospy.logwarn(f"Failed to load gravity calibration: {e}")
            self.is_calibrated = False
    
    def get_compensated_wrench(self, raw_force, raw_torque, tool_quat):
        """
        input:
            raw_force: 原始力传感器读数,numpy数组,单位:N
            raw_torque: 原始力传感器读数,numpy数组,单位:Nm
            tool_quat: 工具末端四元数表示的姿态,numpy数组 [x, y, z, w]
        output:
            comp_force: 补偿后的力,numpy数组,单位:N
            comp_torque: 补偿后的力矩,numpy数组,单位:Nm
        """
        if not self.is_calibrated:
            return raw_force, raw_torque
        
        # 1. 计算传感器再世界坐标系的姿态
        R_world_body = tf_trans.quaternion_matrix(tool_quat)[:3, :3]
        R_world_sensor = np.dot(R_world_body, self.R_tool_sensor)

        # 2. 计算重力引起的力
        g_vec_world = np.array([0, 0, -self.mass * self.gravity])  # 重力向量
        g_vec_sensor = np.dot(R_world_sensor.T, g_vec_world)  # 转换到传感器坐标系

        # 3. 计算补偿后的力矩
        t_gravity_sensor = np.cross(self.com, g_vec_sensor)

        # 4. 计算补偿后的力和力矩
        # F_contact = F_raw - F_gravity - F_bias
        comp_force = raw_force - g_vec_sensor - self.f_bias
        # T_contact = T_raw - T_gravity - T_bias
        comp_torque = raw_torque - t_gravity_sensor - self.t_bias

        return comp_force, comp_torque