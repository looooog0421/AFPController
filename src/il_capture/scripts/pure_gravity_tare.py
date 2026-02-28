#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
根据六个标准姿态下的力传感器数据计算负载质量、重心和零漂
"""

import numpy as np
import pandas as pd
import os
import sys


class GravityCalibrator:
    def __init__(self, csv_file):
        """
        初始化重力标定器
        
        Args:
            csv_file: CSV数据文件路径
        """
        self.csv_file = csv_file
        self.data = None
        self.g = 9.81  # 重力加速度 m/s^2
        
    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file, skipinitialspace=True)
            print("=" * 60)
            print("Loaded data from:", self.csv_file)
            print("=" * 60)
            print(self.data)
            print("=" * 60)
            return True
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def calibrate(self):
        """
        执行标定计算
        
        原理：
        1. 力传感器测量值 = 真实重力 + 零漂
        2. 力矩测量值 = 重心产生的力矩 + 零漂
        3. 在不同姿态下，重力方向改变，但质量和重心不变
        """
        
        # 提取各姿态数据
        poses = ['z_down', 'z_up', 'x_down', 'x_up', 'y_down', 'y_up']
        
        # 检查数据完整性
        for pose in poses:
            if pose not in self.data['pose'].values:
                print(f"Error: Missing pose '{pose}' in data")
                return None
        
        # 提取力和力矩数据
        forces = {}
        torques = {}
        
        for pose in poses:
            row = self.data[self.data['pose'] == pose].iloc[0]
            forces[pose] = np.array([row['forcex'], row['forcey'], row['forcez']])
            torques[pose] = np.array([row['torquex'], row['torquey'], row['torquez']])
        
        print("\nExtracted force and torque data:")
        for pose in poses:
            print(f"{pose:8s}: F={forces[pose]}, T={torques[pose]}")
        
        # ===== 步骤1: 计算质量和力传感器零漂 =====
        print("\n" + "=" * 60)
        print("Step 1: Calculate mass and force bias")
        print("=" * 60)
        
        # 在六个姿态下，重力方向分别为：
        # z_down:  [0, 0, +g]
        # z_up:    [0, 0, -g]
        # x_down:  [+g, 0, 0]
        # x_up:    [-g, 0, 0]
        # y_down:  [0, +g, 0]
        # y_up:    [0, -g, 0]
        
        # 构建方程组: F_measured = m * g_direction + F_bias
        # 对于每个轴，我们有多个方程
        
        # 方法1: 利用对称性
        # Z轴: F_z_down + F_z_up = 2 * F_bias_z
        # X轴: F_x_down + F_x_up = 2 * F_bias_x
        # Y轴: F_y_down + F_y_up = 2 * F_bias_y
        
        f_bias_x = (forces['x_down'][0] + forces['x_up'][0]) / 2.0
        f_bias_y = (forces['y_down'][1] + forces['y_up'][1]) / 2.0
        f_bias_z = (forces['z_down'][2] + forces['z_up'][2]) / 2.0
        
        force_bias = np.array([f_bias_x, f_bias_y, f_bias_z])
        
        print(f"Force bias (N): [{f_bias_x:.6f}, {f_bias_y:.6f}, {f_bias_z:.6f}]")
        
        # 计算质量
        # Z轴: m*g = (F_z_down - F_z_up) / 2
        # X轴: m*g = (F_x_down - F_x_up) / 2
        # Y轴: m*g = (F_y_down - F_y_up) / 2
        
        mass_from_z = abs(forces['z_down'][2] - forces['z_up'][2]) / (2.0 * self.g)
        mass_from_x = abs(forces['x_down'][0] - forces['x_up'][0]) / (2.0 * self.g)
        mass_from_y = abs(forces['y_down'][1] - forces['y_up'][1]) / (2.0 * self.g)
        
        print(f"\nMass estimates:")
        print(f"  From Z-axis: {mass_from_z:.6f} kg")
        print(f"  From X-axis: {mass_from_x:.6f} kg")
        print(f"  From Y-axis: {mass_from_y:.6f} kg")
        
        # 取平均
        mass = np.mean([mass_from_z, mass_from_x, mass_from_y])
        mass_std = np.std([mass_from_z, mass_from_x, mass_from_y])
        
        print(f"\nAverage mass: {mass:.6f} ± {mass_std:.6f} kg")
        
        # ===== 步骤2: 计算重心和力矩零漂 =====
        print("\n" + "=" * 60)
        print("Step 2: Calculate center of mass and torque bias")
        print("=" * 60)
        
        # 力矩方程: T_measured = r × F_gravity + T_bias
        # 其中 r 是重心位置向量
        
        # 对于重力沿Z轴向下 (z_down): g = [0, 0, +g]
        # T = [rx, ry, rz] × m*[0, 0, g] + T_bias
        # T = [ry*m*g, -rx*m*g, 0] + T_bias
        
        # 构建方程组
        # z_down:  T = [ry*m*g, -rx*m*g, 0] + T_bias
        # z_up:    T = [-ry*m*g, rx*m*g, 0] + T_bias
        # x_down:  T = [0, rz*m*g, -ry*m*g] + T_bias
        # x_up:    T = [0, -rz*m*g, ry*m*g] + T_bias
        # y_down:  T = [-rz*m*g, 0, rx*m*g] + T_bias
        # y_up:    T = [rz*m*g, 0, -rx*m*g] + T_bias
        
        # 利用对称性求解力矩零漂
        t_bias_x = (torques['z_down'][0] + torques['z_up'][0]) / 2.0
        t_bias_y = (torques['z_down'][1] + torques['z_up'][1]) / 2.0
        t_bias_z = (torques['x_down'][2] + torques['x_up'][2]) / 2.0
        
        # 也可以从其他轴验证
        t_bias_x_check = (torques['y_down'][0] + torques['y_up'][0]) / 2.0
        t_bias_y_check = (torques['x_down'][1] + torques['x_up'][1]) / 2.0
        t_bias_z_check = (torques['y_down'][2] + torques['y_up'][2]) / 2.0
        
        torque_bias = np.array([
            np.mean([t_bias_x, t_bias_x_check]),
            np.mean([t_bias_y, t_bias_y_check]),
            np.mean([t_bias_z, t_bias_z_check])
        ])
        
        print(f"Torque bias (Nm): [{torque_bias[0]:.6f}, {torque_bias[1]:.6f}, {torque_bias[2]:.6f}]")
        
        # 计算重心位置
        # 从 z_down 和 z_up:
        # ry = (T_x - T_bias_x) / (m * g)  [平均z_down和z_up]
        # rx = -(T_y - T_bias_y) / (m * g)
        
        ry_from_z = (torques['z_down'][0] - torque_bias[0]) / (mass * self.g)
        rx_from_z = -(torques['z_down'][1] - torque_bias[1]) / (mass * self.g)
        
        # 从 x_down 和 x_up:
        rz_from_x = (torques['x_down'][1] - torque_bias[1]) / (mass * self.g)
        ry_from_x = -(torques['x_down'][2] - torque_bias[2]) / (mass * self.g)
        
        # 从 y_down 和 y_up:
        rz_from_y = -(torques['y_down'][0] - torque_bias[0]) / (mass * self.g)
        rx_from_y = (torques['y_down'][2] - torque_bias[2]) / (mass * self.g)
        
        print(f"\nCenter of mass estimates:")
        print(f"  rx from Z-axis: {rx_from_z:.6f} m, from Y-axis: {rx_from_y:.6f} m")
        print(f"  ry from Z-axis: {ry_from_z:.6f} m, from X-axis: {ry_from_x:.6f} m")
        print(f"  rz from X-axis: {rz_from_x:.6f} m, from Y-axis: {rz_from_y:.6f} m")
        
        # 取平均
        center_of_mass = np.array([
            np.mean([rx_from_z, rx_from_y]),
            np.mean([ry_from_z, ry_from_x]),
            np.mean([rz_from_x, rz_from_y])
        ])
        
        print(f"\nAverage center of mass (m): [{center_of_mass[0]:.6f}, {center_of_mass[1]:.6f}, {center_of_mass[2]:.6f}]")
        
        # ===== 返回结果 =====
        results = {
            'mass': mass,
            'mass_std': mass_std,
            'center_of_mass': center_of_mass,
            'force_bias': force_bias,
            'torque_bias': torque_bias
        }
        
        return results
    
    def display_results(self, results):
        """显示标定结果"""
        if results is None:
            print("No results to display")
            return
        
        print("\n" + "=" * 60)
        print("CALIBRATION RESULTS")
        print("=" * 60)
        
        print(f"\nMass:")
        print(f"  {results['mass']:.6f} ± {results['mass_std']:.6f} kg")
        
        print(f"\nCenter of Mass (relative to force sensor frame):")
        print(f"  X: {results['center_of_mass'][0]:.6f} m")
        print(f"  Y: {results['center_of_mass'][1]:.6f} m")
        print(f"  Z: {results['center_of_mass'][2]:.6f} m")
        print(f"  Distance from origin: {np.linalg.norm(results['center_of_mass']):.6f} m")
        
        print(f"\nForce Sensor Bias (zero offset):")
        print(f"  Fx: {results['force_bias'][0]:.6f} N")
        print(f"  Fy: {results['force_bias'][1]:.6f} N")
        print(f"  Fz: {results['force_bias'][2]:.6f} N")
        
        print(f"\nTorque Sensor Bias (zero offset):")
        print(f"  Tx: {results['torque_bias'][0]:.6f} Nm")
        print(f"  Ty: {results['torque_bias'][1]:.6f} Nm")
        print(f"  Tz: {results['torque_bias'][2]:.6f} Nm")
        
        print("\n" + "=" * 60)
        print("FORMATTED OUTPUT FOR YAML:")
        print("=" * 60)
        print(f"mass_kg: {results['mass']:.6f}")
        print(f"center_of_mass_m: [{results['center_of_mass'][0]:.6f}, {results['center_of_mass'][1]:.6f}, {results['center_of_mass'][2]:.6f}]")
        print(f"static_force_offset_N: [{results['force_bias'][0]:.6f}, {results['force_bias'][1]:.6f}, {results['force_bias'][2]:.6f}]")
        print(f"static_torque_offset_Nm: [{results['torque_bias'][0]:.6f}, {results['torque_bias'][1]:.6f}, {results['torque_bias'][2]:.6f}]")
        print("=" * 60)
        
    def verify_results(self, results):
        """验证标定结果"""
        print("\n" + "=" * 60)
        print("VERIFICATION")
        print("=" * 60)
        
        mass = results['mass']
        com = results['center_of_mass']
        f_bias = results['force_bias']
        t_bias = results['torque_bias']
        
        poses = ['z_down', 'z_up', 'x_down', 'x_up', 'y_down', 'y_up']
        gravity_directions = {
            'z_down': np.array([0, 0, 1]),
            'z_up': np.array([0, 0, -1]),
            'x_down': np.array([1, 0, 0]),
            'x_up': np.array([-1, 0, 0]),
            'y_down': np.array([0, 1, 0]),
            'y_up': np.array([0, -1, 0])
        }
        
        print("\nComparing measured vs. predicted values:\n")
        print(f"{'Pose':<10s} {'Axis':<5s} {'Measured':<12s} {'Predicted':<12s} {'Error':<12s}")
        print("-" * 60)
        
        for pose in poses:
            row = self.data[self.data['pose'] == pose].iloc[0]
            f_measured = np.array([row['forcex'], row['forcey'], row['forcez']])
            t_measured = np.array([row['torquex'], row['torquey'], row['torquez']])
            
            # 预测值
            g_dir = gravity_directions[pose]
            f_predicted = mass * self.g * g_dir + f_bias
            t_predicted = np.cross(com, mass * self.g * g_dir) + t_bias
            
            # 力误差
            f_error = f_measured - f_predicted
            print(f"{pose:<10s} Fx    {f_measured[0]:>10.4f}  {f_predicted[0]:>10.4f}  {f_error[0]:>10.4f}")
            print(f"{'':<10s} Fy    {f_measured[1]:>10.4f}  {f_predicted[1]:>10.4f}  {f_error[1]:>10.4f}")
            print(f"{'':<10s} Fz    {f_measured[2]:>10.4f}  {f_predicted[2]:>10.4f}  {f_error[2]:>10.4f}")
            
            # 力矩误差
            t_error = t_measured - t_predicted
            print(f"{'':<10s} Tx    {t_measured[0]:>10.4f}  {t_predicted[0]:>10.4f}  {t_error[0]:>10.4f}")
            print(f"{'':<10s} Ty    {t_measured[1]:>10.4f}  {t_predicted[1]:>10.4f}  {t_error[1]:>10.4f}")
            print(f"{'':<10s} Tz    {t_measured[2]:>10.4f}  {t_predicted[2]:>10.4f}  {t_error[2]:>10.4f}")
            print()
        
        print("=" * 60)


def main():
    # 获取CSV文件路径
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # 默认路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file = '/home/lgx/Project/AFP/src/il_capture/data/gravity_tare.csv'
    
    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found: {csv_file}")
        print("Usage: python3 calculate_gravity_calibration.py [csv_file]")
        sys.exit(1)
    
    # 创建标定器
    calibrator = GravityCalibrator(csv_file)
    
    # 加载数据
    if not calibrator.load_data():
        sys.exit(1)
    
    # 执行标定
    results = calibrator.calibrate()
    
    if results is not None:
        # 显示结果
        calibrator.display_results(results)
        
        # 验证结果
        calibrator.verify_results(results)


if __name__ == '__main__':
    main()