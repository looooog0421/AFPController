#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试1: Pinocchio基础功能
验证FK和position-only IK是否正常工作
"""
import numpy as np
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper
from impedance_control import CartesianState

def test_pinocchio_basic():
    print("\n" + "="*60)
    print("测试1: Pinocchio基础功能")
    print("="*60)
    
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 1. 加载模型
    print("\n1. 加载Pinocchio模型...")
    kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=False)
    print(f"   ✓ 关节数: {kin.n_joints}")
    
    # 2. 测试FK
    print("\n2. 测试正运动学...")
    q_test = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])
    print(f"   输入关节角度 (度): {np.round(np.rad2deg(q_test), 1)}")
    
    pose = kin.forward_kinematics(q_test)
    print(f"   输出位置: {np.round(pose.position, 3)}")
    print(f"   输出姿态: {np.round(pose.orientation, 3)}")
    print("   ✓ FK成功")
    
    # 3. 测试position-only IK
    print("\n3. 测试Position-only逆运动学...")
    kin.set_ik_parameters(eps=1e-3, max_iter=200, damping=1e-4, position_only=True)
    
    # 目标：在z方向移动5mm
    target_pose = pose.copy()
    target_pose.position[2] += 0.005
    print(f"   目标位置: {np.round(target_pose.position, 3)}")
    
    q_solution, success = kin.inverse_kinematics(target_pose, q_test)
    
    if success:
        print(f"   ✓ IK收敛成功")
        print(f"   解关节角度 (度): {np.round(np.rad2deg(q_solution), 1)}")
        
        # 验证解的正确性
        verify_pose = kin.forward_kinematics(q_solution)
        pos_error = np.linalg.norm(verify_pose.position - target_pose.position)
        print(f"   验证位置误差: {pos_error*1000:.2f}mm")
        
        if pos_error < 0.002:  # 2mm
            print("   ✓ IK解验证成功")
            return True
        else:
            print("   ✗ IK解误差过大")
            return False
    else:
        print(f"   ✗ IK未收敛")
        return False

if __name__ == '__main__':
    success = test_pinocchio_basic()
    print("\n" + "="*60)
    if success:
        print("测试1通过 ✓")
    else:
        print("测试1失败 ✗")
    print("="*60)
