#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo vs Pinocchio 运动学一致性检查
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))
from impedance_control import CartesianState
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper

def compare_kinematics():
    """比较MuJoCo和Pinocchio的正运动学"""
    print("=" * 60)
    print("运动学一致性检查")
    print("=" * 60)
    
    # 路径
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 加载模型
    print("\n1. 加载模型...")
    mj_model = mujoco.MjModel.from_xml_path(model_path)
    mj_data = mujoco.MjData(mj_model)
    pin_kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=True)
    print(f"   ✓ MuJoCo自由度: {mj_model.nv}")
    print(f"   ✓ Pinocchio关节数: {pin_kin.n_joints}")
    print(f"   ✓ 坐标系转换: MuJoCo frame enabled")
    
    # 测试多个关节配置
    test_configs = [
        ("零位", np.zeros(6)),
        ("典型姿态", np.array([0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0])),
        ("随机姿态", np.array([0.5, -1.0, 1.5, -0.5, 1.5, 0.3])),
    ]
    
    print("\n2. 比较正运动学结果...")
    print("-" * 60)
    
    for name, q in test_configs:
        print(f"\n配置: {name}")
        print(f"关节角度 (度): {np.round(np.rad2deg(q), 1)}")
        
        # MuJoCo正运动学
        mj_data.qpos[:6] = q
        mujoco.mj_forward(mj_model, mj_data)
        
        # 找到tool0 body
        tool0_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "tool0")
        if tool0_id < 0:
            # 尝试flange
            tool0_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "flange")
        if tool0_id < 0:
            # 使用wrist_3_link
            tool0_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        
        mj_pos = mj_data.xpos[tool0_id].copy()
        mj_rot = mj_data.xmat[tool0_id].reshape(3, 3).copy()
        
        # Pinocchio正运动学
        pin_state = pin_kin.forward_kinematics(q)
        pin_pos = pin_state.position
        pin_T = pin_state.to_pose_matrix()
        pin_rot = pin_T[:3, :3]
        
        # 比较结果
        pos_diff = np.linalg.norm(mj_pos - pin_pos)
        
        print(f"  MuJoCo位置:    {np.round(mj_pos, 4)}")
        print(f"  Pinocchio位置: {np.round(pin_pos, 4)}")
        print(f"  位置差异:      {pos_diff*1000:.2f} mm")
        
        # 计算旋转差异
        rot_diff = np.linalg.norm(mj_rot - pin_rot, 'fro')
        print(f"  旋转矩阵差异:  {rot_diff:.4f}")
        
        if pos_diff > 0.01:  # 超过1cm
            print(f"  ⚠️  位置差异过大！")
        else:
            print(f"  ✓ 位置一致")
    
    print("\n" + "=" * 60)
    print("检查完成！")
    print("=" * 60)
    
    # 如果差异大，打印关节信息
    print("\n3. 关节信息对比:")
    print("\nMuJoCo关节:")
    for i in range(min(6, mj_model.njnt)):
        jnt_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  {i}: {jnt_name}")
    
    print("\nPinocchio关节:")
    for i in range(pin_kin.model.njoints):
        if i < len(pin_kin.model.names):
            print(f"  {i}: {pin_kin.model.names[i]}")
    
    print("\n提示:")
    print("- 如果位置差异<10mm，运动学基本一致")
    print("- 如果位置差异>50mm，可能存在:")
    print("  * 坐标系定义不同")
    print("  * 关节顺序不同")
    print("  * DH参数不同")
    print("  * 末端定义不同（flange vs tool0）")

if __name__ == '__main__':
    compare_kinematics()
