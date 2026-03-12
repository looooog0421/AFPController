#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试9: 纯位置控制 - 接触测试
机器人末端接近平面并进行XY方向移动，测试位置控制是否正常工作
"""
import sys
import os
import numpy as np
import mujoco
import mujoco.viewer
import time
import pinocchio

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))
from impedance_control import (
    CartesianState,
    StandardStrategy,
    CartesianImpedanceController
)


def main():
    print("\n" + "="*60)
    print("测试9: 纯位置控制 - 接触测试")
    print("="*60)
    
    # 加载模型
    print("\n1. 加载模型...")
    xml_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene_contact.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print(f"场景中的geom数量: {model.ngeom}")
    
    # 初始化控制器
    print("\n2. 初始化控制器...")
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    controller = CartesianImpedanceController(
        urdf_path=urdf_path,
        strategy=StandardStrategy(),
        control_frequency=500.0,
        use_mujoco_frame=False
    )
    controller.robot_kin.set_ik_parameters(
        eps=1e-3, max_iter=500, damping=1e-4, position_only=False
    )
    
    # 获取Pinocchio model和data（用于重力补偿）
    model_pin = controller.robot_kin.model
    data_pin = controller.robot_kin.data
    
    # 重力补偿参数
    ENABLE_GRAVITY_COMPENSATION = True
    kp_joints = np.array([5000, 5000, 5000, 1000, 1000, 1000])
    print(f"\n重力补偿: {'启用' if ENABLE_GRAVITY_COMPENSATION else '禁用'}")
    
    # 初始姿态
    q_init = np.array([-np.pi/4, -np.pi/2.2, np.pi/2.5, -np.pi/2, -np.pi/2, 0.0])
    data.qpos[:6] = q_init
    data.ctrl[:6] = q_init
    mujoco.mj_forward(model, data)
    
    # 计算初始位姿
    initial_pose = controller.robot_kin.forward_kinematics(q_init)
    print(f"\n初始末端位置: {initial_pose.position}")
    print(f"初始末端Z高度: {initial_pose.position[2]:.4f}m")
    
    # 设置接触平面高度
    contact_height = 0.40  # 桌面顶部在Z=0.40m（400mm）
    print(f"接触平面高度: {contact_height}m")
    
    # 定义轨迹：先垂直下降，再沿45度方向移动
    # 阶段1：移动到桌面上方起始位置（高于桌面30mm - 安全距离）
    approach_position = initial_pose.position.copy()
    approach_position[2] = contact_height + 0.03  # 高于桌面30mm
    
    # 阶段2：下降到更接近桌面（高于桌面10mm）
    contact_position = approach_position.copy()
    contact_position[2] = contact_height  # 高于桌面10mm
    
    # 阶段3：XY平面运动45度方向
    start_position = contact_position.copy()
    end_position = start_position.copy()
    end_position[0] += 0.10  # X方向+100mm
    end_position[1] += 0.10  # Y方向+100mm（45度）
    end_position[2] = contact_height  # Z保持高度
    
    print(f"\n轨迹阶段1 (接近): {approach_position}")
    print(f"轨迹阶段2 (下降): {contact_position}")
    print(f"轨迹阶段3 (起点): {start_position}")
    print(f"轨迹阶段3 (终点): {end_position}")
    
    def smooth_trajectory(t, t_total, p_start, p_end):
        """五次多项式轨迹"""
        if t >= t_total:
            return p_end
        tau = t / t_total
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        return p_start + s * (p_end - p_start)
    
    def quaternion_distance(q1, q2):
        """计算两个四元数之间的角度差（度）"""
        dot = np.abs(np.dot(q1, q2))
        dot = np.clip(dot, -1.0, 1.0)
        angle_rad = 2 * np.arccos(dot)
        return np.degrees(angle_rad)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 第一阶段：移动到接近位置
        print("\n3. 移动到接近位置...")
        move_duration = 3.0
        move_time = 0.0
        
        while viewer.is_running() and move_time < move_duration:
            pos_ref = smooth_trajectory(move_time, move_duration, 
                                       initial_pose.position, approach_position)
            
            # 获取当前姿态
            q_curr = data.qpos[:6].copy()
            dq_curr = data.qvel[:6].copy()
            curr_pose = controller.robot_kin.forward_kinematics(q_curr)
            
            # 检查姿态偏差
            orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
            
            # 使用初始姿态作为参考
            ref_pose = CartesianState(
                position=pos_ref,
                orientation=initial_pose.orientation.copy(),
                velocity=np.zeros(6)
            )
            
            # 纯位置控制（使用IK求解）
            q_ref, success = controller.robot_kin.inverse_kinematics(ref_pose, q_curr)
            
            if success:
                # 重力补偿
                if ENABLE_GRAVITY_COMPENSATION:
                    tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                    delta_q_compensation = tau_gravity / kp_joints
                    data.ctrl[:6] = q_ref + delta_q_compensation
                else:
                    data.ctrl[:6] = q_ref
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印姿态偏差（每秒一次）
            if int(move_time * 2) % 2 == 0 and int(move_time * 2) != int((move_time - model.opt.timestep) * 2):
                pos_error = np.linalg.norm(curr_pose.position - pos_ref)
                print(f"  时间: {move_time:.1f}s, 位置误差: {pos_error*1000:.2f}mm, 姿态偏差: {orientation_error:.2f}°")
            
            move_time += model.opt.timestep
            time.sleep(model.opt.timestep)
        
        print("✓ 到达接近位置")
        
        # 第二阶段：下降到接近桌面
        print("\n4. 下降接近桌面...")
        contact_duration = 2.0
        contact_time = 0.0
        
        while viewer.is_running() and contact_time < contact_duration:
            pos_ref = smooth_trajectory(contact_time, contact_duration,
                                       approach_position, contact_position)
            
            q_curr = data.qpos[:6].copy()
            dq_curr = data.qvel[:6].copy()
            curr_pose = controller.robot_kin.forward_kinematics(q_curr)
            
            orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
            
            ref_pose = CartesianState(
                position=pos_ref,
                orientation=initial_pose.orientation.copy(),
                velocity=np.zeros(6)
            )
            
            q_ref, success = controller.robot_kin.inverse_kinematics(ref_pose, q_curr)
            
            if success:
                if ENABLE_GRAVITY_COMPENSATION:
                    tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                    delta_q_compensation = tau_gravity / kp_joints
                    data.ctrl[:6] = q_ref + delta_q_compensation
                else:
                    data.ctrl[:6] = q_ref
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            if int(contact_time * 2) % 2 == 0 and int(contact_time * 2) != int((contact_time - model.opt.timestep) * 2):
                pos_error = np.linalg.norm(curr_pose.position - pos_ref)
                print(f"  时间: {contact_time:.1f}s, 位置误差: {pos_error*1000:.2f}mm, 姿态偏差: {orientation_error:.2f}°")
            
            contact_time += model.opt.timestep
            time.sleep(model.opt.timestep)
        
        print("✓ 到达下降位置")
        
        # 第三阶段：XY平面运动
        print("\n5. 开始XY平面运动...")
        print(f"{'时间':>6s} | {'参考X':>8s} {'当前X':>8s} | "
              f"{'参考Y':>8s} {'当前Y':>8s} | "
              f"{'参考Z':>8s} {'当前Z':>8s} | {'姿态偏差':>8s}")
        print("-" * 90)
        
        sim_time = 0.0
        traj_duration = 8.0  # 增加运动时间
        total_duration = 10.0
        step = 0
        
        try:
            while viewer.is_running() and sim_time < total_duration:
                # 生成参考轨迹
                if sim_time < traj_duration:
                    pos_ref = smooth_trajectory(sim_time, traj_duration,
                                               start_position, end_position)
                else:
                    pos_ref = end_position.copy()
                
                q_curr = data.qpos[:6].copy()
                dq_curr = data.qvel[:6].copy()
                curr_pose = controller.robot_kin.forward_kinematics(q_curr)
                
                orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
                
                ref_pose = CartesianState(
                    position=pos_ref,
                    orientation=initial_pose.orientation.copy(),
                    velocity=np.zeros(6)
                )
                
                q_ref, success = controller.robot_kin.inverse_kinematics(ref_pose, q_curr)
                
                if success:
                    if ENABLE_GRAVITY_COMPENSATION:
                        tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                        delta_q_compensation = tau_gravity / kp_joints
                        data.ctrl[:6] = q_ref + delta_q_compensation
                    else:
                        data.ctrl[:6] = q_ref
                
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印
                if step % 250 == 0:
                    print(f"{sim_time:6.2f}s | "
                          f"{pos_ref[0]:8.4f} {curr_pose.position[0]:8.4f} | "
                          f"{pos_ref[1]:8.4f} {curr_pose.position[1]:8.4f} | "
                          f"{pos_ref[2]:8.4f} {curr_pose.position[2]:8.4f} | "
                          f"{orientation_error:7.2f}°")
                
                sim_time += model.opt.timestep
                step += 1
                time.sleep(model.opt.timestep)
        
        except KeyboardInterrupt:
            print("\n测试中断")
        
        # 最终结果
        final_pose = controller.robot_kin.forward_kinematics(data.qpos[:6])
        xy_error = np.linalg.norm(end_position[:2] - final_pose.position[:2])
        z_error = abs(end_position[2] - final_pose.position[2])
        
        print("\n" + "="*60)
        print("最终结果:")
        print(f"  目标XY: [{end_position[0]:.4f}, {end_position[1]:.4f}]")
        print(f"  当前XY: [{final_pose.position[0]:.4f}, {final_pose.position[1]:.4f}]")
        print(f"  XY误差: {xy_error*1000:.2f}mm")
        print(f"  目标Z: {end_position[2]:.4f}m")
        print(f"  当前Z: {final_pose.position[2]:.4f}m")
        print(f"  Z误差: {z_error*1000:.2f}mm")
        print("="*60)


if __name__ == '__main__':
    main()
