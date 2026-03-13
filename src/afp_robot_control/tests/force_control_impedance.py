#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试9: 阻抗控制 - 接触力控制
机器人末端接触平面，沿XY方向移动，Z轴保持恒定接触力
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
    WrenchData,
    ImpedanceParams,
    StandardStrategy,
    CartesianImpedanceController
)


def main():
    print("\n" + "="*60)
    print("测试9: 阻抗控制 - 接触力控制")
    print("="*60)
    
    # 加载模型
    print("\n1. 加载模型...")
    xml_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene_contact.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 在场景中添加接触平面（通过修改data）
    # 或者我们先检查是否已有地面
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
    
    # 获取力传感器ID
    force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'ee_force_sensor')
    torque_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, 'ee_torque_sensor')
    print(f"\n力传感器ID: force={force_sensor_id}, torque={torque_sensor_id}")
    
    # 获取Pinocchio model和data（用于重力补偿）
    model_pin = controller.robot_kin.model
    data_pin = controller.robot_kin.data
    
    # 重力补偿参数
    ENABLE_GRAVITY_COMPENSATION = True
    kp_joints = np.array([5000, 5000, 5000, 1000, 1000, 1000])
    print(f"\n重力补偿: {'启用' if ENABLE_GRAVITY_COMPENSATION else '禁用'}")
    
    # 阻抗控制参数
    # Z轴使用较低刚度（柔顺接触），XY轴较高刚度（精确跟踪）
    # 姿态刚度提高到50，确保碰撞时能保持姿态
    impedance_params = ImpedanceParams(
        stiffness=np.array([500.0, 500.0, 50.0, 50.0, 50.0, 50.0]),  # 位置[Kx,Ky,Kz], 姿态[Krx,Kry,Krz]
        damping=np.array([50.0, 50.0, 15.0, 10.0, 10.0, 10.0])        # 对应的阻尼
    )
    
    # 初始姿态：使末端高度降低，接近桌面
    # 调整shoulder_lift和elbow使机器人更低
    q_init = np.array([-np.pi/4, -np.pi/2.2, np.pi/2.5, -np.pi/2, -np.pi/2, 0.0])
    data.qpos[:6] = q_init
    data.ctrl[:6] = q_init
    mujoco.mj_forward(model, data)
    
    # 计算初始位姿
    initial_pose = controller.robot_kin.forward_kinematics(q_init)
    print(f"\n初始末端位置: {initial_pose.position}")
    print(f"初始末端Z高度: {initial_pose.position[2]:.4f}m")
    
    # 设置接触平面高度（与scene_contact.xml中桌面高度一致）
    # 桌面box: pos=(0.6, -0.6, 0.0), size=(0.4, 0.4, 0.4)
    # 桌面顶部高度 = 0.0 + 0.4 = 0.4m
    contact_height = 0.40  # 桌面顶部在Z=0.40m（400mm）
    print(f"接触平面高度: {contact_height}m")
    
    # 定义轨迹：先垂直下降接触，再沿45度方向移动
    # 阶段1：移动到桌面上方起始位置（高于桌面15mm）
    approach_position = initial_pose.position.copy()
    approach_position[2] = contact_height + 0.01  # 高于桌面15mm
    
    # 阶段2：缓慢下降至接触高度（桌面上方2mm，产生接触力）
    contact_position = approach_position.copy()
    contact_position[2] = contact_height + 0.002  # 接触点：高于桌面只有2mm
    
    # 阶段3：XY平面运动45度方向
    start_position = contact_position.copy()
    end_position = start_position.copy()
    end_position[0] += 0.05  # X方向+50mm
    end_position[1] += 0.05  # Y方向+50mm（45度）
    end_position[2] = contact_height + 0.002  # Z保持接触
    
    print(f"\n轨迹阶段1 (接近): {approach_position}")
    print(f"轨迹阶段2 (接触): {contact_position}")
    print(f"轨迹阶段3 (起点): {start_position}")
    print(f"轨迹阶段3 (终点): {end_position}")
    
    # 目标接触力：Z轴向下10N（将在第二阶段线性增加）
    target_force_z = -10.0  # 目标Z轴力
    print(f"\n目标接触力: [0.0, 0.0, {target_force_z}] N")
    print("注意：参考力将在第二阶段线性增加以实现平滑过渡")
    
    # 先将机器人移动到接近位置
    print("\n3. 移动到接近位置...")
    move_duration = 2.0
    move_time = 0.0
    
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
        # 第一阶段：移动到接近位置（纯位置控制）
        while viewer.is_running() and move_time < move_duration:
            pos_ref = smooth_trajectory(move_time, move_duration, 
                                       initial_pose.position, approach_position)
            
            # 获取当前姿态
            q_curr = data.qpos[:6].copy()
            dq_curr = data.qvel[:6].copy()
            curr_pose = controller.robot_kin.forward_kinematics(q_curr)
            
            # 检查姿态偏差
            orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
            
            # 始终使用初始姿态作为参考，保持姿态不变
            ref_pose = CartesianState(
                position=pos_ref,
                orientation=initial_pose.orientation.copy(),
                velocity=np.zeros(6)
            )
            
            # 读取真实力传感器数据
            force_reading = data.sensordata[force_sensor_id:force_sensor_id+3].copy()
            torque_reading = data.sensordata[torque_sensor_id:torque_sensor_id+3].copy()
            current_wrench = WrenchData(
                force=force_reading, 
                torque=torque_reading, 
                frame='endeffector'
            )
            
            # 阻抗控制（无参考力，纯位置模式）
            output = controller.compute_control(
                current_joint_state=q_curr,
                current_joint_velocity=dq_curr,
                reference_cartesian=ref_pose,
                current_wrench=current_wrench,
                impedance_params=impedance_params,
                reference_wrench=None  # 无参考力，纯位置跟踪
            )
            
            if output.success:
                q_ref = output.joint_positions
                # 重力补偿
                if ENABLE_GRAVITY_COMPENSATION:
                    tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                    delta_q_compensation = tau_gravity / kp_joints
                    data.ctrl[:6] = q_ref + delta_q_compensation
                else:
                    data.ctrl[:6] = q_ref
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印姿态偏差和力传感器数据（每秒一次）
            if int(move_time * 10) % 10 == 0:
                print(f"  时间: {move_time:.1f}s, 姿态偏差: {orientation_error:.2f}°, "
                      f"力: [{force_reading[0]:6.2f}, {force_reading[1]:6.2f}, {force_reading[2]:6.2f}]N")
            
            move_time += model.opt.timestep
            time.sleep(model.opt.timestep)
        
        print("✓ 到达接近位置")
        
        # 第二阶段：缓慢下降接触平面（平滑启用力控制）
        print("\n4. 缓慢下降接触平面（平滑力过渡）...")
        contact_duration = 2.0
        contact_time = 0.0
        
        # 接触力阈值：检测Z轴力超过该值认为已接触
        contact_force_threshold = -1.0  # -1N（向上的反作用力）
        contact_detected = False
        force_transition_start_time = None
        force_transition_duration = 1.0  # 力过渡时间1秒
        
        while viewer.is_running() and contact_time < contact_duration:
            pos_ref = smooth_trajectory(contact_time, contact_duration,
                                       approach_position, contact_position)
            
            # 获取当前姿态
            q_curr = data.qpos[:6].copy()
            dq_curr = data.qvel[:6].copy()
            curr_pose = controller.robot_kin.forward_kinematics(q_curr)
            
            # 检查姿态偏差
            orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
            
            # 始终使用初始姿态作为参考
            ref_pose = CartesianState(
                position=pos_ref,
                orientation=initial_pose.orientation.copy(),
                velocity=np.zeros(6)
            )
            
            # 读取真实力传感器数据
            force_reading = data.sensordata[force_sensor_id:force_sensor_id+3].copy()
            torque_reading = data.sensordata[torque_sensor_id:torque_sensor_id+3].copy()
            current_wrench = WrenchData(
                force=force_reading,
                torque=torque_reading,
                frame='endeffector'
            )
            
            # 接触检测：当Z轴力超过阈值时开始力过渡
            if not contact_detected and force_reading[2] < contact_force_threshold:
                contact_detected = True
                force_transition_start_time = contact_time
                print(f"  ✓ 检测到接触！力={force_reading[2]:.2f}N, 开始平滑力过渡...")
            
            # 计算当前参考力（平滑过渡）
            if contact_detected and force_transition_start_time is not None:
                # 从接触时刻开始，线性增加参考力
                elapsed_time = contact_time - force_transition_start_time
                force_ratio = min(elapsed_time / force_transition_duration, 1.0)
                current_ref_force_z = target_force_z * force_ratio
            else:
                # 未接触或刚开始，参考力为0
                current_ref_force_z = 0.0
            
            # 创建当前参考力
            reference_wrench = WrenchData(
                force=np.array([0.0, 0.0, current_ref_force_z]),
                torque=np.zeros(3),
                frame='endeffector'
            )
            
            # 阻抗控制（Z轴力控制）
            output = controller.compute_control(
                current_joint_state=q_curr,
                current_joint_velocity=dq_curr,
                reference_cartesian=ref_pose,
                current_wrench=current_wrench,
                impedance_params=impedance_params,
                reference_wrench=reference_wrench  # 平滑增加的参考力
            )
            
            if output.success:
                q_ref = output.joint_positions
                if ENABLE_GRAVITY_COMPENSATION:
                    tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                    delta_q_compensation = tau_gravity / kp_joints
                    data.ctrl[:6] = q_ref + delta_q_compensation
                else:
                    data.ctrl[:6] = q_ref
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # 打印姿态偏差和力传感器数据
            if int(contact_time * 10) % 10 == 0:
                print(f"  时间: {contact_time:.1f}s, 姿态偏差: {orientation_error:.2f}°, "
                      f"力: [{force_reading[0]:6.2f}, {force_reading[1]:6.2f}, {force_reading[2]:6.2f}]N, "
                      f"参考力Z: {current_ref_force_z:6.2f}N")
            
            contact_time += model.opt.timestep
            time.sleep(model.opt.timestep)
        
        print("✓ 接触平面")
        
        # 第三阶段：接触力控制下的XY运动
        print("\n5. 开始XY平面运动（保持接触力）...")
        
        # 第三阶段使用固定的目标参考力
        reference_wrench = WrenchData(
            force=np.array([0.0, 0.0, target_force_z]),
            torque=np.zeros(3),
            frame='endeffector'
        )
        
        print(f"{'时间':>6s} | {'参考X':>8s} {'当前X':>8s} | "
              f"{'参考Y':>8s} {'当前Y':>8s} | "
              f"{'参考Z':>8s} {'当前Z':>8s} | "
              f"{'力X':>7s} {'力Y':>7s} {'力Z':>7s} | {'姿态偏差':>8s}")
        print("-" * 120)
        
        sim_time = 0.0
        traj_duration = 5.0
        total_duration = 7.0
        step = 0
        
        try:
            while viewer.is_running() and sim_time < total_duration:
                # 生成参考轨迹（XY移动，Z保持）
                if sim_time < traj_duration:
                    pos_ref = smooth_trajectory(sim_time, traj_duration,
                                               start_position, end_position)
                else:
                    pos_ref = end_position.copy()
                
                # 获取当前状态
                q_curr = data.qpos[:6].copy()
                dq_curr = data.qvel[:6].copy()
                curr_pose = controller.robot_kin.forward_kinematics(q_curr)
                
                # 检查姿态偏差
                orientation_error = quaternion_distance(curr_pose.orientation, initial_pose.orientation)
                
                # 始终使用初始姿态作为参考
                ref_pose = CartesianState(
                    position=pos_ref,
                    orientation=initial_pose.orientation.copy(),
                    velocity=np.zeros(6)
                )
                
                # 读取真实力传感器数据
                force_reading = data.sensordata[force_sensor_id:force_sensor_id+3].copy()
                torque_reading = data.sensordata[torque_sensor_id:torque_sensor_id+3].copy()
                current_wrench = WrenchData(
                    force=force_reading,
                    torque=torque_reading,
                    frame='endeffector'
                )
                
                # 阻抗控制计算
                output = controller.compute_control(
                    current_joint_state=q_curr,
                    current_joint_velocity=dq_curr,
                    reference_cartesian=ref_pose,
                    current_wrench=current_wrench,
                    impedance_params=impedance_params,
                    reference_wrench=reference_wrench  # Z轴力控制
                )
                
                if output.success:
                    q_ref_impedance = output.joint_positions
                    
                    # 重力补偿
                    if ENABLE_GRAVITY_COMPENSATION:
                        tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                        delta_q_compensation = tau_gravity / kp_joints
                        data.ctrl[:6] = q_ref_impedance + delta_q_compensation
                    else:
                        data.ctrl[:6] = q_ref_impedance
                
                # 仿真
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印
                if step % 250 == 0:
                    print(f"{sim_time:6.2f}s | "
                          f"{pos_ref[0]:8.4f} {curr_pose.position[0]:8.4f} | "
                          f"{pos_ref[1]:8.4f} {curr_pose.position[1]:8.4f} | "
                          f"{pos_ref[2]:8.4f} {curr_pose.position[2]:8.4f} | "
                          f"{current_wrench.force[0]:6.1f}N {current_wrench.force[1]:6.1f}N {current_wrench.force[2]:6.1f}N | "
                          f"{orientation_error:7.2f}°")
                
                sim_time += model.opt.timestep
                step += 1
                time.sleep(model.opt.timestep)
        
        except KeyboardInterrupt:
            print("\n测试中断")
        
        # 最终结果
        final_pose = controller.robot_kin.forward_kinematics(data.qpos[:6])
        xy_error = np.linalg.norm(end_position[:2] - final_pose.position[:2])
        z_error = end_position[2] - final_pose.position[2]
        
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
