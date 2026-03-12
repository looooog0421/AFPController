#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试5: 阻抗控制详细调试
打印详细的中间变量，诊断控制流程
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys
import pinocchio

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
    print("测试5: 阻抗控制详细调试")
    print("="*60)
    
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 加载
    print("\n1. 加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # 初始姿态
    q_init = np.array([-np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, 0.0])
    data.qpos[:6] = q_init
    mujoco.mj_forward(model, data)
    
    # 控制器
    print("\n2. 初始化控制器...")
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
    
    # 阻抗控制参数
    impedance_params = ImpedanceParams.create_uniform(stiffness=300.0, damping=40.0)
    
    # 当前力（从传感器读取，这里模拟为0）
    current_wrench = WrenchData(force=np.zeros(3), torque=np.zeros(3), frame='endeffector')
    
    # 期望力（设置为None表示纯位置控制；设置非零值表示力/位混合控制）
    # 示例：如果要施加10N的Z向下压力，设置为 WrenchData(force=[0, 0, -10], torque=[0,0,0])
    reference_wrench = None  # 纯位置控制模式
    reference_wrench = WrenchData(
                            force=np.array([1.0, 0, 0.0]), 
                            torque=np.zeros(3), 
                            frame='endeffector')  # 力控制示例
    
    print(f"\n阻抗控制模式: {'纯位置控制' if reference_wrench is None else '力/位混合控制'}")
    if reference_wrench is not None:
        print(f"期望力: {reference_wrench.force} N")
        print(f"期望力矩: {reference_wrench.torque} Nm")
    
    # 计算初始位姿
    initial_pose = controller.robot_kin.forward_kinematics(q_init)
    print(f"\n初始末端位置: {initial_pose.position}")
    
    # 定义目标
    target_position = initial_pose.position.copy()
    target_position[2] += 0.10  # z+100mm
    print(f"目标末端位置: {target_position}")
    
    # 测试一步控制
    print("\n" + "="*60)
    print("详细测试：从初始位置到目标位置的一步控制")
    print("="*60)
    
    # 创建目标位姿
    target_pose = initial_pose.copy()
    target_pose.position = target_position
    
    print(f"\n当前关节角度: {np.rad2deg(q_init)}")
    print(f"当前末端位置: {initial_pose.position}")
    print(f"参考末端位置: {target_pose.position}")
    print(f"位置误差: {target_pose.position - initial_pose.position}")
    
    # 执行控制计算
    output = controller.compute_control(
        current_joint_state=q_init,
        current_joint_velocity=np.zeros(6),
        reference_cartesian=target_pose,
        current_wrench=current_wrench,
        impedance_params=impedance_params,
        reference_wrench=reference_wrench  # 添加期望力参数
    )
    
    if output.success:
        print(f"\n✓ 控制成功")
        print(f"\n【调试信息】")
        print(f"F_impedance: {output.debug_info.get('F_impedance', 'N/A')}")
        print(f"F_desired: {output.debug_info.get('F_desired', 'N/A')}")
        print(f"F_measured: {output.debug_info.get('F_measured', 'N/A')}")
        print(f"delta_F: {output.debug_info.get('delta_F', 'N/A')}")
        print(f"delta_x: {output.debug_info.get('delta_x', 'N/A')}")
        print(f"corrected_pos: {output.debug_info.get('corrected_pos', 'N/A')}")
        
        print(f"\n目标关节角度: {np.rad2deg(output.joint_positions)}")
        print(f"关节变化: {np.rad2deg(output.joint_positions - q_init)}")
        
        # 验证IK解
        verify_pose = controller.robot_kin.forward_kinematics(output.joint_positions)
        print(f"\n验证IK解的末端位置: {verify_pose.position}")
        print(f"与目标的位置误差: {verify_pose.position - target_position}")
        print(f"位置误差范数: {np.linalg.norm(verify_pose.position - target_position)*1000:.2f}mm")
        
        # 检查是否真的向上移动了
        z_change = verify_pose.position[2] - initial_pose.position[2]
        print(f"\nZ轴实际变化: {z_change*1000:.2f}mm (目标: 10.00mm)")
        
    else:
        print(f"\n✗ 控制失败: {output.error_msg}")
        return
    
    # 动态仿真测试
    print("\n" + "="*60)
    print("动态仿真：观察10秒内的运动")
    print("="*60)
    
    sim_time = 0.0
    duration = 10.0
    traj_duration = 9.0
    
    def smooth_trajectory(t, t_total, p_start, p_end):
        """5次多项式轨迹"""
        if t >= t_total:
            return p_end
        tau = t / t_total
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        return p_start + s * (p_end - p_start)
    
    print("\n开始仿真...")
    print(f"{'时间':>6s} | {'参考X':>8s} {'当前X':>8s} {'误差':>7s} | "
          f"{'参考Y':>8s} {'当前Y':>8s} {'误差':>7s} | "
          f"{'参考Z':>8s} {'当前Z':>8s} {'误差':>7s} | {'最大力矩':>12s}")
    print("-" * 123)
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            step = 0
            
            while viewer.is_running() and sim_time < duration:
                # 生成参考位置
                ref_pose = initial_pose.copy()
                if sim_time < traj_duration:
                    ref_pose.position = smooth_trajectory(
                        sim_time, traj_duration,
                        initial_pose.position, target_position
                    )
                else:
                    ref_pose.position = target_position.copy()
                
                # 获取当前状态
                q_curr = data.qpos[:6].copy()
                dq_curr = data.qvel[:6].copy()
                
                # 计算当前位姿
                curr_pose = controller.robot_kin.forward_kinematics(q_curr)
                
                # 控制计算
                output = controller.compute_control(
                    current_joint_state=q_curr,
                    current_joint_velocity=dq_curr,
                    reference_cartesian=ref_pose,
                    current_wrench=current_wrench,
                    impedance_params=impedance_params,
                    reference_wrench=reference_wrench  # 添加期望力参数
                )
                
                if output.success:
                    q_ref_impedance = output.joint_positions
                    
                    # 重力补偿：计算重力力矩并转换为等效位置偏移
                    if ENABLE_GRAVITY_COMPENSATION:
                        tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                        delta_q_compensation = tau_gravity / kp_joints
                        q_ref_compensated = q_ref_impedance + delta_q_compensation
                        data.ctrl[:6] = q_ref_compensated
                    else:
                        data.ctrl[:6] = q_ref_impedance
                
                # 仿真步进
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印详细信息
                if step % 250 == 0:
                    pos_error = ref_pose.position - curr_pose.position
                    joint_torques = data.actuator_force[:6]
                    max_torque = np.max(np.abs(joint_torques))
                    print(f"{sim_time:6.2f}s | "
                          f"{ref_pose.position[0]:8.4f} {curr_pose.position[0]:8.4f} {pos_error[0]*1000:6.1f}mm | "
                          f"{ref_pose.position[1]:8.4f} {curr_pose.position[1]:8.4f} {pos_error[1]*1000:6.1f}mm | "
                          f"{ref_pose.position[2]:8.4f} {curr_pose.position[2]:8.4f} {pos_error[2]*1000:6.1f}mm | "
                          f"τ_max={max_torque:6.1f}Nm")
                
                sim_time += model.opt.timestep
                step += 1
                time.sleep(model.opt.timestep)
        
        # 最终验证
        final_pose = controller.robot_kin.forward_kinematics(data.qpos[:6])
        final_error = np.linalg.norm(final_pose.position - target_position)
        z_error = target_position[2] - final_pose.position[2]
        
        print("\n" + "="*60)
        print("最终结果:")
        print(f"  目标Z: {target_position[2]:.4f}m")
        print(f"  当前Z: {final_pose.position[2]:.4f}m")
        print(f"  Z误差: {z_error*1000:.2f}mm")
        print(f"  总位置误差: {final_error*1000:.2f}mm")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n测试中断")

if __name__ == '__main__':
    main()
