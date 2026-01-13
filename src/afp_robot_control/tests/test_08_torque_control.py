#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试8: 力矩控制 + 逆动力学前馈
解决稳态误差：使用计算力矩控制替代位置PD
"""
import sys
import os
import numpy as np
import mujoco
import mujoco.viewer
import time

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))
from impedance_control import CartesianState
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper
import pinocchio as pin


class CartesianTrajectoryPlanner:
    """笛卡尔空间轨迹规划器（直接规划笛卡尔位置）"""
    
    def __init__(self, pos_start, pos_end, duration):
        """
        Args:
            pos_start: 起始笛卡尔位置 (3,)
            pos_end: 目标笛卡尔位置 (3,)
            duration: 轨迹总时间 (秒)
        """
        self.pos_start = pos_start.copy()
        self.pos_end = pos_end.copy()
        self.duration = duration
        self.pos_diff = pos_end - pos_start
        
        # 梯形速度规划参数
        self.t_acc = duration * 0.3
        self.t_dec = duration * 0.3
        self.t_const = duration - self.t_acc - self.t_dec
        
    def get_position(self, t):
        """获取t时刻的笛卡尔位置"""
        if t <= 0:
            return self.pos_start.copy()
        if t >= self.duration:
            return self.pos_end.copy()
        
        # 梯形速度轮廓
        if t < self.t_acc:
            s = 0.5 * (t / self.t_acc) ** 2
        elif t < self.t_acc + self.t_const:
            s = 0.5 + (t - self.t_acc) / (self.t_acc + self.t_const)
        else:
            t_rel = t - self.t_acc - self.t_const
            s = 1.0 - 0.5 * (1 - t_rel / self.t_dec) ** 2
        
        return self.pos_start + s * self.pos_diff
    
    def get_velocity(self, t):
        """获取t时刻的笛卡尔速度"""
        if t <= 0 or t >= self.duration:
            return np.zeros(3)
        
        if t < self.t_acc:
            s_dot = (t / self.t_acc) / self.t_acc
        elif t < self.t_acc + self.t_const:
            s_dot = 1.0 / (self.t_acc + self.t_const)
        else:
            t_rel = t - self.t_acc - self.t_const
            s_dot = (1 - t_rel / self.t_dec) / self.t_dec
        
        return s_dot * self.pos_diff


def computed_torque_control(model_pin, data_pin, q_ref, qd_ref, qdd_ref, q_curr, qd_curr, Kp, Kd):
    """
    计算力矩控制（逆动力学 + PD反馈）
    
    τ = M(q)·(qdd_ref + Kp·e + Kd·ė) + C(q,qd) + g(q)
    
    Args:
        model_pin: Pinocchio模型
        data_pin: Pinocchio数据
        q_ref, qd_ref, qdd_ref: 参考关节位置、速度、加速度
        q_curr, qd_curr: 当前关节位置、速度
        Kp, Kd: PD增益对角矩阵
    
    Returns:
        tau: 控制力矩 (n,)
    """
    # 位置和速度误差
    e = q_ref - q_curr
    e_dot = qd_ref - qd_curr
    
    # PD反馈加速度
    qdd_fb = Kp @ e + Kd @ e_dot
    
    # 期望加速度
    qdd_desired = qdd_ref + qdd_fb
    
    # 逆动力学：计算所需力矩
    # M(q)·qdd + C(q,qd)·qd + g(q) = τ
    tau = pin.rnea(model_pin, data_pin, q_curr, qd_curr, qdd_desired)
    
    return tau


def main():
    print("\n" + "="*60)
    print("测试8: 力矩控制 + 逆动力学前馈")
    print("="*60)
    
    # 加载模型
    print("\n1. 加载模型...")
    xml_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 初始化Pinocchio
    print("\n2. 初始化Pinocchio...")
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=False)
    kin.ik_max_iter = 500
    kin.ik_eps = 1e-3
    kin.ik_damping = 1e-4
    
    # Pinocchio模型用于逆动力学
    model_pin = kin.model
    data_pin = kin.data
    
    # 初始关节角度
    q_init = np.array([-np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, 0.0])
    data.qpos[:6] = q_init
    data.qvel[:6] = 0.0
    mujoco.mj_forward(model, data)
    
    # 计算目标
    initial_pose = kin.forward_kinematics(q_init)
    target_position = initial_pose.position.copy()
    target_position[2] += 0.10  # z+100mm
    
    print(f"\n初始位置: {initial_pose.position}")
    print(f"目标位置: {target_position}")
    
    # 创建笛卡尔空间轨迹规划器
    traj_duration = 4.0
    planner = CartesianTrajectoryPlanner(initial_pose.position, target_position, traj_duration)
    
    # 控制增益（相对于位置PD，这里的增益可以低一些）
    Kp = np.diag([200.0, 200.0, 200.0, 20.0, 20.0, 20.0])
    Kd = np.diag([40.0, 40.0, 40.0, 4.0, 4.0, 4.0])
    
    # 检查MuJoCo是否支持力矩控制
    print(f"\n执行器数量: {model.nu}")
    print(f"执行器类型: {[model.actuator_dyntype[i] for i in range(model.nu)]}")
    
    # 仿真
    print("\n3. 启动仿真（力矩控制模式）...")
    print(f"{'时间':>6s} | {'参考X':>8s} {'当前X':>8s} {'误差':>7s} | "
          f"{'参考Y':>8s} {'当前Y':>8s} {'误差':>7s} | "
          f"{'参考Z':>8s} {'当前Z':>8s} {'误差':>7s}")
    print("-" * 110)
    
    sim_time = 0.0
    duration = 5.0
    dt = model.opt.timestep
    
    # 数值微分滤波器
    q_ref_prev = q_init.copy()
    qd_ref_prev = np.zeros(6)
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            step = 0
            
            while viewer.is_running() and sim_time < duration:
                # 获取参考笛卡尔位置和速度
                if sim_time < traj_duration:
                    pos_ref = planner.get_position(sim_time)
                    vel_ref_cart = planner.get_velocity(sim_time)
                else:
                    pos_ref = target_position.copy()
                    vel_ref_cart = np.zeros(3)
                
                # IK求解参考关节角度
                ref_pose = CartesianState(
                    position=pos_ref,
                    orientation=initial_pose.orientation.copy(),
                    velocity=np.zeros(6)
                )
                q_ref, _ = kin.inverse_kinematics(ref_pose, data.qpos[:6])
                
                # 数值微分估计关节速度和加速度参考
                qd_ref = (q_ref - q_ref_prev) / dt
                qdd_ref = (qd_ref - qd_ref_prev) / dt
                
                # 获取当前状态
                q_curr = data.qpos[:6].copy()
                qd_curr = data.qvel[:6].copy()
                
                # 计算控制力矩
                tau = computed_torque_control(
                    model_pin, data_pin,
                    q_ref, qd_ref, qdd_ref,
                    q_curr, qd_curr,
                    Kp, Kd
                )
                
                # 设置力矩指令（直接设置data.ctrl，MuJoCo会根据actuator类型处理）
                # 注意：如果actuator是position类型，需要先转换或修改XML
                # 这里假设可以直接设置力矩
                data.ctrl[:6] = tau
                
                # 仿真
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印
                if step % 250 == 0:
                    curr_cart = kin.forward_kinematics(q_curr)
                    pos_error = pos_ref - curr_cart.position
                    
                    print(f"{sim_time:6.2f}s | "
                          f"{pos_ref[0]:8.4f} {curr_cart.position[0]:8.4f} {pos_error[0]*1000:6.1f}mm | "
                          f"{pos_ref[1]:8.4f} {curr_cart.position[1]:8.4f} {pos_error[1]*1000:6.1f}mm | "
                          f"{pos_ref[2]:8.4f} {curr_cart.position[2]:8.4f} {pos_error[2]*1000:6.1f}mm")
                
                # 更新历史
                q_ref_prev = q_ref.copy()
                qd_ref_prev = qd_ref.copy()
                
                sim_time += dt
                step += 1
                time.sleep(dt)
        
        # 最终结果
        final_pose = kin.forward_kinematics(data.qpos[:6])
        pos_error = target_position - final_pose.position
        total_error = np.linalg.norm(pos_error)
        
        print("\n" + "="*60)
        print("最终结果:")
        print(f"  位置误差: X={pos_error[0]*1000:.2f}mm, Y={pos_error[1]*1000:.2f}mm, Z={pos_error[2]*1000:.2f}mm")
        print(f"  总位置误差: {total_error*1000:.2f}mm")
        
        if total_error < 0.002:  # 2mm误差
            print("\n✓ 力矩控制效果优秀，稳态误差显著减小")
        elif total_error < 0.005:
            print("\n✓ 力矩控制效果良好")
        else:
            print("\n⚠️  误差仍较大")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n测试中断")


if __name__ == '__main__':
    main()
