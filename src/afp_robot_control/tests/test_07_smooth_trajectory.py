#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试7: 关节空间轨迹平滑
解决超调问题：在关节空间生成平滑轨迹
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
from impedance_control import CartesianState
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper


class JointTrajectoryPlanner:
    """关节空间轨迹规划器（梯形速度曲线）"""
    
    def __init__(self, q_start, q_end, duration, max_vel_factor=0.5, max_acc_factor=1.0):
        """
        Args:
            q_start: 起始关节角度 (n,)
            q_end: 目标关节角度 (n,)
            duration: 轨迹总时间 (秒)
            max_vel_factor: 最大速度因子 (0-1)
            max_acc_factor: 最大加速度因子 (0-1)
        """
        self.q_start = q_start.copy()
        self.q_end = q_end.copy()
        self.duration = duration
        
        # 计算路程
        self.q_diff = q_end - q_start
        self.distance = np.linalg.norm(self.q_diff)
        
        # 梯形速度规划参数
        # 加速段、匀速段、减速段的时间分配
        self.t_acc = duration * 0.3  # 加速30%
        self.t_dec = duration * 0.3  # 减速30%
        self.t_const = duration - self.t_acc - self.t_dec  # 匀速40%
        
        # 计算最大速度（关节空间）
        self.v_max = self.q_diff / (self.t_acc/2 + self.t_const + self.t_dec/2)
        
    def get_position(self, t):
        """获取t时刻的关节位置"""
        if t <= 0:
            return self.q_start.copy()
        if t >= self.duration:
            return self.q_end.copy()
        
        # 梯形速度轮廓
        if t < self.t_acc:
            # 加速段：s = 0.5 * a * t^2
            s = 0.5 * (t / self.t_acc) ** 2
        elif t < self.t_acc + self.t_const:
            # 匀速段：s = 0.5 + v * (t - t_acc)
            s = 0.5 + (t - self.t_acc) / (self.t_acc + self.t_const)
        else:
            # 减速段
            t_rel = t - self.t_acc - self.t_const
            s = 1.0 - 0.5 * (1 - t_rel / self.t_dec) ** 2
        
        return self.q_start + s * self.q_diff
    
    def get_velocity(self, t):
        """获取t时刻的关节速度"""
        if t <= 0 or t >= self.duration:
            return np.zeros_like(self.q_start)
        
        if t < self.t_acc:
            # 加速段
            s_dot = (t / self.t_acc) / self.t_acc
        elif t < self.t_acc + self.t_const:
            # 匀速段
            s_dot = 1.0 / (self.t_acc + self.t_const)
        else:
            # 减速段
            t_rel = t - self.t_acc - self.t_const
            s_dot = (1 - t_rel / self.t_dec) / self.t_dec
        
        return s_dot * self.q_diff


def main():
    print("\n" + "="*60)
    print("测试7: 关节空间轨迹平滑")
    print("="*60)
    
    # 加载模型
    print("\n1. 加载模型...")
    xml_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 初始化
    print("\n2. 初始化Pinocchio...")
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=False)
    # 调整IK参数以提高收敛性
    kin.ik_max_iter = 500  # 增加最大迭代次数
    kin.ik_eps = 1e-3      # 放宽收敛阈值（从1e-4到1e-3）
    kin.ik_damping = 1e-4  # 增大阻尼提高数值稳定性
    
    # 获取Pinocchio的model和data（用于重力补偿）
    model_pin = kin.model
    data_pin = kin.data
    
    # 初始关节角度
    q_init = np.array([-np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, 0.0])
    data.qpos[:6] = q_init
    data.ctrl[:6] = q_init
    mujoco.mj_forward(model, data)
    
    # 计算目标
    initial_pose = kin.forward_kinematics(q_init)
    target_position = initial_pose.position.copy()
    target_position[2] += 0.10  # z+100mm (增大位移便于可视化)
    
    target_pose = CartesianState(
        position=target_position,
        orientation=initial_pose.orientation.copy(),
        velocity=np.zeros(6)
    )
    
    q_target, success = kin.inverse_kinematics(target_pose, q_init)
    if not success:
        print("警告: IK未收敛")
    
    print(f"\n初始位置: {initial_pose.position}")
    print(f"目标位置: {target_position}")
    print(f"初始关节: {np.rad2deg(q_init)}")
    print(f"目标关节: {np.rad2deg(q_target)}")
    
    # 重力补偿参数
    ENABLE_GRAVITY_COMPENSATION = True  # 开关：是否启用重力补偿
    # 获取各关节的kp值（从MuJoCo actuator配置中读取）
    kp_joints = np.array([5000, 5000, 5000, 1000, 1000, 1000])  # size3关节和size1关节的kp
    print(f"\n重力补偿: {'启用' if ENABLE_GRAVITY_COMPENSATION else '禁用'}")
    if ENABLE_GRAVITY_COMPENSATION:
        print(f"各关节kp: {kp_joints}")
    
    # 创建笛卡尔空间轨迹规划器（使用五次多项式）
    class CartesianTrajectoryPlanner:
        """笛卡尔空间五次多项式轨迹（保证边界速度和加速度为0）"""
        def __init__(self, pos_start, pos_end, duration):
            self.pos_start = pos_start.copy()
            self.pos_end = pos_end.copy()
            self.duration = duration
            self.pos_diff = pos_end - pos_start
        
        def get_position(self, t):
            if t <= 0:
                return self.pos_start.copy()
            if t >= self.duration:
                return self.pos_end.copy()
            
            # 五次多项式：s(τ) = 10τ³ - 15τ⁴ + 6τ⁵，其中τ = t/T ∈ [0,1]
            tau = t / self.duration
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            
            return self.pos_start + s * self.pos_diff
    
    traj_duration = 9.0
    planner_cart = CartesianTrajectoryPlanner(initial_pose.position, target_position, traj_duration)
    
    # 仿真
    print("\n3. 启动仿真（关节空间平滑轨迹）...")
    print(f"{'时间':>6s} | {'参考X':>8s} {'当前X':>8s} {'误差':>7s} | "
          f"{'参考Y':>8s} {'当前Y':>8s} {'误差':>7s} | "
          f"{'参考Z':>8s} {'当前Z':>8s} {'误差':>7s} | {'最大力矩':>12s}")
    print("-" * 123)
    
    sim_time = 0.0
    duration = 10.0  # 总仿真时间也延长
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            step = 0
            
            while viewer.is_running() and sim_time < duration:
                # 从笛卡尔轨迹规划器获取参考位置
                if sim_time < traj_duration:
                    pos_ref = planner_cart.get_position(sim_time)
                else:
                    pos_ref = target_position.copy()
                
                # 实时IK求解参考关节角度（每步重新计算，保证与笛卡尔轨迹一致）
                ref_pose = CartesianState(
                    position=pos_ref,
                    orientation=initial_pose.orientation.copy(),
                    velocity=np.zeros(6)
                )
                q_ref, _ = kin.inverse_kinematics(ref_pose, data.qpos[:6])
                
                # 获取当前状态
                q_curr = data.qpos[:6].copy()
                
                # 重力补偿：计算重力力矩并转换为等效位置偏移
                tau_gravity_debug = None
                delta_q_debug = None
                q_ref_comp_debug = None
                if ENABLE_GRAVITY_COMPENSATION:
                    # 计算重力力矩（仅重力项，不含科氏力和惯性力）
                    # Pinocchio的computeGeneralizedGravity直接接受q向量
                    tau_gravity = pinocchio.computeGeneralizedGravity(model_pin, data_pin, q_curr)
                    # 转换为等效位置偏移：Δq = τ_gravity / kp
                    # 这样 τ_output = kp × (q_ref + Δq - q_curr) ≈ kp × (q_ref - q_curr) + τ_gravity
                    delta_q_compensation = tau_gravity / kp_joints
                    q_ref_compensated = q_ref + delta_q_compensation
                    data.ctrl[:6] = q_ref_compensated
                    # 保存调试信息
                    tau_gravity_debug = tau_gravity.copy()
                    delta_q_debug = delta_q_compensation.copy()
                    q_ref_comp_debug = q_ref_compensated.copy()
                else:
                    data.ctrl[:6] = q_ref
                
                # 仿真
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印
                if step % 250 == 0:
                    # 使用真实的笛卡尔参考，不再通过FK反算
                    curr_cart = kin.forward_kinematics(q_curr)
                    pos_error = pos_ref - curr_cart.position
                    # 获取关节力矩（MuJoCo自动计算的执行器输出）
                    joint_torques = data.actuator_force[:6]  # 前6个关节的实际力矩
                    max_torque = np.max(np.abs(joint_torques))
                    
                    print(f"{sim_time:6.2f}s | "
                          f"{pos_ref[0]:8.4f} {curr_cart.position[0]:8.4f} {pos_error[0]*1000:6.1f}mm | "
                          f"{pos_ref[1]:8.4f} {curr_cart.position[1]:8.4f} {pos_error[1]*1000:6.1f}mm | "
                          f"{pos_ref[2]:8.4f} {curr_cart.position[2]:8.4f} {pos_error[2]*1000:6.1f}mm | "
                          f"τ_max={max_torque:6.1f}Nm")
                    
                    # 打印重力补偿详情（首次和每5秒）
                    if ENABLE_GRAVITY_COMPENSATION and tau_gravity_debug is not None and (step == 0 or step % 1250 == 0):
                        print(f"  [重力补偿] τ_grav={tau_gravity_debug} Nm")
                        print(f"  [重力补偿] Δq_comp={np.rad2deg(delta_q_debug)}°")
                        print(f"  [重力补偿] q_ref原始={np.rad2deg(q_ref)}°")
                        print(f"  [重力补偿] q_ref补偿后={np.rad2deg(q_ref_comp_debug)}°")
                
                sim_time += model.opt.timestep
                step += 1
                time.sleep(model.opt.timestep)
        
        # 最终结果
        final_pose = kin.forward_kinematics(data.qpos[:6])
        pos_error = target_position - final_pose.position
        total_error = np.linalg.norm(pos_error)
        
        # 计算姿态误差（旋转向量）
        from scipy.spatial.transform import Rotation as R
        ref_quat_xyzw = np.array([initial_pose.orientation[1], initial_pose.orientation[2], 
                                  initial_pose.orientation[3], initial_pose.orientation[0]])  # [qw,qx,qy,qz] -> [qx,qy,qz,qw]
        curr_quat_xyzw = np.array([final_pose.orientation[1], final_pose.orientation[2],
                                   final_pose.orientation[3], final_pose.orientation[0]])
        
        ref_rot = R.from_quat(ref_quat_xyzw)
        curr_rot = R.from_quat(curr_quat_xyzw)
        ori_error_rotvec = (ref_rot * curr_rot.inv()).as_rotvec()
        
        print("\n" + "="*60)
        print("最终结果:")
        print(f"  位置误差: X={pos_error[0]*1000:.2f}mm, Y={pos_error[1]*1000:.2f}mm, Z={pos_error[2]*1000:.2f}mm")
        print(f"  总位置误差: {total_error*1000:.2f}mm")
        print(f"  姿态误差(旋转向量): [{np.rad2deg(ori_error_rotvec[0]):.3f}°, "
              f"{np.rad2deg(ori_error_rotvec[1]):.3f}°, {np.rad2deg(ori_error_rotvec[2]):.3f}°]")
        
        if total_error < 0.005:  # 5mm误差可接受
            print("\n✓ 关节空间轨迹平滑，跟踪精度良好")
        else:
            print("\n⚠️  跟踪误差较大，需要进一步优化")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n测试中断")


if __name__ == '__main__':
    main()
