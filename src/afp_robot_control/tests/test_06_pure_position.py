#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试6: 纯位置控制（无阻抗控制）
验证超调是否来自MuJoCo动力学而非阻抗控制器
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper

def main():
    print("\n" + "="*60)
    print("测试6: 纯位置控制（无阻抗控制）")
    print("="*60)
    
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 加载
    print("\n1. 加载模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # 初始化
    q_init = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])
    data.qpos[:6] = q_init
    mujoco.mj_forward(model, data)
    
    # Pinocchio IK
    print("\n2. 初始化Pinocchio...")
    kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=False)
    kin.set_ik_parameters(eps=1e-3, max_iter=200, damping=1e-4, position_only=True)
    
    # 计算轨迹
    initial_pose = kin.forward_kinematics(q_init)
    target_position = initial_pose.position.copy()
    target_position[2] += 0.01  # z+10mm
    
    print(f"\n初始位置: {initial_pose.position}")
    print(f"目标位置: {target_position}")
    
    # 平滑轨迹
    def smooth_trajectory(t, t_total, p_start, p_end):
        if t >= t_total:
            return p_end
        s = t / t_total
        s_smooth = 10*s**3 - 15*s**4 + 6*s**5
        return p_start + s_smooth * (p_end - p_start)
    
    # 仿真
    print("\n3. 启动仿真（纯IK位置控制，无阻抗控制）...")
    print(f"{'时间':>6s} | {'参考Z':>8s} | {'当前Z':>8s} | {'误差':>8s} | {'Ctrl[1]':>10s} | {'Pos[1]':>10s}")
    print("-" * 80)
    
    sim_time = 0.0
    duration = 3.0
    traj_duration = 2.0
    
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
                
                # 从MuJoCo获取当前关节角度
                q_mj = data.qpos[:6].copy()
                
                # Pinocchio IK计算目标
                q_target, _ = kin.inverse_kinematics(ref_pose, q_mj)
                
                # 直接设置位置指令
                data.ctrl[:6] = q_target
                
                # 仿真
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印
                if step % 250 == 0:
                    curr_pose = kin.forward_kinematics(q_mj)
                    pos_error = ref_pose.position[2] - curr_pose.position[2]
                    print(f"{sim_time:6.2f}s | {ref_pose.position[2]:8.4f} | {curr_pose.position[2]:8.4f} | "
                          f"{pos_error*1000:6.2f}mm | {np.rad2deg(data.ctrl[1]):8.2f}deg | {np.rad2deg(q_mj[1]):8.2f}deg")
                
                sim_time += model.opt.timestep
                step += 1
                time.sleep(model.opt.timestep)
        
        # 最终
        final_pose = kin.forward_kinematics(data.qpos[:6])
        final_error = np.linalg.norm(final_pose.position - target_position)
        z_error = target_position[2] - final_pose.position[2]
        
        print("\n" + "="*60)
        print("最终结果:")
        print(f"  Z误差: {z_error*1000:.2f}mm")
        print(f"  总位置误差: {final_error*1000:.2f}mm")
        
        if abs(z_error) > 0.005:
            print("\n⚠️  仍然有较大误差，说明问题在MuJoCo动力学响应")
            print("   可能原因：")
            print("   1. MuJoCo位置控制器PD增益不足")
            print("   2. 关节摩擦/阻尼导致稳态误差")
            print("   3. 需要在控制器中补偿")
        else:
            print("\n✓ 纯IK位置控制准确")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n测试中断")

if __name__ == '__main__':
    main()
