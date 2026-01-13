#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo阻抗控制快速测试
简化版本，用于快速验证阻抗控制器在MuJoCo中的表现
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import sys

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))

from impedance_control import (
    CartesianState,
    WrenchData,
    ImpedanceParams,
    StandardStrategy,
    AFPStrategy,
    CartesianImpedanceController
)


def main():
    """快速测试主函数"""
    print("\n" + "=" * 60)
    print("MuJoCo阻抗控制快速测试")
    print("=" * 60)
    
    # 路径配置
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 检查文件
    if not os.path.exists(model_path):
        print(f"✗ 找不到MuJoCo场景: {model_path}")
        print("\n可用的场景文件:")
        env_dir = "/home/lgx/Project/AFP/src/afp_mjc/env"
        if os.path.exists(env_dir):
            for item in os.listdir(env_dir):
                item_path = os.path.join(env_dir, item)
                if os.path.isdir(item_path):
                    print(f"  - {item}/")
                    for file in os.listdir(item_path):
                        if file.endswith('.xml'):
                            print(f"    └─ {file}")
        return
    
    if not os.path.exists(urdf_path):
        print(f"✗ 找不到URDF: {urdf_path}")
        return
    
    # 1. 加载MuJoCo模型
    print(f"\n1. 加载MuJoCo模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print(f"   ✓ 自由度: {model.nv}, 控制输入: {model.nu}")
    
    # 2. 设置更好的初始关节配置
    print(f"\n2. 设置初始关节配置...")
    # 使用典型的工作姿态而不是零位
    q_home = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])
    data.qpos[:6] = q_home
    mujoco.mj_forward(model, data)  # 更新运动学
    print(f"   ✓ 设置为典型工作姿态")
    
    # 3. 初始化阻抗控制器
    print(f"\n3. 初始化阻抗控制器...")
    strategy = StandardStrategy()
    controller = CartesianImpedanceController(
        urdf_path=urdf_path,
        strategy=strategy,
        control_frequency=1.0 / model.opt.timestep,
        use_mujoco_frame=False  # 完全基于Pinocchio坐标系工作，不做转换
    )
    
    # 放宽IK参数
    controller.robot_kin.set_ik_parameters(
        eps=1e-3,        # 放宽收敛阈值
        max_iter=200,    # 增加最大迭代次数
        damping=1e-4,    # 调整阻尼
        position_only=True  # 只做位置IK，忽略姿态
    )
    print(f"   ✓ 控制频率: {1.0/model.opt.timestep:.1f} Hz")
    print(f"   ✓ 完全基于Pinocchio坐标系（不做MuJoCo转换）")
    print(f"   ✓ Position-only IK已启用（姿态保持不变）")
    
    # 4. 设置阻抗参数（降低刚度避免过大的位置修正）
    impedance_params = ImpedanceParams.create_uniform(
        stiffness=300.0,  # 降低刚度
        damping=40.0
    )
    print(f"   ✓ 刚度: {impedance_params.stiffness[0]}, 阻尼: {impedance_params.damping[0]}")
    
    # 5. 获取初始关节状态
    print(f"\n4. 获取初始状态...")
    q_init = data.qpos[:6].copy()
    dq_init = data.qvel[:6].copy()
    print(f"   初始关节角度: {np.round(np.rad2deg(q_init), 1)}")
    
    # 6. 计算初始末端位姿（使用Pinocchio）
    print(f"\n5. 正运动学计算...")
    initial_ee_pose = controller.robot_kin.forward_kinematics(q_init)
    print(f"   末端位置: {np.round(initial_ee_pose.position, 3)}")
    print(f"   末端姿态: {np.round(initial_ee_pose.orientation, 3)}")
    
    # 7. 设置参考轨迹（使用轨迹插值）
    print(f"\n6. 生成测试轨迹...")
    target_position = initial_ee_pose.position.copy()
    target_position[2] += 0.01  # 目标：z方向向上移动10mm
    trajectory_duration = 3.0  # 轨迹时长3秒
    print(f"   初始位置: {np.round(initial_ee_pose.position, 3)}")
    print(f"   目标位置: {np.round(target_position, 3)}")
    print(f"   位置偏移: Δz=10mm")
    print(f"   轨迹时长: {trajectory_duration}秒（平滑插值）")
    
    # 8. 模拟力传感器数据（无外力）
    wrench_ee = WrenchData(
        force=np.zeros(3),
        torque=np.zeros(3),
        frame='endeffector'
    )
    
    # 9. 执行一次控制计算
    print(f"\n7. 执行阻抗控制计算...")
    output = controller.compute_control(
        current_joint_state=q_init,
        current_joint_velocity=dq_init,
        reference_cartesian=initial_ee_pose,  # 使用初始位姿作为参考
        current_wrench=wrench_ee,
        impedance_params=impedance_params
    )
    
    if output.success:
        print(f"   ✓ 控制计算成功")
        print(f"   目标关节角度: {np.round(np.rad2deg(output.joint_positions), 1)}")
        print(f"   位置误差: {output.debug_info.get('error_6d_norm', 0):.6f}")
        print(f"   力误差: {output.debug_info.get('delta_F_norm', 0):.6f}")
    else:
        print(f"   ⚠ 控制计算警告: {output.error_msg}")
        print(f"   → 继续运行仿真（使用原始关节角度）...")
        output.joint_positions = q_init  # 使用当前角度继续
    # 10. 在MuJoCo中可视化
    print(f"\n8. 启动MuJoCo可视化...")
    print(f"   提示: 按ESC退出，Ctrl+C提前终止")
    print(f"   观察: 机器人应该平滑移动到目标位置")
    
    sim_time = 0.0
    duration = 5.0  # 总仿真时长5秒
    
    # 轨迹插值函数（五次多项式，保证速度和加速度连续）
    def smooth_trajectory(t, t_total, p_start, p_end):
        """平滑轨迹插值：五次多项式"""
        if t >= t_total:
            return p_end
        s = t / t_total
        # 五次多项式：s(t) = 10s³ - 15s⁴ + 6s⁵
        s_smooth = 10*s**3 - 15*s**4 + 6*s**5
        return p_start + s_smooth * (p_end - p_start)
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print(f"\n开始仿真 (按Ctrl+C提前退出)...")
            step_count = 0
            
            while viewer.is_running() and sim_time < duration:
                # 生成当前时刻的参考位置（平滑插值）
                reference_pose = initial_ee_pose.copy()
                if sim_time < trajectory_duration:
                    reference_pose.position = smooth_trajectory(
                        sim_time, trajectory_duration, 
                        initial_ee_pose.position, target_position
                    )
                else:
                    reference_pose.position = target_position.copy()
                
                # 获取当前状态
                q_curr = data.qpos[:6].copy()
                dq_curr = data.qvel[:6].copy()
                
                # 控制计算
                output = controller.compute_control(
                    current_joint_state=q_curr,
                    current_joint_velocity=dq_curr,
                    reference_cartesian=reference_pose,
                    current_wrench=wrench_ee,
                    impedance_params=impedance_params
                )
                
                # 设置控制指令
                if output.success:
                    data.ctrl[:6] = output.joint_positions
                
                # 仿真步进
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印进度
                if step_count % 500 == 0:
                    curr_pose = controller.robot_kin.forward_kinematics(q_curr)
                    pos_error = np.linalg.norm(reference_pose.position - curr_pose.position)
                    print(f"   t={sim_time:.1f}s | 位置误差={pos_error*1000:.2f}mm")
                
                sim_time += model.opt.timestep
                step_count += 1
                
                # 实时同步
                time.sleep(model.opt.timestep)
    
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    
    print(f"\n" + "=" * 60)
    print(f"测试完成！")
    print(f"  仿真时间: {sim_time:.2f}s")
    print(f"  仿真步数: {step_count}")
    print("=" * 60)


if __name__ == '__main__':
    main()
