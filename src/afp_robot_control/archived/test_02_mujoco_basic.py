#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试2: MuJoCo基础控制
验证MuJoCo加载、关节控制、基本仿真
"""
import mujoco
import mujoco.viewer
import numpy as np
import time

def test_mujoco_basic():
    print("\n" + "="*60)
    print("测试2: MuJoCo基础控制")
    print("="*60)
    
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    
    # 1. 加载模型
    print("\n1. 加载MuJoCo模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print(f"   ✓ 自由度: {model.nv}")
    print(f"   ✓ 控制输入: {model.nu}")
    print(f"   ✓ 时间步长: {model.opt.timestep}s ({1/model.opt.timestep:.0f}Hz)")
    
    # 2. 设置初始姿态
    print("\n2. 设置初始关节角度...")
    q_home = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])
    data.qpos[:6] = q_home
    mujoco.mj_forward(model, data)
    print(f"   ✓ 关节角度 (度): {np.round(np.rad2deg(q_home), 1)}")
    
    # 3. 测试基本仿真
    print("\n3. 测试位置控制...")
    print("   将在可视化窗口中让关节1旋转30度")
    print("   提示: 观察3秒后自动关闭")
    
    # 目标关节角度（关节1旋转30度）
    q_target = q_home.copy()
    q_target[0] += np.deg2rad(30)
    
    sim_time = 0.0
    duration = 3.0
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n   开始仿真...")
            step_count = 0
            
            while viewer.is_running() and sim_time < duration:
                # 平滑插值到目标
                alpha = min(sim_time / 2.0, 1.0)  # 2秒内到达
                data.ctrl[:6] = (1-alpha) * q_home + alpha * q_target
                
                # 仿真步进
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # 打印进度
                if step_count % 500 == 0:
                    current_q = data.qpos[:6]
                    print(f"   t={sim_time:.1f}s | 关节1: {np.rad2deg(current_q[0]):.1f}度")
                
                sim_time += model.opt.timestep
                step_count += 1
                time.sleep(model.opt.timestep)
        
        print(f"\n   ✓ 仿真完成")
        print(f"   最终关节1角度: {np.rad2deg(data.qpos[0]):.1f}度 (目标: 30度)")
        
        # 检查是否到达目标
        error = abs(data.qpos[0] - q_target[0])
        if error < np.deg2rad(5):  # 5度误差
            print("   ✓ 位置控制成功")
            return True
        else:
            print(f"   ✗ 误差过大: {np.rad2deg(error):.1f}度")
            return False
            
    except KeyboardInterrupt:
        print("\n   测试被用户中断")
        return False

if __name__ == '__main__':
    success = test_mujoco_basic()
    print("\n" + "="*60)
    if success:
        print("测试2通过 ✓")
    else:
        print("测试2失败 ✗")
    print("="*60)
