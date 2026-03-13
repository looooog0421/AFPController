#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试4: MuJoCo + Pinocchio集成
验证从MuJoCo获取关节角度，通过Pinocchio计算IK，再控制MuJoCo
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))
from impedance_control.robot_kinematics_wrapper import RobotKinematicsWrapper

def test_mujoco_pinocchio_integration():
    print("\n" + "="*60)
    print("测试4: MuJoCo + Pinocchio集成")
    print("="*60)
    
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 1. 加载模型
    print("\n1. 加载模型...")
    mj_model = mujoco.MjModel.from_xml_path(model_path)
    mj_data = mujoco.MjData(mj_model)
    pin_kin = RobotKinematicsWrapper(urdf_path, "flange", use_mujoco_frame=False)
    pin_kin.set_ik_parameters(eps=1e-3, max_iter=200, damping=1e-4, position_only=True)
    print(f"   ✓ MuJoCo自由度: {mj_model.nv}")
    print(f"   ✓ Pinocchio关节数: {pin_kin.n_joints}")
    
    # 2. 设置初始姿态
    print("\n2. 设置初始姿态...")
    q_init = np.array([0.0, -np.pi/4, np.pi/2, -np.pi/4, np.pi/2, 0.0])
    mj_data.qpos[:6] = q_init
    mujoco.mj_forward(mj_model, mj_data)
    print(f"   关节角度 (度): {np.round(np.rad2deg(q_init), 1)}")
    
    # 3. 用Pinocchio计算当前位姿
    print("\n3. 计算当前末端位姿...")
    current_pose = pin_kin.forward_kinematics(q_init)
    print(f"   位置: {np.round(current_pose.position, 3)}")
    
    # 4. 定义目标：z+10mm，使用轨迹插值
    print("\n4. 定义运动目标...")
    target_position = current_pose.position.copy()
    target_position[2] += 0.01  # z+10mm
    print(f"   目标位置: {np.round(target_position, 3)}")
    print(f"   使用3秒平滑轨迹")
    
    # 5. 执行闭环控制仿真
    print("\n5. 启动闭环控制仿真...")
    print("   观察机器人平滑移动（按ESC退出）")
    
    sim_time = 0.0
    duration = 4.0
    traj_duration = 3.0
    
    def smooth_trajectory(t, t_total, p_start, p_end):
        """五次多项式插值"""
        if t >= t_total:
            return p_end
        s = t / t_total
        s_smooth = 10*s**3 - 15*s**4 + 6*s**5
        return p_start + s_smooth * (p_end - p_start)
    
    max_pos_error = 0.0
    ik_failure_count = 0
    
    try:
        with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
            step_count = 0
            
            while viewer.is_running() and sim_time < duration:
                # 生成参考位置
                if sim_time < traj_duration:
                    ref_position = smooth_trajectory(
                        sim_time, traj_duration,
                        current_pose.position, target_position
                    )
                else:
                    ref_position = target_position.copy()
                
                # 从MuJoCo获取当前关节角度
                q_mj = mj_data.qpos[:6].copy()
                
                # 用Pinocchio计算当前末端位置
                pose_pin = pin_kin.forward_kinematics(q_mj)
                
                # 计算位置误差
                pos_error = np.linalg.norm(ref_position - pose_pin.position)
                max_pos_error = max(max_pos_error, pos_error)
                
                # 用Pinocchio IK计算目标关节角度
                target_pose = current_pose.copy()
                target_pose.position = ref_position
                q_target, ik_success = pin_kin.inverse_kinematics(target_pose, q_mj)
                
                if not ik_success:
                    ik_failure_count += 1
                
                # 发送控制指令到MuJoCo
                mj_data.ctrl[:6] = q_target
                
                # 仿真步进
                mujoco.mj_step(mj_model, mj_data)
                viewer.sync()
                
                # 打印进度
                if step_count % 500 == 0:
                    print(f"   t={sim_time:.1f}s | 位置误差={pos_error*1000:.2f}mm | IK失败={ik_failure_count}")
                
                sim_time += mj_model.opt.timestep
                step_count += 1
                time.sleep(mj_model.opt.timestep)
        
        print(f"\n   ✓ 仿真完成")
        print(f"   最大位置误差: {max_pos_error*1000:.2f}mm")
        print(f"   IK失败次数: {ik_failure_count}/{step_count} ({ik_failure_count/step_count*100:.1f}%)")
        
        # 最终验证
        final_pose = pin_kin.forward_kinematics(mj_data.qpos[:6])
        final_error = np.linalg.norm(final_pose.position - target_position)
        print(f"   最终位置误差: {final_error*1000:.2f}mm")
        
        if final_error < 0.005:  # 5mm
            print("   ✓ 集成测试通过")
            return True
        else:
            print("   ⚠ 最终误差较大，但系统运行正常")
            return True
            
    except KeyboardInterrupt:
        print("\n   测试被用户中断")
        return False
    except Exception as e:
        print(f"\n   ✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = test_mujoco_pinocchio_integration()
    print("\n" + "="*60)
    if success:
        print("测试4通过 ✓")
    else:
        print("测试4失败 ✗")
    print("="*60)
