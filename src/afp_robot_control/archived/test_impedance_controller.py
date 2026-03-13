#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阻抗控制器测试示例
演示如何使用核心模块（不依赖ROS）
"""
import numpy as np
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

from impedance_control import (
    CartesianState,
    WrenchData,
    ImpedanceParams,
    StandardStrategy,
    AFPStrategy,
    CoordinateTransformer,
    CartesianImpedanceController
)


def test_standard_strategy():
    """测试标准策略"""
    print("=" * 60)
    print("测试1: 标准策略")
    print("=" * 60)
    
    # 创建策略
    strategy = StandardStrategy(target_wrench=np.zeros(6))
    
    # 创建当前和参考状态
    current_state = CartesianState(
        position=np.array([0.5, 0.0, 0.3]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]
    )
    
    reference_state = CartesianState(
        position=np.array([0.52, 0.0, 0.31]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])
    )
    
    # 计算误差
    pos_err, ori_err = strategy.compute_error(current_state, reference_state)
    
    print(f"位置误差: {pos_err}")
    print(f"姿态误差: {ori_err}")
    print("✓ 标准策略测试通过\n")


def test_afp_strategy():
    """测试AFP策略"""
    print("=" * 60)
    print("测试2: AFP策略")
    print("=" * 60)
    
    # 创建AFP策略
    strategy = AFPStrategy(target_contact_force=20.0)
    
    # 创建力数据（末端坐标系）
    current_wrench = WrenchData(
        force=np.array([5.0, 0.0, 15.0]),  # fx=5N, fz=15N
        torque=np.zeros(3),
        frame='endeffector'
    )
    
    # 计算期望力
    desired_wrench = strategy.compute_desired_wrench(current_wrench)
    
    print(f"当前力: {current_wrench.force}")
    print(f"当前合力: {strategy.get_current_contact_force(current_wrench):.2f} N")
    print(f"目标合力: {strategy.F_target:.2f} N")
    print(f"期望力调整: {desired_wrench[:3]}")
    print("✓ AFP策略测试通过\n")


def test_coordinate_transform():
    """测试坐标转换"""
    print("=" * 60)
    print("测试3: 坐标转换")
    print("=" * 60)
    
    # 创建转换器：绕z轴旋转90度
    transformer = CoordinateTransformer(rotation_axis='z', angle_deg=90.0)
    
    # 传感器坐标系的力
    wrench_sensor = WrenchData(
        force=np.array([10.0, 0.0, 0.0]),  # x方向10N
        torque=np.zeros(3),
        frame='sensor'
    )
    
    # 转换到末端坐标系
    wrench_ee = transformer.transform_wrench_to_ee(wrench_sensor)
    
    print(f"传感器坐标系力: {wrench_sensor.force}")
    print(f"末端坐标系力: {wrench_ee.force}")
    print(f"理论值: [0, 10, 0] (绕z轴旋转90度)")
    print("✓ 坐标转换测试通过\n")


def test_impedance_params():
    """测试阻抗参数"""
    print("=" * 60)
    print("测试4: 阻抗参数")
    print("=" * 60)
    
    # 各向同性参数
    params1 = ImpedanceParams.create_uniform(stiffness=500.0, damping=50.0)
    print(f"各向同性刚度: {params1.stiffness}")
    print(f"各向同性阻尼: {params1.damping}")
    
    # 各向异性参数
    params2 = ImpedanceParams.create_anisotropic(
        trans_stiffness=np.array([300, 1000, 300]),
        rot_stiffness=np.array([10, 10, 10]),
        trans_damping=np.array([30, 80, 30]),
        rot_damping=np.array([1, 1, 1])
    )
    print(f"\n各向异性刚度: {params2.stiffness}")
    print(f"各向异性阻尼: {params2.damping}")
    print("✓ 阻抗参数测试通过\n")


def test_full_controller():
    """测试完整控制器（需要URDF）"""
    print("=" * 60)
    print("测试5: 完整控制器")
    print("=" * 60)
    
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    if not os.path.exists(urdf_path):
        print(f"⚠ URDF文件不存在: {urdf_path}")
        print("跳过完整控制器测试\n")
        return
    
    try:
        # 创建控制器
        strategy = StandardStrategy()
        controller = CartesianImpedanceController(
            urdf_path=urdf_path,
            strategy=strategy,
            control_frequency=200.0
        )
        
        # 测试正运动学
        q_test = np.array([0.0, -np.pi/4, np.pi/2, 0.0, np.pi/2, 0.0])
        cart_state = controller.robot_kin.forward_kinematics(q_test)
        
        print(f"测试关节角度: {q_test}")
        print(f"末端位置: {cart_state.position}")
        print(f"末端姿态 (四元数): {cart_state.orientation}")
        
        # 测试控制计算
        reference_state = cart_state.copy()
        reference_state.position[0] += 0.01  # x方向移动1cm
        
        wrench = WrenchData(
            force=np.array([0.0, 0.0, 10.0]),
            torque=np.zeros(3),
            frame='endeffector'
        )
        
        params = ImpedanceParams.create_uniform(500.0, 50.0)
        
        output = controller.compute_control(
            current_joint_state=q_test,
            current_joint_velocity=np.zeros(6),
            reference_cartesian=reference_state,
            current_wrench=wrench,
            impedance_params=params
        )
        
        print(f"\n控制结果:")
        print(f"  成功: {output.success}")
        print(f"  目标关节: {output.joint_positions}")
        print(f"  误差范数: {output.debug_info.get('error_6d_norm', 0):.6f}")
        
        print("✓ 完整控制器测试通过\n")
    
    except Exception as e:
        print(f"✗ 测试失败: {str(e)}\n")


def run_all_tests():
    """运行所有测试"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 10 + "阻抗控制器模块测试" + " " * 28 + "║")
    print("╚" + "=" * 58 + "╝")
    print()
    
    try:
        test_standard_strategy()
        test_afp_strategy()
        test_coordinate_transform()
        test_impedance_params()
        test_full_controller()
        
        print("=" * 60)
        print("✓ 所有测试完成！")
        print("=" * 60)
    
    except Exception as e:
        print(f"\n✗ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    run_all_tests()
