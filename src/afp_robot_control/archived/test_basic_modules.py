#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阻抗控制器基础模块测试（不需要Pinocchio）
测试核心数据结构、策略和坐标转换
"""
import numpy as np
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

from impedance_control.impedance_types import (
    CartesianState,
    WrenchData,
    ImpedanceParams
)

from impedance_control.task_strategy import (
    StandardStrategy,
    AFPStrategy,
    HybridStrategy
)

from impedance_control.coordinate_transform import CoordinateTransformer


def test_cartesian_state():
    """测试笛卡尔状态"""
    print("=" * 60)
    print("测试1: 笛卡尔状态")
    print("=" * 60)
    
    # 创建状态
    state = CartesianState(
        position=np.array([0.5, 0.0, 0.3]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])  # [qw, qx, qy, qz]
    )
    
    print(f"位置: {state.position}")
    print(f"姿态 (四元数): {state.orientation}")
    print(f"速度: {state.velocity}")
    
    # 测试转换为齐次矩阵
    T = state.to_pose_matrix()
    print(f"齐次变换矩阵:\n{T}")
    
    # 测试从齐次矩阵创建
    state2 = CartesianState.from_pose_matrix(T)
    print(f"重构位置: {state2.position}")
    print(f"重构姿态: {state2.orientation}")
    
    assert np.allclose(state.position, state2.position), "位置不匹配"
    assert np.allclose(state.orientation, state2.orientation), "姿态不匹配"
    
    print("✓ 笛卡尔状态测试通过\n")


def test_wrench_data():
    """测试力/力矩数据"""
    print("=" * 60)
    print("测试2: 力/力矩数据")
    print("=" * 60)
    
    wrench = WrenchData(
        force=np.array([10.0, 5.0, 20.0]),
        torque=np.array([0.5, 0.2, 0.1]),
        frame='endeffector'
    )
    
    print(f"力: {wrench.force}")
    print(f"力矩: {wrench.torque}")
    print(f"坐标系: {wrench.frame}")
    
    # 转换为6D数组
    wrench_6d = wrench.to_array()
    print(f"6D数组: {wrench_6d}")
    
    # 从6D数组创建
    wrench2 = WrenchData.from_array(wrench_6d, frame='endeffector')
    assert np.allclose(wrench.force, wrench2.force), "力不匹配"
    assert np.allclose(wrench.torque, wrench2.torque), "力矩不匹配"
    
    print("✓ 力/力矩数据测试通过\n")


def test_impedance_params():
    """测试阻抗参数"""
    print("=" * 60)
    print("测试3: 阻抗参数")
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
    print(f"  平移 [Kx, Ky, Kz]: {params2.stiffness[:3]}")
    print(f"  旋转 [Krx, Kry, Krz]: {params2.stiffness[3:]}")
    print(f"各向异性阻尼: {params2.damping}")
    
    print("✓ 阻抗参数测试通过\n")


def test_standard_strategy():
    """测试标准策略"""
    print("=" * 60)
    print("测试4: 标准策略")
    print("=" * 60)
    
    strategy = StandardStrategy(target_wrench=np.zeros(6))
    
    # 创建测试状态
    current_state = CartesianState(
        position=np.array([0.5, 0.0, 0.3]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])
    )
    
    reference_state = CartesianState(
        position=np.array([0.52, 0.01, 0.31]),  # 期望位置
        orientation=np.array([0.9999, 0.01, 0.0, 0.0])  # 微小旋转
    )
    
    # 计算误差
    pos_err, ori_err = strategy.compute_error(current_state, reference_state)
    
    print(f"位置误差: {pos_err}")
    print(f"  期望移动: [0.02, 0.01, 0.01] m")
    print(f"姿态误差 (轴角): {ori_err}")
    
    # 验证位置误差
    expected_pos_err = reference_state.position - current_state.position
    assert np.allclose(pos_err, expected_pos_err, atol=1e-6), "位置误差计算错误"
    
    print("✓ 标准策略测试通过\n")


def test_afp_strategy():
    """测试AFP策略"""
    print("=" * 60)
    print("测试5: AFP策略")
    print("=" * 60)
    
    # 创建AFP策略：目标接触力20N
    strategy = AFPStrategy(target_contact_force=20.0)
    
    # 测试场景1: 当前合力小于目标（需要增加压力）
    print("场景1: 当前合力 < 目标合力")
    current_wrench1 = WrenchData(
        force=np.array([3.0, 0.0, 12.0]),  # fx=3, fz=12, 合力=√(9+144)≈12.37
        torque=np.zeros(3),
        frame='endeffector'
    )
    
    F_curr1 = strategy.get_current_contact_force(current_wrench1)
    desired_wrench1 = strategy.compute_desired_wrench(current_wrench1)
    
    print(f"  当前力: fx={current_wrench1.force[0]:.1f}, fz={current_wrench1.force[2]:.1f}")
    print(f"  当前合力: {F_curr1:.2f} N")
    print(f"  目标合力: {strategy.F_target:.2f} N")
    print(f"  期望力调整: fx={desired_wrench1[0]:.2f}, fz={desired_wrench1[2]:.2f}")
    print(f"  调整比例: {desired_wrench1[0]/desired_wrench1[2]:.3f} (应该≈ {current_wrench1.force[0]/current_wrench1.force[2]:.3f})")
    
    # 验证调整方向
    assert desired_wrench1[0] > 0 and desired_wrench1[2] > 0, "应该增加x和z方向的力"
    
    # 测试场景2: 当前合力大于目标（需要减小压力）
    print("\n场景2: 当前合力 > 目标合力")
    current_wrench2 = WrenchData(
        force=np.array([10.0, 0.0, 20.0]),  # 合力≈22.36
        torque=np.zeros(3),
        frame='endeffector'
    )
    
    F_curr2 = strategy.get_current_contact_force(current_wrench2)
    desired_wrench2 = strategy.compute_desired_wrench(current_wrench2)
    
    print(f"  当前合力: {F_curr2:.2f} N")
    print(f"  期望力调整: fx={desired_wrench2[0]:.2f}, fz={desired_wrench2[2]:.2f}")
    
    # 验证调整方向
    assert desired_wrench2[0] < 0 and desired_wrench2[2] < 0, "应该减小x和z方向的力"
    
    print("✓ AFP策略测试通过\n")


def test_coordinate_transform():
    """测试坐标转换"""
    print("=" * 60)
    print("测试6: 坐标系转换")
    print("=" * 60)
    
    # 测试1: 绕z轴旋转90度
    print("测试6.1: 绕Z轴旋转90度")
    transformer_z90 = CoordinateTransformer(rotation_axis='z', angle_deg=90.0)
    
    wrench_sensor = WrenchData(
        force=np.array([10.0, 0.0, 0.0]),  # 传感器x方向10N
        torque=np.zeros(3),
        frame='sensor'
    )
    
    wrench_ee = transformer_z90.transform_wrench_to_ee(wrench_sensor)
    
    print(f"  传感器坐标系力: {wrench_sensor.force}")
    print(f"  末端坐标系力: {wrench_ee.force}")
    print(f"  理论值: [0, 10, 0]")
    
    assert np.allclose(wrench_ee.force, [0, 10, 0], atol=1e-6), "Z轴旋转90度转换错误"
    
    # 测试2: 绕z轴旋转180度
    print("\n测试6.2: 绕Z轴旋转180度")
    transformer_z180 = CoordinateTransformer(rotation_axis='z', angle_deg=180.0)
    
    wrench_ee2 = transformer_z180.transform_wrench_to_ee(wrench_sensor)
    print(f"  传感器坐标系力: {wrench_sensor.force}")
    print(f"  末端坐标系力: {wrench_ee2.force}")
    print(f"  理论值: [-10, 0, 0]")
    
    assert np.allclose(wrench_ee2.force, [-10, 0, 0], atol=1e-6), "Z轴旋转180度转换错误"
    
    # 测试3: 无旋转
    print("\n测试6.3: 无旋转")
    transformer_identity = CoordinateTransformer()
    wrench_ee3 = transformer_identity.transform_wrench_to_ee(wrench_sensor)
    print(f"  传感器坐标系力: {wrench_sensor.force}")
    print(f"  末端坐标系力: {wrench_ee3.force}")
    
    assert np.allclose(wrench_ee3.force, wrench_sensor.force, atol=1e-6), "单位变换错误"
    
    print("✓ 坐标转换测试通过\n")


def test_hybrid_strategy():
    """测试混合策略"""
    print("=" * 60)
    print("测试7: 混合策略")
    print("=" * 60)
    
    # Z轴力控制，其他位置控制
    strategy = HybridStrategy(
        force_controlled_dofs=[2],  # Z轴（索引2）
        target_wrench=np.array([0, 0, 10, 0, 0, 0])  # Z方向10N
    )
    
    current_state = CartesianState(
        position=np.array([0.5, 0.0, 0.3]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])
    )
    
    reference_state = CartesianState(
        position=np.array([0.52, 0.01, 0.31]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0])
    )
    
    # 计算误差
    pos_err, ori_err = strategy.compute_error(current_state, reference_state)
    error_6d = np.concatenate([pos_err, ori_err])
    
    print(f"位置误差: {pos_err}")
    print(f"  注意: Z轴(索引2)误差应为0（力控制）")
    print(f"  实际Z轴误差: {pos_err[2]}")
    
    assert abs(pos_err[2]) < 1e-10, "力控制自由度误差应为0"
    assert abs(pos_err[0]) > 0 or abs(pos_err[1]) > 0, "位置控制自由度应有误差"
    
    print("✓ 混合策略测试通过\n")


def run_all_tests():
    """运行所有测试"""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 10 + "阻抗控制器基础模块测试" + " " * 22 + "║")
    print("║" + " " * 16 + "(不依赖Pinocchio)" + " " * 23 + "║")
    print("╚" + "=" * 58 + "╝")
    print()
    
    try:
        test_cartesian_state()
        test_wrench_data()
        test_impedance_params()
        test_standard_strategy()
        test_afp_strategy()
        test_coordinate_transform()
        test_hybrid_strategy()
        
        print("=" * 60)
        print("✓ 所有基础模块测试通过！")
        print("=" * 60)
        print("\n提示: 要测试完整控制器（需要运动学），请先安装Pinocchio:")
        print("  pip install pin")
        print("  或参考: CMAKE_SETUP.md")
        print()
    
    except Exception as e:
        print(f"\n✗ 测试失败: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    run_all_tests()
