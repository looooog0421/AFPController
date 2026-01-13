#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo阻抗控制器仿真测试
在MuJoCo环境中测试笛卡尔空间阻抗控制器
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import sys
from collections import deque

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../afp_robot_control/src'))

from impedance_control import (
    CartesianState,
    WrenchData,
    ImpedanceParams,
    StandardStrategy,
    AFPStrategy,
    CoordinateTransformer,
    CartesianImpedanceController
)


class MuJoCoImpedanceTest:
    """MuJoCo阻抗控制仿真测试"""
    
    def __init__(self, model_path: str, urdf_path: str):
        """
        初始化仿真环境和控制器
        
        Args:
            model_path: MuJoCo场景XML路径
            urdf_path: 机器人URDF路径（用于Pinocchio运动学）
        """
        print("=" * 60)
        print("初始化MuJoCo阻抗控制测试环境")
        print("=" * 60)
        
        # 加载MuJoCo模型
        full_path = os.path.abspath(model_path)
        self.model = mujoco.MjModel.from_xml_path(full_path)
        self.data = mujoco.MjData(self.model)
        
        print(f"✓ MuJoCo模型加载: {model_path}")
        print(f"  自由度: {self.model.nv}")
        print(f"  控制输入: {self.model.nu}")
        print(f"  时间步: {self.model.opt.timestep} s")
        
        # 查找机械臂关节
        self.arm_joint_ids = self._find_arm_joints()
        print(f"  机械臂关节数: {len(self.arm_joint_ids)}")
        
        # 查找末端执行器
        self.ee_body_id = self._find_end_effector()
        print(f"  末端执行器ID: {self.ee_body_id}")
        
        # 初始化阻抗控制器（先用标准策略）
        self.strategy = StandardStrategy()
        self.controller = CartesianImpedanceController(
            urdf_path=urdf_path,
            strategy=self.strategy,
            control_frequency=1.0 / self.model.opt.timestep
        )
        print(f"✓ 阻抗控制器初始化完成")
        
        # 坐标转换器（假设传感器与末端对齐）
        self.coord_transformer = CoordinateTransformer()
        
        # 阻抗参数
        self.impedance_params = ImpedanceParams.create_anisotropic(
            trans_stiffness=np.array([500, 500, 500]),
            rot_stiffness=np.array([10, 10, 10]),
            trans_damping=np.array([50, 50, 50]),
            rot_damping=np.array([1, 1, 1])
        )
        
        # 参考轨迹（初始为当前位置）
        self.reference_pose = None
        
        # 模拟力传感器（通过接触力计算）
        self.contact_force = np.zeros(3)
        self.contact_torque = np.zeros(3)
        
        # 数据记录
        self.time_history = []
        self.pos_error_history = []
        self.force_history = []
        self.joint_cmd_history = []
        
        print("=" * 60)
    
    def _find_arm_joints(self):
        """查找机械臂关节"""
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                      'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_ids = []
        for name in joint_names:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id >= 0:
                joint_ids.append(jnt_id)
        
        # 如果找不到命名关节，使用前N个
        if len(joint_ids) == 0:
            print("  警告: 未找到命名关节，使用前6个关节")
            joint_ids = list(range(min(6, self.model.nv)))
        
        return joint_ids
    
    def _find_end_effector(self):
        """查找末端执行器"""
        ee_names = ['flange', 'wrist_3_link', 'ee_link', 'tool0']
        for name in ee_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                return body_id
        
        # 如果找不到，使用最后一个body
        print("  警告: 未找到命名末端，使用最后一个body")
        return self.model.nbody - 1
    
    def get_joint_state(self):
        """获取当前关节状态"""
        q = np.zeros(len(self.arm_joint_ids))
        dq = np.zeros(len(self.arm_joint_ids))
        
        for i, jnt_id in enumerate(self.arm_joint_ids):
            qpos_adr = self.model.jnt_qposadr[jnt_id]
            dof_adr = self.model.jnt_dofadr[jnt_id]
            q[i] = self.data.qpos[qpos_adr]
            dq[i] = self.data.qvel[dof_adr]
        
        return q, dq
    
    def get_ee_pose(self):
        """获取末端执行器位姿（从MuJoCo）"""
        # 位置
        pos = self.data.xpos[self.ee_body_id].copy()
        
        # 姿态（旋转矩阵转四元数）
        rot_mat = self.data.xmat[self.ee_body_id].reshape(3, 3).copy()
        from scipy.spatial.transform import Rotation as R
        quat_xyzw = R.from_matrix(rot_mat).as_quat()
        quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        
        return CartesianState(position=pos, orientation=quat)
    
    def get_contact_wrench(self):
        """获取接触力/力矩"""
        # 简化：累加所有接触力
        force = np.zeros(3)
        torque = np.zeros(3)
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            # 检查是否涉及末端执行器
            if contact.geom1 == self.ee_body_id or contact.geom2 == self.ee_body_id:
                # 接触力（法向+切向）
                contact_force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, contact_force)
                
                # 转换到世界坐标系
                force_local = contact_force[:3]
                # 简化：假设接触力已在全局坐标系
                force += contact.frame[:3] * contact_force[0]  # 法向力
        
        return WrenchData(force=force, torque=torque, frame='base')
    
    def set_joint_command(self, q_target: np.ndarray):
        """设置关节位置指令"""
        for i, jnt_id in enumerate(self.arm_joint_ids):
            if i < len(q_target):
                # MuJoCo位置控制
                ctrl_adr = self.model.jnt_qposadr[jnt_id]
                if ctrl_adr < self.model.nu:
                    self.data.ctrl[ctrl_adr] = q_target[i]
    
    def run_trajectory_test(self, trajectory_func, duration: float = 10.0, 
                           use_afp: bool = False, target_force: float = 20.0):
        """
        运行轨迹跟踪测试
        
        Args:
            trajectory_func: 轨迹生成函数 trajectory_func(t) -> CartesianState
            duration: 测试时长（秒）
            use_afp: 是否使用AFP策略
            target_force: AFP目标接触力
        """
        print("\n" + "=" * 60)
        print(f"开始轨迹跟踪测试 ({'AFP' if use_afp else '标准'}策略)")
        print("=" * 60)
        
        # 切换策略
        if use_afp:
            self.strategy = AFPStrategy(target_contact_force=target_force)
            self.controller.set_strategy(self.strategy)
            print(f"✓ 使用AFP策略，目标接触力: {target_force}N")
        else:
            self.strategy = StandardStrategy()
            self.controller.set_strategy(self.strategy)
            print("✓ 使用标准策略")
        
        # 重置数据记录
        self.time_history = []
        self.pos_error_history = []
        self.force_history = []
        self.joint_cmd_history = []
        
        # 初始化参考轨迹为当前位置
        self.reference_pose = self.get_ee_pose()
        
        # 启动仿真
        print("\n开始仿真...")
        start_time = time.time()
        sim_time = 0.0
        step_count = 0
        
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and sim_time < duration:
                    step_start = time.time()
                    
                    # 1. 获取当前状态
                    q_curr, dq_curr = self.get_joint_state()
                    ee_pose_mj = self.get_ee_pose()
                    
                    # 2. 生成参考轨迹
                    self.reference_pose = trajectory_func(sim_time)
                    
                    # 3. 获取力传感器数据
                    wrench = self.get_contact_wrench()
                    # 转换到末端坐标系
                    ee_rot = self.data.xmat[self.ee_body_id].reshape(3, 3)
                    wrench_ee = WrenchData(
                        force=ee_rot.T @ wrench.force,
                        torque=ee_rot.T @ wrench.torque,
                        frame='endeffector'
                    )
                    
                    # 4. 阻抗控制计算
                    output = self.controller.compute_control(
                        current_joint_state=q_curr,
                        current_joint_velocity=dq_curr,
                        reference_cartesian=self.reference_pose,
                        current_wrench=wrench_ee,
                        impedance_params=self.impedance_params
                    )
                    
                    # 5. 发送控制指令
                    if output.success:
                        self.set_joint_command(output.joint_positions)
                    
                    # 6. 仿真步进
                    mujoco.mj_step(self.model, self.data)
                    viewer.sync()
                    
                    # 7. 记录数据
                    if step_count % 10 == 0:  # 每10步记录一次
                        pos_error = np.linalg.norm(
                            self.reference_pose.position - ee_pose_mj.position)
                        self.time_history.append(sim_time)
                        self.pos_error_history.append(pos_error)
                        self.force_history.append(np.linalg.norm(wrench_ee.force))
                        self.joint_cmd_history.append(output.joint_positions.copy())
                        
                        # 打印状态
                        if step_count % 100 == 0:
                            print(f"  t={sim_time:.2f}s | "
                                  f"位置误差={pos_error*1000:.2f}mm | "
                                  f"力={np.linalg.norm(wrench_ee.force):.2f}N | "
                                  f"IK={'✓' if output.success else '✗'}")
                    
                    # 8. 时间控制
                    sim_time += self.model.opt.timestep
                    step_count += 1
                    
                    # 实时同步
                    elapsed = time.time() - step_start
                    if elapsed < self.model.opt.timestep:
                        time.sleep(self.model.opt.timestep - elapsed)
        
        except KeyboardInterrupt:
            print("\n用户中断测试")
        
        real_time = time.time() - start_time
        print(f"\n测试完成:")
        print(f"  仿真时间: {sim_time:.2f}s")
        print(f"  实际时间: {real_time:.2f}s")
        print(f"  仿真步数: {step_count}")
        print(f"  平均位置误差: {np.mean(self.pos_error_history)*1000:.2f}mm")
        print(f"  最大位置误差: {np.max(self.pos_error_history)*1000:.2f}mm")
        
        return self.time_history, self.pos_error_history, self.force_history
    
    def plot_results(self):
        """绘制测试结果"""
        try:
            import matplotlib.pyplot as plt
            
            fig, axes = plt.subplots(2, 1, figsize=(10, 8))
            
            # 位置误差
            axes[0].plot(self.time_history, np.array(self.pos_error_history) * 1000)
            axes[0].set_xlabel('时间 (s)')
            axes[0].set_ylabel('位置误差 (mm)')
            axes[0].set_title('轨迹跟踪误差')
            axes[0].grid(True)
            
            # 接触力
            axes[1].plot(self.time_history, self.force_history)
            axes[1].set_xlabel('时间 (s)')
            axes[1].set_ylabel('接触力 (N)')
            axes[1].set_title('接触力变化')
            axes[1].grid(True)
            
            plt.tight_layout()
            plt.show()
            
        except ImportError:
            print("提示: 安装matplotlib可以可视化结果 (pip install matplotlib)")


def circular_trajectory(center, radius, omega, t):
    """圆形轨迹"""
    pos = center.copy()
    pos[0] += radius * np.cos(omega * t)
    pos[2] += radius * np.sin(omega * t)
    
    # 保持姿态不变
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    
    return CartesianState(position=pos, orientation=quat)


def main():
    """主测试函数"""
    print("\n" + "╔" + "=" * 58 + "╗")
    print("║" + " " * 15 + "MuJoCo阻抗控制测试" + " " * 25 + "║")
    print("╚" + "=" * 58 + "╝\n")
    
    # 路径配置
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
    
    # 检查文件
    if not os.path.exists(model_path):
        print(f"✗ MuJoCo场景文件不存在: {model_path}")
        return
    if not os.path.exists(urdf_path):
        print(f"✗ URDF文件不存在: {urdf_path}")
        return
    
    # 创建测试环境
    test_env = MuJoCoImpedanceTest(model_path, urdf_path)
    
    # 获取初始位置
    initial_pose = test_env.get_ee_pose()
    print(f"\n初始末端位置: {initial_pose.position}")
    
    # 测试1: 固定位置保持（标准策略）
    print("\n" + "=" * 60)
    print("测试选项:")
    print("  1. 固定位置保持（标准策略）")
    print("  2. 圆形轨迹跟踪（标准策略）")
    print("  3. 圆形轨迹跟踪（AFP策略）")
    print("=" * 60)
    
    choice = input("请选择测试 (1/2/3，直接回车默认1): ").strip() or "1"
    
    if choice == "1":
        # 固定位置
        trajectory = lambda t: initial_pose
        test_env.run_trajectory_test(trajectory, duration=5.0, use_afp=False)
    
    elif choice == "2":
        # 圆形轨迹
        center = initial_pose.position.copy()
        radius = 0.05  # 5cm半径
        omega = 0.5    # 角速度
        trajectory = lambda t: circular_trajectory(center, radius, omega, t)
        test_env.run_trajectory_test(trajectory, duration=10.0, use_afp=False)
    
    elif choice == "3":
        # AFP策略
        center = initial_pose.position.copy()
        radius = 0.05
        omega = 0.5
        trajectory = lambda t: circular_trajectory(center, radius, omega, t)
        test_env.run_trajectory_test(trajectory, duration=10.0, use_afp=True, target_force=15.0)
    
    # 绘制结果
    if len(test_env.time_history) > 0:
        plot_choice = input("\n是否绘制结果? (y/n，默认n): ").strip().lower()
        if plot_choice == 'y':
            test_env.plot_results()
    
    print("\n测试完成！")


if __name__ == '__main__':
    main()
