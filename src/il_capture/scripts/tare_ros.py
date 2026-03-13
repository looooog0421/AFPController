#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, AccelStamped
import numpy as np
from tare_netft import tare_netft
import message_filters
import os
import sys
import yaml
import tf.transformations as tf_trans
from rigidbodytracker import RigidBodyTracker
from il_capture.msg import KineticStateStamped

def get_parent_dir():
    script_path = os.path.realpath(sys.argv[0])
    parent_dir = os.path.dirname(os.path.dirname(script_path))
    return parent_dir

"""
任务描述：
读取ROS话题中的力传感器数据和姿态数据,进行力传感器的重力补偿标定,并输出标定结果。
步骤：
1. 初始化ROS节点,订阅力传感器数据和动捕点数据话题
1.1. 动捕点有五个,构成一个四棱锥,其中marker点在刚体局部坐标系的坐标为:
        (0, 0, 0.1175)
        (0.03*sqrt(3), 0.03, -0.003)
        (0.035*sqrt(3), -0.035, -0.003)
        (-0.03*sqrt(3), -0.03, -0.003)
        (-0.04*sqrt(3), 0.04, -0.003)
2. 收集数据,当前记录的位置需要与上一时刻的位置有一定变化,以确保多样性,采集100个数据点后停止
3. 调用tare_netft函数进行标定,获取质量、静态力偏置、质心位置和静态力矩偏置
4. 输出标定结果
"""

class ToolGravityTare:
    def __init__(self, required_samples=30, quat_dot_threshold=0.95):
        rospy.init_node('tool_gravity_tare', anonymous=True)
        
        # 参数
        self.required_samples = required_samples
        self.quat_dot_threshold = quat_dot_threshold
        
        # 标定数据列表
        self.force_list = []
        self.torque_list = []
        self.quat_list = []
        self.last_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.is_collecting = True
        
        # 阈值参数
        self.MAX_LINEAR_V = 0.01
        self.MAX_ANGULAR_V = 0.03
        self.MAX_LINEAR_A = 0.15
        self.MAX_ANGULAR_A = 0.5
        
        # ===== 使用 ApproximateTimeSynchronizer =====
        
        # 1. 创建订阅者（不直接指定callback）
        wrench_sub = message_filters.Subscriber("/netft_data", WrenchStamped)
        kinetic_sub = message_filters.Subscriber("/mimic_tool/kinetic_state", KineticStateStamped)
        
        # 2. 创建时间同步器
        # queue_size: 缓存队列大小，越大越容易匹配但延迟更高
        # slop: 允许的最大时间差（秒），0.05表示50ms
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [wrench_sub, kinetic_sub],
            queue_size=10,
            slop=0.05,  # 50ms的时间容差
            allow_headerless=False
        )
        
        # 3. 注册同步回调函数
        self.sync.registerCallback(self.synchronized_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Tool Gravity Tare Node Initialized (with Time Sync)")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Required samples: {self.required_samples}")
        rospy.loginfo(f"Time synchronization tolerance: 50ms")
        rospy.loginfo("=" * 60)
    
    def synchronized_callback(self, wrench_msg, kinetic_msg):
        """
        同步回调函数：同时接收力传感器和动捕数据
        只有当两个消息的时间戳在slop范围内时才会被调用
        """
        if not self.is_collecting:
            return
        
        # 提取力和力矩数据
        force = np.array([
            wrench_msg.wrench.force.x,
            wrench_msg.wrench.force.y,
            wrench_msg.wrench.force.z
        ])
        torque = np.array([
            wrench_msg.wrench.torque.x,
            wrench_msg.wrench.torque.y,
            wrench_msg.wrench.torque.z
        ])
        
        # 数据有效性检查
        if np.any(np.isnan(force)) or np.any(np.isnan(torque)):
            rospy.logwarn_throttle(2.0, "Received invalid force/torque data (NaN values)")
            return
        
        # 提取四元数
        current_quat = np.array([
            kinetic_msg.pose.orientation.x,
            kinetic_msg.pose.orientation.y,
            kinetic_msg.pose.orientation.z,
            kinetic_msg.pose.orientation.w
        ])
        
        # 提取速度和加速度
        linear_v = np.array([
            kinetic_msg.twist.linear.x,
            kinetic_msg.twist.linear.y,
            kinetic_msg.twist.linear.z
        ])
        angular_v = np.array([
            kinetic_msg.twist.angular.x,
            kinetic_msg.twist.angular.y,
            kinetic_msg.twist.angular.z
        ])
        linear_a = np.array([
            kinetic_msg.accel.linear.x,
            kinetic_msg.accel.linear.y,
            kinetic_msg.accel.linear.z
        ])
        angular_a = np.array([
            kinetic_msg.accel.angular.x,
            kinetic_msg.accel.angular.y,
            kinetic_msg.accel.angular.z
        ])
        
        # 静态检测
        lin_v_mag = np.linalg.norm(linear_v)
        ang_v_mag = np.linalg.norm(angular_v)
        lin_a_mag = np.linalg.norm(linear_a)
        ang_a_mag = np.linalg.norm(angular_a)
        
        if lin_v_mag > self.MAX_LINEAR_V or ang_v_mag > self.MAX_ANGULAR_V:
            rospy.logwarn_throttle(1.0,
                f"Tool moving too fast: v_lin={lin_v_mag:.4f} m/s, v_ang={ang_v_mag:.4f} rad/s")
            return
        
        if lin_a_mag > self.MAX_LINEAR_A or ang_a_mag > self.MAX_ANGULAR_A:
            rospy.logwarn_throttle(1.0,
                f"Tool acceleration too high: a_lin={lin_a_mag:.4f} m/s², a_ang={ang_a_mag:.4f} rad/s²")
            return
        
        # 检查姿态变化
        dot_product = np.abs(np.dot(self.last_quat, current_quat))
        should_record = (dot_product < self.quat_dot_threshold) or (len(self.force_list) == 0)
        
        if should_record:
            # 转换四元数到力传感器坐标系
            quat_in_ft_frame = self.trans_quat2ft_frame(current_quat)
            
            # 记录数据
            self.force_list.append(force)
            self.torque_list.append(torque)
            self.quat_list.append(quat_in_ft_frame)
            
            self.last_quat = current_quat
            
            rospy.loginfo(f"✓ Sample {len(self.force_list)}/{self.required_samples} collected "
                         f"(pose change: {np.arccos(min(dot_product, 1.0))*180/np.pi:.1f}°)")
            
            if len(self.force_list) >= self.required_samples:
                self.is_collecting = False
                self.calibrate_and_output()
                rospy.signal_shutdown("Calibration complete.")



    def trans_quat2ft_frame(self, quat):
        """
        将四元数列表从动捕坐标系转换到力传感器坐标系
        假设已知动捕坐标系到力传感器坐标系的固定变换关系
        """

        # R_body_sensor = tf_trans.rotation_matrix(np.pi, [0, 1, 0])  # 绕Y轴旋转180度
        R_body_sensor = tf_trans.rotation_matrix(0, [0, 0, 1]) # 单位矩阵

        M_world_body = tf_trans.quaternion_matrix(quat)
        M_world_sensor = np.dot(M_world_body, R_body_sensor)
        
        quat_wf = tf_trans.quaternion_from_matrix(M_world_sensor)

        return quat_wf

    def calibrate_and_output(self):
        """
        调用 tare_netft 进行标定, 并输出结果
        """

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Data collection complete ({self.required_samples} samples)")
        rospy.loginfo("Starting calibration calculation...")
        rospy.loginfo("=" * 60)

        try: 
            mass, static_force_offset, center_of_mass, static_torque_offset = tare_netft(
                self.force_list, 
                self.torque_list, 
                self.quat_list
            )

            if isinstance(static_force_offset, np.ndarray) and static_force_offset.ndim > 1:
                static_force_offset = np.mean(static_force_offset, axis=1)
            
            if isinstance(static_torque_offset, np.ndarray) and static_torque_offset.ndim > 1:
                static_torque_offset = np.mean(static_torque_offset, axis=1)

            decimals = 5
            mass_save = round(float(mass), decimals)
            force_save = np.round(static_force_offset, decimals).tolist()
            torque_save = np.round(static_torque_offset, decimals).tolist()
            com_save = np.round(center_of_mass.flatten(), decimals).tolist()

            results_data = {
                'description': 'Tool gravity calibration results based on motion capture data.',
                'mass_kg': mass_save,
                'static_force_offset_N': force_save,
                'center_of_mass_m': com_save,
                'static_torque_offset_Nm': torque_save,
                'coordinate_system_note': 'All values in force/torque sensor coordinate frame',
                'quaternion_format': 'xyzw'
                }
            
            # 输出结果
            rospy.loginfo("=" * 60)
            rospy.loginfo("CALIBRATION RESULTS")
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"  Mass (kg):                 {mass_save}")
            rospy.loginfo(f"  Static Force Offset (N):   {format_list(force_save)}")
            rospy.loginfo(f"  Center of Mass (m):        {format_list(com_save)}")
            rospy.loginfo(f"  Static Torque Offset (Nm): {format_list(torque_save)}")
            rospy.loginfo("=" * 60)
            
            # 输出一些有用的诊断信息
            self.print_diagnostics()
            
            # 保存结果
            self.save_yaml_results(results_data)        
        
        except Exception as e:
            rospy.logerr("=" * 60)
            rospy.logerr(f"CALIBRATION FAILED: {e}")
            rospy.logerr("=" * 60)
            import traceback
            rospy.logerr(traceback.format_exc())

    def print_diagnostics(self):
        """
        打印诊断信息，帮助判断数据质量
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("DATA DIAGNOSTICS")
        rospy.loginfo("=" * 60)
        
        force_array = np.array(self.force_list)
        torque_array = np.array(self.torque_list)
        
        rospy.loginfo(f"Force range:")
        rospy.loginfo(f"  X: [{np.min(force_array[:, 0]):.2f}, {np.max(force_array[:, 0]):.2f}] N")
        rospy.loginfo(f"  Y: [{np.min(force_array[:, 1]):.2f}, {np.max(force_array[:, 1]):.2f}] N")
        rospy.loginfo(f"  Z: [{np.min(force_array[:, 2]):.2f}, {np.max(force_array[:, 2]):.2f}] N")
        
        rospy.loginfo(f"Torque range:")
        rospy.loginfo(f"  X: [{np.min(torque_array[:, 0]):.2f}, {np.max(torque_array[:, 0]):.2f}] Nm")
        rospy.loginfo(f"  Y: [{np.min(torque_array[:, 1]):.2f}, {np.max(torque_array[:, 1]):.2f}] Nm")
        rospy.loginfo(f"  Z: [{np.min(torque_array[:, 2]):.2f}, {np.max(torque_array[:, 2]):.2f}] Nm")
        
        # 检查姿态多样性
        quat_array = np.array(self.quat_list)
        min_dot = 1.0
        for i in range(len(quat_array)):
            for j in range(i+1, len(quat_array)):
                dot = abs(np.dot(quat_array[i], quat_array[j]))
                min_dot = min(min_dot, dot)
        
        max_rotation = np.arccos(min_dot) * 180 / np.pi
        rospy.loginfo(f"Maximum rotation between any two poses: {max_rotation:.1f}°")
        
        if max_rotation < 30:
            rospy.logwarn("Warning: Pose diversity might be insufficient. Try larger rotations.")
        
        rospy.loginfo("=" * 60)

    def save_yaml_results(self, data):
        """
        将标定结果保存到yaml文件
        """
        try:
            parent_dir = get_parent_dir()
            file_name = "capture_config.yaml"
            output_path = os.path.join(parent_dir, "config", file_name)

            with open(output_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)
            
            rospy.loginfo(f"Calibration results saved to {output_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save calibration results: {e}")
            
def format_list(lst, precision=4):
    return "[" + ", ".join(f"{x:.{precision}f}" for x in lst) + "]"

if __name__ == "__main__":
    try:
        # 可以从命令行参数或 rosparam 读取配置
        required_samples = rospy.get_param('~required_samples', 30)
        quat_dot_threshold = rospy.get_param('~quat_dot_threshold', 0.95)
        
        tare_node = ToolGravityTare(
            required_samples=required_samples,
            quat_dot_threshold=quat_dot_threshold
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())