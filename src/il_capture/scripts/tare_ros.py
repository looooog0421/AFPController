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
    def __init__(self, required_samples=10, quat_dot_threashold=0.9):
        # 初始化节点
        rospy.init_node('tool_gravity_tare', anonymous=True)
    
        # 参数
        self.required_samples = required_samples
        self.quat_dot_threashold = quat_dot_threashold

        # 标定数据列表
        self.force_list = []
        self.torque_list = []
        self.quat_list = []

        # 上次记录的姿态
        self.last_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 初始为单位四元数
        self.is_collecting = True

        # 缓存最新的传感器数据
        self.last_force = None
        self.latest_force = None
        self.latest_twist = None
        self.latest_accel = None

        # 订阅力传感器数据和动捕点数据话题
        rospy.Subscriber("/netft_data", WrenchStamped, self.wrench_callback, queue_size=1)
        
        # 订阅刚体姿态话题
        rospy.Subscriber("/mimic_tool/kinetic_state", KineticStateStamped, self.kinetic_callback, queue_size=1)

        self.MAX_LINEAR_V = 0.03
        self.MAX_ANGULAR_V = 0.08
        self.MAX_LINEAR_A = 0.3
        self.MAX_ANGULAR_A = 0.8
        
        rospy.loginfo("Tool Gravity Tare Node Initialized.")
        rospy.loginfo(f"Thresholds -> V: {self.MAX_LINEAR_V}, W: {self.MAX_ANGULAR_V}, A: {self.MAX_LINEAR_A}, Alpha: {self.MAX_ANGULAR_A}")
        rospy.loginfo("Waiting for data...")

    def wrench_callback(self, msg):
        """
        self.latest_force: 缓存最新的力传感器数据
        """
        # print("Received wrench message.")
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        self.latest_force = (force, torque)

    def kinetic_callback(self, msg):
        """
        处理刚体姿态消息, 进行数据采集
        """
        # print("Received kinetic state message.")
        self.latest_kinematic_state = msg

        if not self.is_collecting:
            return
        
        if self.latest_force is None:
            # rospy.logwarn("Waiting for force/torque and twist data...")
            return  # 尚未收到力传感器数据


        # 提取四元数
        current_quat_msg = msg.pose.orientation
        current_quat = np.array([
                    current_quat_msg.w,
                    current_quat_msg.x, 
                    current_quat_msg.y, 
                    current_quat_msg.z
                ])
        
        # 检查姿态变化是否足够大
        dot_product = np.abs(np.dot(self.last_quat, current_quat))
        should_record = (dot_product < self.quat_dot_threashold) or (len(self.force_list) == 0)

        # print(f"Quat dot product: {dot_product:.3f}, Should record: {should_record}")
        if should_record:
            # 提取数据
            # rospy.loginfo("Received pose message.")
            linear_v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            angular_v = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
            # self.latest_twist = (linear_v, angular_v)

            linear_a = np.array([msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z])
            angular_a = np.array([msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z])
            self.latest_accel = (linear_a, angular_a)        

            # 静态检测: 检查刚体速度是否足够小
            # linear_v, angular_v = self.latest_twist
            lin_v_mag = np.linalg.norm(linear_v)
            ang_v_mag = np.linalg.norm(angular_v)
            lin_a_mag = np.linalg.norm(linear_a)
            ang_a_mag = np.linalg.norm(angular_a)

            # 检查速度
            if lin_v_mag > self.MAX_LINEAR_V or ang_v_mag > self.MAX_ANGULAR_V:
                rospy.logwarn(f"Tool is moving too fast (linear_v: {lin_v_mag:.3f}, angular_v: {ang_v_mag:.3f}). Hold still for accurate tare.")
                return  # 工具移动过快, 不进行记录
            
            # 检查加速度
            if lin_a_mag > self.MAX_LINEAR_A or ang_a_mag > self.MAX_ANGULAR_A:
                rospy.logwarn(f"Tool acceleration too high (linear_a: {lin_a_mag:.3f}, angular_a: {ang_a_mag:.3f}). Hold still for accurate tare.")
                return  # 工具加速度过大, 不进行记录

            self.record_data(current_quat, dot_product)

            # 更新记录姿态
            self.last_quat = current_quat

            rospy.loginfo(f"Collected {len(self.force_list)}/{self.required_samples} samples.")
            if len(self.force_list) >= self.required_samples:
                # 采集完毕， 开始标定
                self.is_collecting = False
                self.quat_list = self.trans_quat_list(self.quat_list)
                self.calibrate_and_output()
                rospy.loginfo("Data collection and calibration complete.")

                # 关闭节点
                rospy.signal_shutdown("Calibration complete.")

    def record_data(self, quat, dot_product):
            """
            记录数据
            """
            force, torque = self.latest_force

            self.force_list.append(force)
            self.torque_list.append(torque)
            self.quat_list.append(quat)

            self.last_quat = quat

            # rospy.loginfo(f"Recorded sample {len(self.force_list)}/{self.required_samples} (quat dot: {dot_product:.3f})")

    def trans_quat2ft_frame(self, quat):
        """
        将四元数列表从动捕坐标系转换到力传感器坐标系
        假设已知动捕坐标系到力传感器坐标系的固定变换关系
        """

        R_body_sensor = tf_trans.rotation_matrix(np.pi, [0, 1, 0])  # 绕Y轴旋转180度
        # R_GF = np.diag([-1, 1, -1, 1])  # 这个坐标变换矩阵表示的是力传感器的坐标系与我建立的刚体之间的坐标变换

        quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]  # 转换为(x,y,z,w)顺序

        M_world_body = tf_trans.quaternion_matrix(quat_xyzw)

        M_world_sensor = np.dot(M_world_body, R_body_sensor)
        
        quat_wf = tf_trans.quaternion_from_matrix(M_world_sensor)

        return quat_wf

    def trans_quat_list(self, quat_list):
        """
        将四元数列表从动捕坐标系转换到力传感器坐标系
        """

        transformed_quat_list = []
        for quat in quat_list:
            transformed_quat = self.trans_quat2ft_frame(quat)
            transformed_quat_list.append(transformed_quat)
        
        return transformed_quat_list

    def calibrate_and_output(self):
        """
        调用 tare_netft 进行标定, 并输出结果
        """

        rospy.loginfo("================================")
        rospy.loginfo("=======数据采集完成(%d samples)=======", self.required_samples)
        rospy.loginfo("=======开始进行标定计算=========")

        try: 
            mass, static_force_offset, center_of_mass, static_torque_offset = tare_netft(self.force_list, self.torque_list, self.quat_list)

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
                    'coordinate_system_note': 'Rigid body pose was transformed by R_GF = R_Y(180) before calibration.'
                }
            
            # 4. 输出并保存结果
            rospy.loginfo("\n--- 工具重力标定结果 ---")
            rospy.loginfo(f"  - Mass (kg): {mass_save}")
            rospy.loginfo(f"  - Static Force Offset (N): {force_save}")
            rospy.loginfo(f"  - Center of Mass (m): {com_save}")
            rospy.loginfo(f"  - Static Torque Offset (Nm): {torque_save}")
            
            self.save_yaml_results(results_data)        
        
        except Exception as e:
            rospy.logerr(f"Calibration failed: {e}")

    def save_yaml_results(self, data):
        """
        将标定结果保存到yaml文件
        """
        try:
            parent_dir = get_parent_dir()

            file_name = "gravity_tare.yaml"
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
        ToolGravityTare()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    