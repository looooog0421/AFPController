#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import sys
import csv
import time
from enum import Enum
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, AccelStamped
from il_capture.msg import KineticStateStamped
from gravity_compensator import GravityCompensator

def quat2rotmatrix(q):
    """
    x,y,z,w = q
    return 3x3 matrix
    """
    x, y, z, w = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])
def rotmatrix2quat(R):
    """
    将 3x3 旋转矩阵转换为四元数 [x, y, z, w]
    算法保证数值稳定性
    """
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return np.array([qx, qy, qz, qw])

def get_parent_dir():
    script_path = os.path.realpath(sys.argv[0])
    parent_dir = os.path.dirname(os.path.dirname(script_path))
    return parent_dir

def print_banner(text_type):
    if text_type == "START":
        # 绿色 (\033[92m)
        print("\033[92m")
        print(r"""
  ____ _____  _    ____  _____ 
 / ___|_   _|/ \  |  _ \|_   _|
 \___ \ | | / _ \ | |_) | | |  
  ___) || |/ ___ \|  _ <  | |  
 |____/ |_/_/   \_\_| \_\ |_|  
        """)
        print(">>> RECORDING STARTED <<<")
        print("\033[0m")
        
    elif text_type == "STOP":
        # 红色 (\033[91m)
        print("\033[91m")
        print(r"""
  ____ _____ ___  ____  
 / ___|_   _/ _ \|  _ \ 
 \___ \ | || | | | |_) |
  ___) || || |_| |  __/ 
 |____/ |_| \___/|_|    
        """)
        print(">>> RECORDING STOPPED & SAVED <<<")
        print("\033[0m")

class RecorderState(Enum):
    IDLE = 0 # 空闲状态
    RECORDING = 1 # 录制状态
    COOLDOWN = 2 # 冷却状态 记录结束后短暂冷却，防止立即重新开始录制

class DataRecorder:
    def __init__(self):
        rospy.loginfo("=======================================create datarecorder=================================")
        rospy.init_node('data_recorder', anonymous=True)

        # 1. 解析 YAML
        config_file_path = rospy.get_param('~config_path', os.path.join(get_parent_dir(), 'config', 'data_collect_param.yaml'))
        rospy.loginfo(f"Loading config from: {config_file_path}") # 打印一下路径，确认参数传进来了
        self.parse_yaml_config(config_file_path) # 这一步如果失败会直接退出

        # 2. 设置数据保存目录
        try:
            self.output_dir = rospy.get_param('~data_directory', 'data') # 默认相对路径
        except KeyError:
            rospy.logerr("Parameter 'data_directory' not found in ROS parameters.")
            self.output_dir = os.path.join(get_parent_dir(), 'data')
        rospy.loginfo(f"Data will be saved to: {self.output_dir}")

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"Created output directory at: {self.output_dir}")    


        
        # 重力补偿
        tare_config_path = os.path.join(get_parent_dir(), 'config', self.gravity_tare_file_name)
        if not os.path.exists(tare_config_path):
             rospy.logwarn(f"Gravity tare file not found at {tare_config_path}. Compensator will be uncalibrated.")
             
        self.gravity_compensator = GravityCompensator(tare_config_path)

        # 力传感器到工具末端的变换
        self.T_sensor_ee = np.eye(4)
        if hasattr(self, 'tool_length') and self.tool_length > 0:
            self.T_sensor_ee[2, 3] = self.tool_length
        
        if hasattr(self, 'cfg_R_sensor_ee'):
            self.T_sensor_ee = np.array(self.cfg_T_sensor_ee).reshape(4, 4)

        # 状态变量
        self.state = RecorderState.IDLE
        self.current_segment_data = [] # 当前录制段的数据
        self.dwell_start_time = None
        self.record_start_time = None
        self.segment_count = 0

        self.latest_force = None
        self.latest_kinetic_state = None

        # 订阅话题
        rospy.Subscriber('/netft_data', WrenchStamped, self.force_callback)
        rospy.Subscriber("/mimic_tool/kinetic_state", KineticStateStamped, self.kinetic_callback)

        # 定时器
        rospy.Timer(rospy.Duration(0.01), self.timer_callback) # 100Hz

        rospy.loginfo(f"Data Recorder Ready. Save path: {self.output_dir}")
        rospy.loginfo(f"Trigger: Force > {self.FORCE_TRIGGER_THRESHOLD}N & Static for {self.DWELL_TIME_REQUIRED}s")

    def force_callback(self, msg: WrenchStamped):
        self.latest_force = msg
    
    def kinetic_callback(self, msg: KineticStateStamped):
        self.latest_kinetic_state = msg

    def parse_yaml_config(self, config_path):
        import yaml
        if not os.path.exists(config_path):
            rospy.logerr(f"Config file not found: {config_path}")
            sys.exit(1) # 如果配置没找到，直接退出，不要硬撑
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            # self.output_dir = config['data_directory']
            self.VEL_STATIC_THRESHOLD = config['record']['vel_static_threshold']
            self.FORCE_TRIGGER_THRESHOLD = config['record']['force_trigger_threshold']
            self.DWELL_TIME_REQUIRED = config['record']['dwell_time_required']
            self.MIN_RECORD_TIME = config['record']['min_record_time']
            self.gravity_tare_file_name = config.get('gravity_tare_file_name', 'gravity_tare.yaml')

            self.tool_length = config['tool']['length']
            if 'transform' in config['tool']:
                 self.cfg_T_sensor_ee = config['tool']['transform']
            
        except Exception as e:
            rospy.logerr(f"Failed to load YAML config: {e}")
            sys.exit(1) # 失败直接退出

    def timer_callback(self, event):
        if self.latest_force is None or self.latest_kinetic_state is None:
            return
        
        # 1. 数据对齐与预处理
        sensor_pos = np.array([self.latest_kinetic_state.pose.position.x,
                               self.latest_kinetic_state.pose.position.y,
                               self.latest_kinetic_state.pose.position.z])
        sensor_quat = np.array([self.latest_kinetic_state.pose.orientation.x,
                                self.latest_kinetic_state.pose.orientation.y,
                                self.latest_kinetic_state.pose.orientation.z,
                                self.latest_kinetic_state.pose.orientation.w])
        raw_force = np.array([self.latest_force.wrench.force.x, 
                              self.latest_force.wrench.force.y, 
                              self.latest_force.wrench.force.z])
        raw_torque = np.array([self.latest_force.wrench.torque.x, 
                               self.latest_force.wrench.torque.y, 
                               self.latest_force.wrench.torque.z])
        

        # 重力补偿
        comp_force, comp_torque = self.gravity_compensator.get_compensated_wrench(raw_force, raw_torque, sensor_quat)
        
        # 计算末端位姿和受力
        R_world_sensor = quat2rotmatrix(sensor_quat)
        T_world_sensor = np.eye(4)
        T_world_sensor[:3, :3] = R_world_sensor
        T_world_sensor[:3, 3] = sensor_pos

        T_world_ee = np.dot(T_world_sensor, self.T_sensor_ee)

        ee_pos_world = T_world_ee[:3, 3]
        ee_rot_world = T_world_ee[:3, :3]

        R_sensor_ee = self.T_sensor_ee[:3, :3]
        r_sensor2ee = self.T_sensor_ee[:3, 3]

        ee_force = np.dot(R_sensor_ee.T, comp_force)
        torque_offset = np.cross(r_sensor2ee, comp_force)
        ee_torque = np.dot(R_sensor_ee.T, comp_torque - torque_offset)


        ee_force_mag = np.linalg.norm(ee_force)
        now = rospy.Time.now().to_sec()
        
        # 检查是否静止
        vel_mag = np.linalg.norm(np.array([self.latest_kinetic_state.twist.linear.x, 
                                           self.latest_kinetic_state.twist.linear.y, 
                                           self.latest_kinetic_state.twist.linear.z]))
        # print("Velocity magnitude:", vel_mag, "Force magnitude:", force_mag)
        is_static = vel_mag < self.VEL_STATIC_THRESHOLD

        
        
        # 2. 状态机逻辑

        START_FORCE = self.FORCE_TRIGGER_THRESHOLD
        STOP_FORCE = self.FORCE_TRIGGER_THRESHOLD * 0.4 # 带点滞后

        if self.state == RecorderState.IDLE:
            # 数据采集启动逻辑：只要力足够大立刻开始
            if ee_force_mag >= START_FORCE:
                rospy.loginfo(f"Force trigger exceeded: {ee_force_mag:.2f}N >= {START_FORCE}N")
                self.start_recording(now)
        elif self.state == RecorderState.RECORDING:
            self.buffer_data(now, ee_pos_world, ee_rot_world, ee_force, ee_torque, raw_force, raw_torque)

            # 数据采集停止逻辑：力回落，且录制时间超过最小限制
            if (now - self.record_start_time) > self.MIN_RECORD_TIME:
                if ee_force_mag <= STOP_FORCE:
                    rospy.loginfo(f"Force dropped below stop threshold: {ee_force_mag:.2f}N <= {STOP_FORCE}N")
                    self.stop_recording_and_save()

        elif self.state == RecorderState.COOLDOWN:
            if (now - self.last_stop_time) >= 1.0: # 冷却1秒后回到空闲
                if ee_force_mag < START_FORCE:
                    self.state = RecorderState.IDLE
                    rospy.loginfo("Cooldown complete. Returning to IDLE state.")

    def start_recording(self, now):
        self.state = RecorderState.RECORDING
        self.record_start_time = now
        self.current_segment_data = []
        self.segment_count += 1
        self.dwell_start_time = None # 重置驻留时间
        print_banner("START")
        rospy.loginfo(f"Started recording segment {self.segment_count}...")

    def stop_recording_and_save(self):
        rospy.loginfo(f"Stopping recording segment {self.segment_count}. Saving {len(self.current_segment_data)} points.")
        self.save2csv()
        print_banner("STOP")
        self.state = RecorderState.COOLDOWN
        self.last_stop_time = rospy.Time.now().to_sec()  # <--- 加这一行
        self.dwell_start_time = None

    def buffer_data(self, now, ee_pos, ee_rot, ee_force, ee_torque, raw_force, raw_torque):
        # 将旋转矩阵转回四元数存起来，或者直接存矩阵元素，这里为了csv简洁通常存四元数
        # 这里为了简单，假设你不需要存 EE 的四元数，或者你自己实现 rot2quat
        # 暂时只存位置和力
        
        ee_quat = rotmatrix2quat(ee_rot)

        row = {
            'time': now - self.record_start_time,
            # --- 新增：末端(EE)状态 ---
            'ee_x': ee_pos[0], 'ee_y': ee_pos[1], 'ee_z': ee_pos[2],
            'ee_qx': ee_quat[0], 'ee_qy': ee_quat[1], 'ee_qz': ee_quat[2], 'ee_qw': ee_quat[3],
            'ee_fx': ee_force[0], 'ee_fy': ee_force[1], 'ee_fz': ee_force[2],
            'ee_tx': ee_torque[0], 'ee_ty': ee_torque[1], 'ee_tz': ee_torque[2],
            
            # 原始数据备份 (Sensor Frame)
            'raw_fx': raw_force[0], 'raw_fy': raw_force[1], 'raw_fz': raw_force[2],
            'raw_tx': raw_torque[0], 'raw_ty': raw_torque[1], 'raw_tz': raw_torque[2]
        }
        self.current_segment_data.append(row)

    def save2csv(self):
        if not self.current_segment_data:
            return
        
        # current time (year_month_days_hours_minutes_seconds)
        current_time = time.localtime()

        filename = f"segment_{self.segment_count:03d}.csv"
        filepath = os.path.join(self.output_dir, current_time, filename)
        if not os.path.exists(os.path.dirname(filepath)):
            os.makedirs(os.path.dirname(filepath))
        keys = self.current_segment_data[0].keys()
        try:
            with open(filepath, 'w', newline='') as output_file:
                dict_writer = csv.DictWriter(output_file, fieldnames=keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.current_segment_data)
            rospy.loginfo(f"Segment {self.segment_count} saved to {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to save segment {self.segment_count} to CSV: {e}")

if __name__ == '__main__':
    recorder = DataRecorder()
    rospy.spin()