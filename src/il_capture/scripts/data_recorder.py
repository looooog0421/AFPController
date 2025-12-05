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


def get_parent_dir():
    script_path = os.path.realpath(sys.argv[0])
    parent_dir = os.path.dirname(os.path.dirname(script_path))
    return parent_dir

class RecorderState(Enum):
    IDLE = 0 # 空闲状态
    RECORDING = 1 # 录制状态
    COOLDOWN = 2 # 冷却状态 记录结束后短暂冷却，防止立即重新开始录制

class DataRecorder:
    def __init__(self):
        rospy.init_node('data_recorder', anonymous=True)

        # 解析 YAML 配置文件
        self.parse_yaml_config(rospy.get_param('~config_path', os.path.join(get_parent_dir(), 'config', 'data_collect_param.yaml')))
        
        # 重力补偿
        config_path = os.path.join(get_parent_dir(), 'config', self.gravity_tare_file_name)
        self.gravity_compensator = GravityCompensator(config_path)

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
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            # self.output_dir = config['data_directory']
            self.VEL_STATIC_THRESHOLD = config['vel_static_threshold']
            self.FORCE_TRIGGER_THRESHOLD = config['force_trigger_threshold']
            self.DWELL_TIME_REQUIRED = config['dwell_time_required']
            self.MIN_RECORD_TIME = config['min_record_time']
            self.gravity_tare_file_name = config['gravity_tare_file_name']
            raw_dir = config['data_directory']
            if os.path.isabs(raw_dir):
                self.output_dir = raw_dir
            else:
                # 假设相对路径是相对于 il_capture 包的根目录
                self.output_dir = os.path.join(get_parent_dir(), raw_dir)

            # 确保目录存在
            if not os.path.exists(self.output_dir):
                os.makedirs(self.output_dir)
        except Exception as e:
            rospy.logerr(f"Failed to load YAML config: {e}")
            return {}

    def timer_callback(self, event):
        if self.latest_force is None or self.latest_kinetic_state is None:
            return
        
        # 1. 数据对齐与预处理
        pose = self.latest_kinetic_state.pose
        twist = self.latest_kinetic_state.twist
        raw_force = np.array([self.latest_force.wrench.force.x, 
                              self.latest_force.wrench.force.y, 
                              self.latest_force.wrench.force.z])
        raw_torque = np.array([self.latest_force.wrench.torque.x, 
                               self.latest_force.wrench.torque.y, 
                               self.latest_force.wrench.torque.z])
        tool_quat = np.array([pose.orientation.x, pose.orientation.y, 
                              pose.orientation.z, pose.orientation.w])
        
        # 计算接触力
        comp_force, comp_torque = self.gravity_compensator.get_compensated_wrench(raw_force, raw_torque, tool_quat)
        force_mag = np.linalg.norm(comp_force)

        # 检查是否静止
        vel_mag = np.linalg.norm(np.array([twist.linear.x, twist.linear.y, twist.linear.z]))
        is_static = vel_mag < self.VEL_STATIC_THRESHOLD

        now = rospy.Time.now().to_sec()
        
        # 2. 状态机逻辑

        if self.state == RecorderState.IDLE:
            # 触发条件： 静止且力超过阈值
            if is_static and force_mag > self.FORCE_TRIGGER_THRESHOLD:
                if self.dwell_start_time is None:
                    # 开始计时
                    self.dwell_start_time = now
                elif (now - self.dwell_start_time) > self.DWELL_TIME_REQUIRED:
                    # 达到静止时间要求，开始录制
                    self.start_recording(now)
            else:
                self.dwell_start_time = None # 重置计时器
        elif self.state == RecorderState.RECORDING:
            # 记录数据
            self.buffer_data(now, pose, twist, comp_force, comp_torque, raw_force, raw_torque)

            # 停止条件： 力低于阈值或达到最小录制时间
            if (now - self.record_start_time) > self.MIN_RECORD_TIME:
                if is_static:
                    if self.dwell_start_time is None:
                        self.dwell_start_time = now
                    elif (now - self.dwell_start_time) > self.DWELL_TIME_REQUIRED:
                        self.stop_recording_and_save()
                else:
                    self.dwell_start_time = None
        elif self.state == RecorderState.COOLDOWN:
            # 冷却状态，等待一段时间后回到IDLE
            if not is_static:
                self.state = RecorderState.IDLE
                self.dwell_start_time = None

    def start_recording(self, now):
        self.state = RecorderState.RECORDING
        self.record_start_time = now
        self.current_segment_data = []
        self.segment_count += 1
        self.dwell_start_time = None # 重置驻留时间
        rospy.loginfo(f"Started recording segment {self.segment_count}...")

    def stop_recording_and_save(self):
        rospy.loginfo(f"Stopping recording segment {self.segment_count}. Saving {len(self.current_segment_data)} points.")
        self.save2csv()
        self.state = RecorderState.COOLDOWN
        self.dwell_start_time = None

    def buffer_data(self, now, pose, twist, comp_force, comp_torque, raw_force, raw_torque):
        # 数据格式
        row = {
            'time': now - self.record_start_time,
            'pos_x': pose.position.x, 'pos_y': pose.position.y, 'pos_z': pose.position.z,
            'quat_x': pose.orientation.x, 'quat_y': pose.orientation.y, 'quat_z': pose.orientation.z, 'quat_w': pose.orientation.w,
            'vel_x': twist.linear.x, 'vel_y': twist.linear.y, 'vel_z': twist.linear.z,
            'ang_vel_x': twist.angular.x, 'ang_vel_y': twist.angular.y, 'ang_vel_z': twist.angular.z,
            'force_x': comp_force[0], 'force_y': comp_force[1], 'force_z': comp_force[2], # 接触力
            'torque_x': comp_torque[0], 'torque_y': comp_torque[1], 'torque_z': comp_torque[2],
            'raw_fx': raw_force[0], 'raw_fy': raw_force[1], 'raw_fz': raw_force[2] # 原始力备份
        }
        self.current_segment_data.append(row)

    def save2csv(self):
        if not self.current_segment_data:
            return
        
        filename = f"segment_{self.segment_count:03d}_{int(time.time())}.csv"
        filepath = os.path.join(self.output_dir, filename)

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