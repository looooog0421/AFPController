#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pinocchio as pin
import numpy as np
import os
import sys
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped 
from std_msgs.msg import Float32, String

try:
    sys.path.append('/home/hzk/AFPController/src/ur5e_control/src') 
    from ur5e_control.ur5e_controller import UR5eController
except ImportError:
    UR5eController = None

class ImpedanceFeederNode:
    def __init__(self):
        rospy.init_node("impedance_feeder_node")

        # =================== [核心参数配置] ===================
        # 1. 运动参数
        self.target_range = 0.04   
        self.hard_limit_dist = 0.08
        self.cruise_speed = 0.02   

        # 2. 阻抗参数
        self.target_force = 0.0     
        self.k_lift = 0.05          # 抬起灵敏度
        self.k_drop = 0.02          # 下压灵敏度
        self.gravity_drift = -0.001 # 微弱下沉

        # 3. 滤波
        self.filter_alpha = 0.1     
        self.filtered_force = 0.0

        # 4. 方向配置
        self.Y_AXIS_INVERT = -1.0   
        # ====================================================

        self.arm_urdf_path = "/home/hzk/AFPController/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"
        self.tcp_offset_xyz = np.array([0.0, 0.0, 0.0]) 
        self.roller_diameter = 0.10

        if not os.path.exists(self.arm_urdf_path): 
            rospy.logerr("URDF not found!")
            return
            
        self.model = pin.buildModelFromUrdf(self.arm_urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId("flange") if self.model.existFrame("flange") else self.model.nframes - 1

        self.q_curr = np.zeros(self.model.nq)
        self.force_curr = 0.0 
        self.force_bias = 0.0 
        self.bias_samples = []
        
        self.total_y_distance = 0.0 
        self.prev_real_pos = None
        self.start_tcp_pos = None    
        self.min_z_height = None 
        
        self.last_joint_update_time = rospy.Time.now()
        self.current_target_y = self.target_range 

        rospy.Subscriber("/joint_states", JointState, self.joint_cb)
        rospy.Subscriber("/netft_data", WrenchStamped, self.force_cb)
        
        self.feed_pub = rospy.Publisher("/afp/robot_feedforward_dist", Float32, queue_size=10)
        self.ur_script_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=1)

        rospy.loginfo(">>> 阻抗节点：逻辑已校正 <<<")
        self.rate = rospy.Rate(100)
        self.control_loop()

    def joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.q_curr = np.array(msg.position[:6])
            self.last_joint_update_time = rospy.Time.now()

    def force_cb(self, msg):
        raw_fz = msg.wrench.force.z
        if len(self.bias_samples) < 100: 
            self.bias_samples.append(raw_fz)
            if len(self.bias_samples) == 100:
                self.force_bias = sum(self.bias_samples) / len(self.bias_samples)
                rospy.loginfo("传感器零点校准完成")
            return
        real_fz = raw_fz - self.force_bias
        self.filtered_force = (self.filter_alpha * real_fz) + ((1.0 - self.filter_alpha) * self.filtered_force)
        self.force_curr = self.filtered_force

    def get_current_pose_and_rotation(self):
        pin.forwardKinematics(self.model, self.data, self.q_curr)
        pin.updateFramePlacements(self.model, self.data)
        flange_se3 = self.data.oMf[self.ee_frame_id]
        offset_se3 = pin.SE3(np.eye(3), self.tcp_offset_xyz)
        tcp_se3 = flange_se3 * offset_se3
        return tcp_se3.translation.copy(), tcp_se3.rotation.copy()

    def emergency_stop(self):
        self.ur_script_pub.publish(String(data="stopl(5.0)"))

    def control_loop(self):
        wait_count = 0
        while not rospy.is_shutdown() and len(self.bias_samples) < 100:
            wait_count += 1
            if wait_count > 500:
                rospy.logwarn("力传感器超时，进入【无力控模式】")
                self.force_bias = 0.0
                break
            self.rate.sleep()

        for i in range(50):
            self.start_tcp_pos, _ = self.get_current_pose_and_rotation()
            self.rate.sleep()
            
        self.min_z_height = self.start_tcp_pos[2] - 0.005
        self.prev_real_pos = self.start_tcp_pos.copy() 
        rospy.loginfo(f"系统就绪。虚拟地板: {self.min_z_height:.4f}m")

        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_joint_update_time).to_sec() > 0.2:
                pass

            current_tcp_pos, current_rot = self.get_current_pose_and_rotation()
            
            # ================= [逻辑A：绝对距离] =================
            delta_vector = current_tcp_pos - self.prev_real_pos
            self.prev_real_pos = current_tcp_pos.copy()

            delta_y = delta_vector[1] 
            spatial_increment = delta_y * 1.0 

            if abs(spatial_increment) < 0.00001: 
                spatial_increment = 0.0

            self.total_y_distance += spatial_increment
            
            # 强制广播
            msg = Float32()
            msg.data = self.total_y_distance
            self.feed_pub.publish(msg)

            # ================= [逻辑B：Y轴 往复] =================
            dist_y = current_tcp_pos[1] - self.start_tcp_pos[1]
            if abs(dist_y) > self.hard_limit_dist:
                rospy.logerr("超出范围急停")
                self.emergency_stop()
                return
            
            if self.current_target_y > 0 and dist_y > self.target_range:
                self.current_target_y = -self.target_range
            elif self.current_target_y < 0 and dist_y < -self.target_range:
                self.current_target_y = self.target_range
            
            nav_error = self.current_target_y - dist_y
            direction = np.sign(nav_error)
            
            if dist_y > self.target_range:
                vy_cmd = -abs(self.cruise_speed) * self.Y_AXIS_INVERT * 1.5
            elif dist_y < -self.target_range:
                vy_cmd = abs(self.cruise_speed) * self.Y_AXIS_INVERT * 1.5
            else:
                vy_cmd = direction * self.cruise_speed * self.Y_AXIS_INVERT

            # ================= [逻辑C：Z轴 阻抗控制] =================
            force_err = self.target_force - self.force_curr
            
            # 【核心恢复】 既然上推是负值，(0 - (-28)) = +28，我们需要正速度上升
            # 所以这里不需要加负号！保持正向逻辑！
            v_z = force_err * (self.k_lift if force_err > 0 else self.k_drop)
            v_z = np.clip(v_z, -0.08, 0.20)
            
            v_vec = current_rot.dot(np.array([0,0,v_z]))
            vz_final = v_vec[2] + self.gravity_drift 

            if current_tcp_pos[2] <= self.min_z_height and vz_final < 0:
                vz_final = 0.0

            if vz_final > 0.015: 
                vy_cmd = 0.0       
            
            cmd_str = f"speedl([0,{vy_cmd:.4f},{vz_final:.4f},0,0,0], 3.0, 0.02)"
            self.ur_script_pub.publish(String(data=cmd_str))

            if abs(spatial_increment) > 0.00001:
                rospy.loginfo_throttle(0.2, f"Total:{self.total_y_distance*1000:.1f}mm | Force:{self.force_curr:.1f}N")
            
            self.rate.sleep()

if __name__ == "__main__":
    try:
        ImpedanceFeederNode()
    except rospy.ROSInterruptException:
        pass