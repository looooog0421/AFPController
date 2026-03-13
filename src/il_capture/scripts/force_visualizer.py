#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Point, Vector3
from visualization_msgs.msg import Marker
from il_capture.msg import KineticStateStamped
import tf.transformations as tf_trans
import tf
from gravity_compensator import GravityCompensator
import os
import sys

def get_parent_dir():
    script_path = os.path.realpath(sys.argv[0])
    parent_dir = os.path.dirname(script_path)
    return parent_dir

class ForceVisualizer:
    def __init__(self):
        rospy.init_node('force_visualizer', anonymous=True)

        # 参数：工具末端在传感器坐标系下的Z轴偏移
        self.tool_tip_offset_z = rospy.get_param('~tool_tip_offset_z', 0.1715)

        # 加载重力补偿配置
        script_dir = get_parent_dir()
        config_path = os.path.join(os.path.dirname(script_dir), 'config', 'capture_config.yaml')
        print("config_path: ", config_path)
        self.gravity_compensator = GravityCompensator(config_path)

        # 缓存
        self.latest_kinetic = None

        # 1. 发布 Marker (用于 RViz 3D 显示)
        self.marker_pub = rospy.Publisher('/visualization/tool_force', Marker, queue_size=1)
        self.tip_pub = rospy.Publisher('/visualization/tool_tip', Marker, queue_size=1)
        
        # 2. 发布 WrenchStamped (新增：用于 rqt_plot 曲线显示)
        # 话题名: /mimic_tool/compensated_wrench
        self.comp_wrench_pub = rospy.Publisher('/mimic_tool/compensated_wrench', WrenchStamped, queue_size=1)

        # TF 广播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 订阅
        self.force_sub = rospy.Subscriber('/netft_data', WrenchStamped, self.force_callback)
        self.kinetic_sub = rospy.Subscriber('/mimic_tool/kinetic_state', KineticStateStamped, self.kinetic_callback)

        # 刚体坐标系与传感器坐标系对齐 (0度)
        self.R_body_sensor = tf_trans.rotation_matrix(0, [0, 1, 0])[:3, :3]

    def kinetic_callback(self, msg: KineticStateStamped):
        self.latest_kinetic = msg

    def force_callback(self, msg: WrenchStamped):
        if self.latest_kinetic is None:
            return
        
        # --- A. 数据准备 ---
        raw_f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        raw_t = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        
        pose = self.latest_kinetic.pose
        try:
            frame_id = self.latest_kinetic.header.frame_id
        except AttributeError:
            try:
                frame_id = self.latest_kinetic.Header.frame_id 
            except AttributeError:
                frame_id = "world"
        if not frame_id: frame_id = "world"

        quat = np.array([pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w])
        pose_body = np.array([pose.position.x, pose.position.y, pose.position.z])

        # --- B. 发布 TF (便于 RViz 观察) ---
        self.tf_broadcaster.sendTransform(
            pose_body, quat, rospy.Time.now(), "mimic_tool_frame", frame_id
        )

        # --- C. 重力补偿计算 ---
        # comp_f, comp_t 是在传感器坐标系(也是刚体坐标系)下的值
        comp_f, comp_t = self.gravity_compensator.get_compensated_wrench(raw_f, raw_t, quat)

        # --- D. 发布 rqt 可视化数据 (新增) ---
        self.publish_compensated_wrench(comp_f, comp_t)

        # --- E. 发布 RViz Marker ---
        # 1. 计算可视化位置 (世界系)
        R_world_body = tf_trans.quaternion_matrix(quat)[:3, :3]
        R_world_sensor = np.dot(R_world_body, self.R_body_sensor)
        
        offset_vec_sensor = np.array([0, 0, self.tool_tip_offset_z])
        tip_pos_world = pose_body + np.dot(R_world_sensor, offset_vec_sensor)
        
        # 2. 将力旋转到世界系以便绘制箭头
        f_world = np.dot(R_world_sensor, comp_f)

        self.publish_arrow(tip_pos_world, f_world, frame_id)
        self.publish_sphere(tip_pos_world, frame_id)

    def publish_compensated_wrench(self, force, torque):
        """
        发布标准的 WrenchStamped 消息
        """
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        # 数据是相对于传感器/工具本身的，所以 frame_id 用工具坐标系
        msg.header.frame_id = "mimic_tool_frame" 
        
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        
        self.comp_wrench_pub.publish(msg)

    def publish_arrow(self, start_pos, vec, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tool_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        p_start = Point(*start_pos)
        # 放大力向量：0.01 表示 100N = 1m 长。如果你觉得箭头太短看不见，可以把这个改大，比如 0.05
        scale_factor = 0.02 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        p_end = Point(*(start_pos + vec * scale_factor))
        marker.points = [p_start, p_end]
        
        marker.scale = Vector3(0.005, 0.01, 0.02)
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
        self.marker_pub.publish(marker)
    
    def publish_sphere(self, pos, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tool_tip"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(*pos)
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.02, 0.02, 0.02)
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.8
        self.tip_pub.publish(marker)

if __name__ == '__main__':
    try:
        ForceVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")