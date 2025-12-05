#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Point, Vector3
from visualization_msgs.msg import Marker
from il_capture.msg import KineticStateStamped
import tf.transformations as tf_trans
from gravity_compensator import GravityCompensator
import os
import sys

def get_parent_dir():
    script_path = os.path.realpath(sys.argv[0])
    parent_dir = os.path.dirname(os.path.dirname(script_path))
    return parent_dir

class ForceVisualizer:
    def __init__(self):
        rospy.init_node('force_visualizer', anonymous=True)

        # 参数：工具末端在传感器坐标系下的Z轴偏移
        self.tool_tip_offset_z = rospy.get_param('~tool_tip_offset_z', 0.1715)

        # 加载重力补偿配置
        script_dir = get_parent_dir()
        config_path = os.path.join(os.path.dirname(script_dir), 'config', 'gravity_compensation.yaml')
        self.gravity_compensator = GravityCompensator(config_path)

        # 缓存
        self.latest_kinetic = None

        # 发布 Marker
        self.marker_pub = rospy.Publisher('/visualization/tool_force', Marker, queue_size=1)
        self.tip_pub = rospy.Publisher('/visualization/tool_tip', Marker, queue_size=1)

        # 订阅
        self.force_sub = rospy.Subscriber('/netft_data', WrenchStamped, self.force_callback)
        self.kinetic_sub = rospy.Subscriber('/mimic_tool/kinetic_state', KineticStateStamped, self.kinetic_callback)

        self.R_body_sensor = tf_trans.rotation_matrix(np.pi, [0, 1, 0])[:3, :3]

    def kinetic_callback(self, msg: KineticStateStamped):
        """
        处理动捕点数据
        """
        self.latest_kinetic = msg

    def force_callback(self, msg: WrenchStamped):
        """
        处理力传感器数据
        """
        if self.latest_kinetic is None:
            rospy.logwarn("No kinetic data available yet.")
            return
        
        # 获取工具末端姿态
        raw_f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        raw_t = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        
        pose = self.latest_kinetic.pose
        quat = np.array([pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w])
        pose_body = np.array([pose.position.x, pose.position.y, pose.position.z])

        # 补偿重力
        comp_f, comp_t = self.gravity_compensator.get_compensated_wrench(
            raw_f, raw_t, quat)

        # 计算工具末端的世界坐标
        R_world_body = tf_trans.quaternion_matrix(quat)[:3, :3]
        R_world_sensor = np.dot(R_world_body, self.R_body_sensor)

        offset_vec_sensor = np.array([0, 0, self.tool_tip_offset_z])
        tip_pos_world = pose_body + np.dot(R_world_sensor, offset_vec_sensor)
        
        # 接触力转换到世界坐标系
        f_world = np.dot(R_world_sensor, comp_f)

        self.publish_arrow(tip_pos_world, f_world)
        self.publish_sphere(tip_pos_world)

    def publish_arrow(self, start_pos, vec):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tool_force"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 设置起点和终点
        p_start = Point(*start_pos)
        # 放大显示的力以方便观察
        scale_factor = 0.05
        p_end = Point(*(start_pos + vec * scale_factor))
        marker.points = [p_start, p_end]

        # 设置外观
        marker.scale = Vector3(0.01, 0.02, 0.0)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
    
    def publish_sphere(self, pos):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tool_tip"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 设置位置
        marker.pose.position = Point(*pos)
        marker.pose.orientation.w = 1.0

        # 设置外观
        marker.scale = Vector3(0.02, 0.02, 0.02)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.tip_pub.publish(marker)

if __name__ == '__main__':
    try:
        ForceVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")