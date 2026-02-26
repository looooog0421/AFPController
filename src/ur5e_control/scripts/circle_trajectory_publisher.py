#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆形轨迹发布器
在y-z平面上绘制圆形轨迹，起点为[-0.3, -0.3, 0.4]
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time


class CircleTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('circle_trajectory_publisher', anonymous=True)
        
        # 发布器
        self.traj_pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)
        
        # 轨迹参数
        self.start_point = np.array([-0.3, -0.3, 0.4])  # 起点
        self.radius = 0.1  # 圆的半径 (米)
        self.period = 10.0  # 完成一圈的时间 (秒)
        
        # 计算圆心 (在y-z平面上，x不变)
        # 假设圆心在起点的z轴上方
        self.center = self.start_point.copy()
        self.center[2] -= self.radius  # z坐标上移半径
        
        # 验证起点是否在圆上
        dist = np.linalg.norm(self.start_point[1:] - self.center[1:])
        if abs(dist - self.radius) > 0.001:
            rospy.logwarn(f"起点不在圆上！距离={dist:.4f}, 半径={self.radius:.4f}")
            rospy.logwarn(f"调整圆心或半径...")
            self.radius = dist
        
        # 计算起点对应的角度
        dy = self.start_point[1] - self.center[1]
        dz = self.start_point[2] - self.center[2]
        self.start_angle = np.arctan2(dz, dy)
        
        # 控制频率
        self.freq = 100.0
        self.rate = rospy.Rate(self.freq)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Circle Trajectory Publisher Initialized")
        rospy.loginfo(f"Start Point: {self.start_point}")
        rospy.loginfo(f"Circle Center: {self.center}")
        rospy.loginfo(f"Radius: {self.radius:.4f} m")
        rospy.loginfo(f"Period: {self.period} s")
        rospy.loginfo(f"Start Angle: {np.rad2deg(self.start_angle):.2f} deg")
        rospy.loginfo(f"Publishing Frequency: {self.freq} Hz")
        rospy.loginfo("=" * 60)
        
    def compute_circle_point(self, t):
        """计算给定时间t对应的圆上的点
        
        Args:
            t: 当前时间 (秒)
        
        Returns:
            position: [x, y, z] 位置
        """
        # 角速度
        omega = 2 * np.pi / self.period
        
        # 当前角度
        theta = self.start_angle + omega * t
        
        # 计算y-z平面上的坐标
        x = self.center[0]  # x保持不变
        y = self.center[1] + self.radius * np.cos(theta)
        z = self.center[2] + self.radius * np.sin(theta)
        
        return np.array([x, y, z])
    
    def create_pose_msg(self, position):
        """创建PoseStamped消息
        
        Args:
            position: [x, y, z] 位置
        
        Returns:
            PoseStamped消息
        """
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        # 姿态保持不变 (单位四元数)
        msg.pose.orientation.w = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = -1.0
        msg.pose.orientation.z = 0.0
        
        return msg
    
    def run(self):
        """运行轨迹发布循环"""
        rospy.loginfo("Starting circle trajectory publishing...")
        rospy.loginfo("Press Ctrl+C to stop")
        
        start_time = time.time()
        last_log_time = start_time
        loop_count = 0
        
        while not rospy.is_shutdown():
            current_time = time.time()
            t = current_time - start_time
            
            # 计算当前目标位置
            position = self.compute_circle_point(t)
            
            # 创建并发布消息
            msg = self.create_pose_msg(position)
            self.traj_pub.publish(msg)
            
            loop_count += 1
            
            # 每2秒打印一次状态
            if current_time - last_log_time >= 2.0:
                circle_progress = (t % self.period) / self.period * 100
                rospy.loginfo(f"Time: {t:.2f}s | Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] | Circle: {circle_progress:.1f}%")
                last_log_time = current_time
            
            # 维持频率
            self.rate.sleep()


if __name__ == '__main__':
    try:
        publisher = CircleTrajectoryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nShutting down circle trajectory publisher")
