#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AFP 同步送带控制节点 (基于 TF/正运动学 位置映射)
架构说明：
    1. 依赖 robot_state_publisher 提供 /tf 变换
    2. 计算 TCP 实际空间位移
    3. 映射为电机目标角度
"""
import rospy
import tf2_ros
import math
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import JointState

class AFPFeedforwardKinematics:
    def __init__(self,
                 roller_diameter:float,
                 reduction_ratio:float,
                 feedforward_gain:float=1.0):
        self.roller_radius = roller_diameter / 2.0
        self.roller_circumference  = math.pi * roller_diameter
        self.reduction_ratio = reduction_ratio
        self.feedforward_gain = feedforward_gain

        self.accumulated_angle = 0.0

    def calculate_distance(self, p1: tuple, p2: tuple) -> float:
        """计算两点之间的欧氏距离"""
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)

    def update_target_angle(self, delta_distance: float) -> float:
        """
        根据位移增量计算累计目标电机角度
        注意：此处计算的是放料卷的角度增量
        """
        delta_roller_angle = (delta_distance / self.roller_circumference) * 360.0  # 转换为滚轮角度

        delta_angle = delta_roller_angle * self.feedforward_gain  # 应用前馈增益

        self.accumulated_angle += delta_angle
        return self.accumulated_angle
    
class AFPMotorSyncNode:
    def __init__(self):
        rospy.init_node('afp_motor_sync_node')

        # 参数配置
        self.base_frame = rospy.get_param('~base_frame', 'base')
        self.tcp_frame = rospy.get_param('~tcp_frame', 'tool0_controller')
        self.update_rate = rospy.get_param('~update_rate', 200.0)  # Hz
        
        roller_dia = rospy.get_param('~roller_diameter', 0.057)  # 米
        gear_ratio = rospy.get_param('~reduction_ratio', 36.0)
        ff_gain = rospy.get_param('~feedforward_gain', 1.0)

        self.kinematics = AFPFeedforwardKinematics(roller_diameter=roller_dia,
                                                  reduction_ratio=gear_ratio,
                                                  feedforward_gain=ff_gain)
    
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.angle_pub = rospy.Publisher('/afp/motor_target_angle', Float32, queue_size=10)

        # 可优化方向：订阅速度指令，实现指令前馈+状态反馈双环控制
        # rospy.Subscriber("/joint_group_vel_controller/command", Float64MultiArray, self.vel_cmd_cb)

        self.last_position = None
        self.latest_vel_cmd = None

        rospy.loginfo(f"AFP 同步节点启动. 监听 TF: {self.base_frame} -> {self.tcp_frame}")

        # --- 5. 启动主循环定时器 ---
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.control_loop)

    def vel_cmd_cb(self, msg: Float64MultiArray):
        """接收速度指令 (可选)"""
        self.latest_vel_cmd = msg.data

    def get_current_tcp_position(self):
        """获取当前 TCP 在基座坐标系下的位置"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, rospy.Time(0), rospy.Duration(0.01)
            )
            pos = trans.transform.translation

            return (pos.x, pos.y, pos.z)
        except (tf2_ros.LookupTransformAction, tf2_ros.ConvertRegistration) as e:
            rospy.logwarn_throttle(2.0, f"TF 变换获取失败: {e}")
            return None
        
    def control_loop(self, event):

        current_position = self.get_current_tcp_position()

        # 设置初始位置
        if current_position is None:
            return
        
        if self.last_position is None:
            self.last_position = current_position
            return
        
        # 计算笛卡尔空间位移
        delta_dist = self.kinematics.calculate_distance(self.last_position, current_position)

        if delta_dist > 0.0002:
            target_angle = self.kinematics.update_target_angle(delta_dist)

             # 发布目标角度
            self.angle_pub.publish(Float32(data=target_angle))

            # 打印调试信息
            rospy.loginfo_throttle(0.5, 
                f"TCP 增量: {delta_dist*1000:.3f} mm | 累计电机指令: {target_angle:.2f}°"
            )
        
            self.last_position = current_position
    

if __name__ == "__main__":
    try:
        node = AFPMotorSyncNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
