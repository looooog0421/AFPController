#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS阻抗控制器测试节点

功能：
1. 发布参考轨迹到 /reference_trajectory
2. 订阅当前关节状态和力传感器数据
3. 支持多种测试场景：固定位置、圆形轨迹、XY平面运动
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray
import sys


class ImpedanceControllerTester:
    """阻抗控制器ROS测试节点"""
    
    def __init__(self):
        rospy.init_node('impedance_controller_tester', anonymous=False)
        
        # ============ 参数配置 ============
        self.test_mode = rospy.get_param('~test_mode', 'fixed_position')
        # 可选模式: 'fixed_position', 'circular', 'xy_motion', 'approach_contact'
        
        self.control_frequency = rospy.get_param('~control_frequency', 100.0)  # Hz
        
        # 固定位置模式参数
        self.target_position = rospy.get_param('~target_position', [0.5, 0.0, 0.4])
        self.target_orientation = rospy.get_param('~target_orientation', [1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        
        # 圆形轨迹参数
        self.circle_center = rospy.get_param('~circle_center', [0.5, 0.0, 0.4])
        self.circle_radius = rospy.get_param('~circle_radius', 0.05)  # 5cm
        self.circle_angular_vel = rospy.get_param('~circle_angular_vel', 0.5)  # rad/s
        
        # XY平面运动参数
        self.xy_start = rospy.get_param('~xy_start', [0.5, -0.1, 0.4])
        self.xy_end = rospy.get_param('~xy_end', [0.5, 0.1, 0.4])
        self.xy_duration = rospy.get_param('~xy_duration', 5.0)  # 秒
        
        # 接触测试参数
        self.approach_start_z = rospy.get_param('~approach_start_z', 0.5)
        self.approach_end_z = rospy.get_param('~approach_end_z', 0.41)
        self.approach_duration = rospy.get_param('~approach_duration', 3.0)
        
        # ============ 状态变量 ============
        self.current_joint_state = None
        self.current_wrench = None
        self.start_time = None
        
        # ============ ROS发布器 ============
        self.traj_pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)
        
        # 可选：动态调整阻抗参数
        self.impedance_pub = rospy.Publisher('/impedance_params_dynamic', Float32MultiArray, queue_size=1)
        
        # ============ ROS订阅器 ============
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        rospy.Subscriber('/netft_data', WrenchStamped, self.wrench_callback, queue_size=1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Impedance Controller Tester Initialized")
        rospy.loginfo(f"Test Mode: {self.test_mode}")
        rospy.loginfo(f"Control Frequency: {self.control_frequency} Hz")
        rospy.loginfo("=" * 60)
        
        # 等待订阅器连接
        rospy.sleep(0.5)
        
    def joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_state = msg
        
    def wrench_callback(self, msg):
        """力/力矩传感器回调"""
        self.current_wrench = msg
    
    def publish_impedance_params(self, position_stiffness, position_damping, 
                                 orientation_stiffness, orientation_damping):
        """发布阻抗参数"""
        msg = Float32MultiArray()
        msg.data = list(position_stiffness) + list(position_damping) + \
                   list(orientation_stiffness) + list(orientation_damping)
        self.impedance_pub.publish(msg)
        rospy.loginfo(f"Published impedance params: kp={position_stiffness}, dp={position_damping}")
    
    def create_pose_msg(self, position, orientation):
        """创建PoseStamped消息"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        msg.pose.orientation.w = orientation[0]
        msg.pose.orientation.x = orientation[1]
        msg.pose.orientation.y = orientation[2]
        msg.pose.orientation.z = orientation[3]
        
        return msg
    
    def smooth_trajectory(self, t, t_total, start, end):
        """五次多项式平滑轨迹"""
        if t >= t_total:
            return np.array(end)
        tau = t / t_total
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        return np.array(start) + s * (np.array(end) - np.array(start))
    
    def test_fixed_position(self):
        """测试1: 固定位置保持"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Test Mode: Fixed Position Control")
        rospy.loginfo(f"Target Position: {self.target_position}")
        rospy.loginfo(f"Target Orientation: {self.target_orientation}")
        rospy.loginfo("="*60)
        
        # 设置阻抗参数（标准刚度）
        self.publish_impedance_params(
            position_stiffness=[500, 500, 500],
            position_damping=[50, 50, 50],
            orientation_stiffness=[50, 50, 50],
            orientation_damping=[10, 10, 10]
        )
        
        rospy.sleep(0.5)
        
        rate = rospy.Rate(self.control_frequency)
        duration = 10.0  # 保持10秒
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed > duration:
                rospy.loginfo("Fixed position test completed!")
                break
            
            # 发布固定目标位姿
            msg = self.create_pose_msg(self.target_position, self.target_orientation)
            self.traj_pub.publish(msg)
            
            # 打印状态（每秒一次）
            if int(elapsed) != int(elapsed - 1.0/self.control_frequency):
                if self.current_wrench:
                    force_z = self.current_wrench.wrench.force.z
                    rospy.loginfo(f"Time: {elapsed:.1f}s, Force Z: {force_z:.2f}N")
            
            rate.sleep()
    
    def test_circular_trajectory(self):
        """测试2: 圆形轨迹跟踪"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Test Mode: Circular Trajectory")
        rospy.loginfo(f"Center: {self.circle_center}, Radius: {self.circle_radius}m")
        rospy.loginfo(f"Angular Velocity: {self.circle_angular_vel} rad/s")
        rospy.loginfo("="*60)
        
        # 设置阻抗参数
        self.publish_impedance_params(
            position_stiffness=[500, 500, 500],
            position_damping=[50, 50, 50],
            orientation_stiffness=[50, 50, 50],
            orientation_damping=[10, 10, 10]
        )
        
        rospy.sleep(0.5)
        
        rate = rospy.Rate(self.control_frequency)
        duration = 20.0  # 运行20秒
        start_time = rospy.Time.now()
        
        center = np.array(self.circle_center)
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed > duration:
                rospy.loginfo("Circular trajectory test completed!")
                break
            
            # 计算圆形轨迹上的点
            angle = self.circle_angular_vel * elapsed
            x = center[0] + self.circle_radius * np.cos(angle)
            y = center[1] + self.circle_radius * np.sin(angle)
            z = center[2]
            
            position = [x, y, z]
            msg = self.create_pose_msg(position, self.target_orientation)
            self.traj_pub.publish(msg)
            
            # 打印状态
            if int(elapsed * 2) % 2 == 0 and int(elapsed * 2) != int((elapsed - 1.0/self.control_frequency) * 2):
                rospy.loginfo(f"Time: {elapsed:.1f}s, Angle: {np.degrees(angle):.1f}°, Pos: [{x:.3f}, {y:.3f}, {z:.3f}]")
            
            rate.sleep()
    
    def test_xy_motion(self):
        """测试3: XY平面直线运动"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Test Mode: XY Plane Motion")
        rospy.loginfo(f"Start: {self.xy_start}")
        rospy.loginfo(f"End: {self.xy_end}")
        rospy.loginfo(f"Duration: {self.xy_duration}s")
        rospy.loginfo("="*60)
        
        # 设置阻抗参数（Z轴更柔顺）
        self.publish_impedance_params(
            position_stiffness=[500, 500, 50],
            position_damping=[50, 50, 15],
            orientation_stiffness=[50, 50, 50],
            orientation_damping=[10, 10, 10]
        )
        
        rospy.sleep(0.5)
        
        rate = rospy.Rate(self.control_frequency)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed > self.xy_duration + 2.0:  # 额外保持2秒
                rospy.loginfo("XY motion test completed!")
                break
            
            # 计算当前目标位置
            if elapsed < self.xy_duration:
                position = self.smooth_trajectory(elapsed, self.xy_duration, 
                                                 self.xy_start, self.xy_end)
            else:
                position = self.xy_end
            
            msg = self.create_pose_msg(position, self.target_orientation)
            self.traj_pub.publish(msg)
            
            # 打印状态
            if int(elapsed * 4) % 4 == 0 and int(elapsed * 4) != int((elapsed - 1.0/self.control_frequency) * 4):
                rospy.loginfo(f"Time: {elapsed:.1f}s, Pos: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
                if self.current_wrench:
                    force_z = self.current_wrench.wrench.force.z
                    rospy.loginfo(f"  Force Z: {force_z:.2f}N")
            
            rate.sleep()
    
    def test_approach_contact(self):
        """测试4: 接近接触表面"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Test Mode: Approach Contact Surface")
        rospy.loginfo(f"Start Z: {self.approach_start_z}m")
        rospy.loginfo(f"End Z: {self.approach_end_z}m")
        rospy.loginfo(f"Duration: {self.approach_duration}s")
        rospy.loginfo("="*60)
        
        # 第一阶段：标准刚度接近
        rospy.loginfo("\nPhase 1: Approaching...")
        self.publish_impedance_params(
            position_stiffness=[500, 500, 500],
            position_damping=[50, 50, 50],
            orientation_stiffness=[50, 50, 50],
            orientation_damping=[10, 10, 10]
        )
        
        rospy.sleep(0.5)
        
        rate = rospy.Rate(self.control_frequency)
        start_time = rospy.Time.now()
        
        base_position = [self.target_position[0], self.target_position[1], 0]
        contact_detected = False
        contact_threshold = -1.0  # N
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            # 第一阶段：下降
            if elapsed < self.approach_duration:
                z = self.approach_start_z - (self.approach_start_z - self.approach_end_z) * \
                    (10*(elapsed/self.approach_duration)**3 - 15*(elapsed/self.approach_duration)**4 + 6*(elapsed/self.approach_duration)**5)
                position = [base_position[0], base_position[1], z]
                
                # 检测接触
                if self.current_wrench and not contact_detected:
                    force_z = self.current_wrench.wrench.force.z
                    if force_z < contact_threshold:
                        contact_detected = True
                        rospy.logwarn(f"Contact detected at {elapsed:.2f}s, Force Z: {force_z:.2f}N")
                        # 切换到柔顺模式
                        rospy.loginfo("\nPhase 2: Contact - Switching to compliant mode")
                        self.publish_impedance_params(
                            position_stiffness=[500, 500, 20],  # Z轴非常柔顺
                            position_damping=[50, 50, 10],
                            orientation_stiffness=[50, 50, 50],
                            orientation_damping=[10, 10, 10]
                        )
            
            # 第二阶段：保持
            elif elapsed < self.approach_duration + 5.0:
                position = [base_position[0], base_position[1], self.approach_end_z]
            
            else:
                rospy.loginfo("Approach contact test completed!")
                break
            
            msg = self.create_pose_msg(position, self.target_orientation)
            self.traj_pub.publish(msg)
            
            # 打印状态
            if int(elapsed * 2) % 2 == 0 and int(elapsed * 2) != int((elapsed - 1.0/self.control_frequency) * 2):
                status = "CONTACT" if contact_detected else "FREE"
                rospy.loginfo(f"Time: {elapsed:.1f}s, Z: {position[2]:.4f}m, Status: {status}")
                if self.current_wrench:
                    force_z = self.current_wrench.wrench.force.z
                    rospy.loginfo(f"  Force Z: {force_z:.2f}N")
            
            rate.sleep()
    
    def run(self):
        """运行测试"""
        rospy.loginfo("Waiting for subscribers to connect...")
        rospy.sleep(1.0)
        
        # 检查阻抗控制器是否在运行
        topics = rospy.get_published_topics()
        if not any('/reference_trajectory' in topic[0] for topic in topics):
            rospy.logwarn("Waiting for impedance controller to start...")
            rospy.sleep(2.0)
        
        try:
            if self.test_mode == 'fixed_position':
                self.test_fixed_position()
            
            elif self.test_mode == 'circular':
                self.test_circular_trajectory()
            
            elif self.test_mode == 'xy_motion':
                self.test_xy_motion()
            
            elif self.test_mode == 'approach_contact':
                self.test_approach_contact()
            
            else:
                rospy.logerr(f"Unknown test mode: {self.test_mode}")
                rospy.loginfo("Available modes: fixed_position, circular, xy_motion, approach_contact")
                return
        
        except rospy.ROSInterruptException:
            rospy.loginfo("Test interrupted by user")
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("Test completed!")
        rospy.loginfo("="*60)


def main():
    """主函数"""
    try:
        tester = ImpedanceControllerTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
