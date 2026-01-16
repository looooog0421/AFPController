#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取ros节点jointstate数据,并计算出末端位置,绘制轨迹图
"""

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pinocchio as pin
import os
import sys
from collections import deque


class TrajectoryPlotter:
    def __init__(self, update_interval=0.1):
        """初始化轨迹绘图器
        
        Args:
            update_interval: 图像更新间隔（秒）
        """
        rospy.init_node('trajectory_plotter', anonymous=True)
        
        # 加载机器人模型
        urdf_path = os.path.join(os.path.expanduser("~"), 
                                 "Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf")
        self.robot_model = pin.buildModelFromUrdf(urdf_path)
        self.robot_data = self.robot_model.createData()
        
        # 数据存储（保留所有历史点）
        self.positions = deque()  # 末端位置 [x, y, z]
        self.timestamps = deque()
        
        # 关节映射顺序
        self.joint_order = [2, 1, 0, 3, 4, 5]
        
        # 数据更新标志
        self.data_lock = False  # 简单的标志锁
        self.new_data = False
        
        # 订阅关节状态
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, 
                                         self.joint_state_callback, queue_size=1)
        
        # 绘图设置
        self.update_interval = update_interval
        
        # 创建图形（必须在主线程中）
        plt.ion()  # 交互模式
        self.fig = plt.figure(figsize=(12, 10))
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_xz = self.fig.add_subplot(223)
        self.ax_yz = self.fig.add_subplot(224)
        
        self.setup_plots()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Trajectory Plotter Initialized")
        rospy.loginfo("Saving all trajectory points (unlimited)")
        rospy.loginfo(f"Update interval: {update_interval}s")
        rospy.loginfo("Subscribing to /joint_states")
        rospy.loginfo("=" * 60)
    
    def setup_plots(self):
        """设置图形"""
        # 3D轨迹图
        self.ax_3d.set_xlabel('X (m)', fontsize=10)
        self.ax_3d.set_ylabel('Y (m)', fontsize=10)
        self.ax_3d.set_zlabel('Z (m)', fontsize=10)
        self.ax_3d.set_title('3D Trajectory', fontsize=12, fontweight='bold')
        self.ax_3d.set_xlim(-1, 0)
        self.ax_3d.set_ylim(-1, 0)
        self.ax_3d.set_zlim(-1, 0)
        self.ax_3d.grid(True, alpha=0.3)
        
        # XY平面
        self.ax_xy.set_xlabel('X (m)', fontsize=10)
        self.ax_xy.set_ylabel('Y (m)', fontsize=10)
        self.ax_xy.set_title('XY Plane', fontsize=12, fontweight='bold')
        self.ax_xy.set_xlim(-1, 1)
        self.ax_xy.set_ylim(-1, 0)
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        # XZ平面
        self.ax_xz.set_xlabel('X (m)', fontsize=10)
        self.ax_xz.set_ylabel('Z (m)', fontsize=10)
        self.ax_xz.set_title('XZ Plane', fontsize=12, fontweight='bold')
        self.ax_xz.set_xlim(-1, 0)
        self.ax_xz.set_ylim(-1, 0)
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        # YZ平面
        self.ax_yz.set_xlabel('Y (m)', fontsize=10)
        self.ax_yz.set_ylabel('Z (m)', fontsize=10)
        self.ax_yz.set_title('YZ Plane', fontsize=12, fontweight='bold')
        self.ax_yz.set_xlim(-1, 0)
        self.ax_yz.set_ylim(-1, 0)
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
        
        plt.tight_layout()
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调函数 - 仅存储数据"""
        try:
            # 重新映射关节顺序
            joint_positions = np.array(msg.position)[self.joint_order]
            
            # 正运动学计算
            pin.forwardKinematics(self.robot_model, self.robot_data, joint_positions)
            pin.updateFramePlacements(self.robot_model, self.robot_data)
            
            # 获取末端位置
            flange_id = self.robot_model.getFrameId("flange")
            ee_transform = self.robot_data.oMf[flange_id]
            ee_position = ee_transform.translation
            
            # 保存数据（线程安全）
            if not self.data_lock:
                self.data_lock = True
                current_time = rospy.Time.now().to_sec()
                self.positions.append(ee_position.copy())
                self.timestamps.append(current_time)
                self.new_data = True
                self.data_lock = False
            
        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {e}")
    
    def update_plot(self):
        """更新绘图 - 必须在主线程中调用"""
        if not self.new_data or len(self.positions) < 2:
            return
        
        self.new_data = False
        
        # 转换为numpy数组
        positions_array = np.array(self.positions)
        x = positions_array[:, 0]
        y = positions_array[:, 1]
        z = positions_array[:, 2]
        
        # 创建渐变颜色（从蓝到红，表示时间）
        n_points = len(x)
        colors = plt.cm.viridis(np.linspace(0, 1, n_points))
        
        # 清空所有图
        self.ax_3d.cla()
        self.ax_xy.cla()
        self.ax_xz.cla()
        self.ax_yz.cla()
        
        # 绘制3D轨迹
        self.ax_3d.scatter(x, y, z, c=colors, s=1, alpha=0.6)
        self.ax_3d.plot(x, y, z, 'b-', alpha=0.3, linewidth=0.5)
        self.ax_3d.scatter(x[-1], y[-1], z[-1], c='r', s=50, marker='o', 
                          edgecolors='black', linewidths=1.5, label='Current')
        self.ax_3d.set_xlabel('X (m)', fontsize=10)
        self.ax_3d.set_ylabel('Y (m)', fontsize=10)
        self.ax_3d.set_zlabel('Z (m)', fontsize=10)
        self.ax_3d.set_title(f'3D Trajectory ({n_points} points)', fontsize=12, fontweight='bold')
        self.ax_3d.legend()
        self.ax_3d.grid(True, alpha=0.3)
        
        # XY平面
        self.ax_xy.scatter(x, y, c=colors, s=1, alpha=0.6)
        self.ax_xy.plot(x, y, 'b-', alpha=0.3, linewidth=0.5)
        self.ax_xy.scatter(x[-1], y[-1], c='r', s=50, marker='o', 
                          edgecolors='black', linewidths=1.5)
        self.ax_xy.set_xlabel('X (m)', fontsize=10)
        self.ax_xy.set_ylabel('Y (m)', fontsize=10)
        self.ax_xy.set_title('XY Plane', fontsize=12, fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        # XZ平面
        self.ax_xz.scatter(x, z, c=colors, s=1, alpha=0.6)
        self.ax_xz.plot(x, z, 'b-', alpha=0.3, linewidth=0.5)
        self.ax_xz.scatter(x[-1], z[-1], c='r', s=50, marker='o', 
                          edgecolors='black', linewidths=1.5)
        self.ax_xz.set_xlabel('X (m)', fontsize=10)
        self.ax_xz.set_ylabel('Z (m)', fontsize=10)
        self.ax_xz.set_title('XZ Plane', fontsize=12, fontweight='bold')
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        # YZ平面
        self.ax_yz.scatter(y, z, c=colors, s=1, alpha=0.6)
        self.ax_yz.plot(y, z, 'b-', alpha=0.3, linewidth=0.5)
        self.ax_yz.scatter(y[-1], z[-1], c='r', s=50, marker='o', 
                          edgecolors='black', linewidths=1.5)
        self.ax_yz.set_xlabel('Y (m)', fontsize=10)
        self.ax_yz.set_ylabel('Z (m)', fontsize=10)
        self.ax_yz.set_title('YZ Plane', fontsize=12, fontweight='bold')
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
        
        # 更新显示
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)
    
    def save_trajectory(self, filename='trajectory.txt'):
        """保存轨迹数据到文件"""
        if len(self.positions) == 0:
            rospy.logwarn("No trajectory data to save")
            return
        
        data = np.column_stack([self.timestamps, np.array(self.positions)])
        np.savetxt(filename, data, header='timestamp x y z', 
                   fmt='%.6f', delimiter='\t')
        rospy.loginfo(f"Saved {len(self.positions)} points to {filename}")
    
    def run(self):
        """运行绘图器 - 使用定时器在主线程更新图形"""
        rospy.loginfo("Trajectory plotter running. Press Ctrl+C to stop and save.")
        
        # 创建matplotlib定时器在主线程中更新
        timer = self.fig.canvas.new_timer(interval=int(self.update_interval * 1000))
        timer.add_callback(self.update_plot)
        timer.start()
        
        try:
            # 让ROS在后台运行
            while not rospy.is_shutdown():
                plt.pause(0.1)  # 让matplotlib处理事件
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            timer.stop()
            self.save_trajectory()
            plt.ioff()
            plt.show()


if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotter(update_interval=0.1)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
