#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取ros节点jointstate数据和参考轨迹,绘制实际轨迹与参考轨迹对比图
"""

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pinocchio as pin
import os
import sys
from collections import deque
import signal


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
        
        # 实际轨迹数据存储
        self.actual_positions = deque()
        self.actual_orientations = deque()
        self.actual_timestamps = deque()
        
        # 参考轨迹数据存储
        self.ref_positions = deque()
        self.ref_orientations = deque()
        self.ref_timestamps = deque()
        
        # 关节映射顺序
        self.joint_order = [2, 1, 0, 3, 4, 5]
        
        # 数据更新标志
        self.data_lock = False
        self.new_data = False
        
        # 运行标志
        self.running = True
        
        # 订阅关节状态（实际轨迹）
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, 
                                         self.joint_state_callback, queue_size=1)
        
        # 订阅参考轨迹
        self.ref_sub = rospy.Subscriber('/reference_trajectory', PoseStamped,
                                       self.reference_callback, queue_size=1)
        
        # 绘图设置
        self.update_interval = update_interval
        
        # 创建图形（必须在主线程中）
        plt.ion()
        self.fig = plt.figure(figsize=(16, 12))
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_xz = self.fig.add_subplot(223)
        self.ax_yz = self.fig.add_subplot(224)
        
        self.setup_plots()
        
        # 添加定时器引用
        self.timer = None
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Trajectory Plotter with Reference Initialized")
        rospy.loginfo(f"Update interval: {update_interval}s")
        rospy.loginfo("Subscribing to /joint_states (actual trajectory)")
        rospy.loginfo("Subscribing to /reference_trajectory (reference)")
        rospy.loginfo("=" * 60)
    
    def setup_plots(self):
        """设置图形"""
        # 3D轨迹图
        self.ax_3d.set_xlabel('X (m)', fontsize=10)
        self.ax_3d.set_ylabel('Y (m)', fontsize=10)
        self.ax_3d.set_zlabel('Z (m)', fontsize=10)
        self.ax_3d.set_title('3D Trajectory', fontsize=12, fontweight='bold')
        self.ax_3d.grid(True, alpha=0.3)
        
        # XY平面
        self.ax_xy.set_xlabel('X (m)', fontsize=10)
        self.ax_xy.set_ylabel('Y (m)', fontsize=10)
        self.ax_xy.set_title('XY Plane', fontsize=12, fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        # XZ平面
        self.ax_xz.set_xlabel('X (m)', fontsize=10)
        self.ax_xz.set_ylabel('Z (m)', fontsize=10)
        self.ax_xz.set_title('XZ Plane', fontsize=12, fontweight='bold')
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        # YZ平面
        self.ax_yz.set_xlabel('Y (m)', fontsize=10)
        self.ax_yz.set_ylabel('Z (m)', fontsize=10)
        self.ax_yz.set_title('YZ Plane', fontsize=12, fontweight='bold')
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
        
        plt.tight_layout()
    
    def reference_callback(self, msg: PoseStamped):
        """参考轨迹回调函数"""
        if not self.running:
            return
        
        try:
            if not self.data_lock:
                self.data_lock = True
                
                # 提取位置
                pos = np.array([msg.pose.position.x, 
                               msg.pose.position.y, 
                               msg.pose.position.z])
                
                # 提取姿态（四元数）
                quat = np.array([msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z,
                                msg.pose.orientation.w])
                
                current_time = rospy.Time.now().to_sec()
                self.ref_positions.append(pos)
                self.ref_orientations.append(quat)
                self.ref_timestamps.append(current_time)
                
                self.new_data = True
                self.data_lock = False
                
        except Exception as e:
            rospy.logerr(f"Error in reference callback: {e}")
            self.data_lock = False
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调函数 - 实际轨迹"""
        if not self.running:
            return
            
        try:
            # 重新映射关节顺序
            joint_positions = np.array(msg.position)[self.joint_order]
            
            # 正运动学计算
            pin.forwardKinematics(self.robot_model, self.robot_data, joint_positions)
            pin.updateFramePlacements(self.robot_model, self.robot_data)
            
            # 获取末端位置
            tool0_id = self.robot_model.getFrameId("tool0")
            ee_transform = self.robot_data.oMf[tool0_id]
            ee_position = ee_transform.translation
            ee_orientation = ee_transform.rotation
            ee_quat = pin.Quaternion(ee_orientation).coeffs()
            
            # 保存数据
            if not self.data_lock:
                self.data_lock = True
                current_time = rospy.Time.now().to_sec()
                self.actual_positions.append(ee_position.copy())
                self.actual_orientations.append(ee_quat.copy())
                self.actual_timestamps.append(current_time)
                self.new_data = True
                self.data_lock = False
            
        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {e}")
    
    def update_plot(self):
        """更新绘图 - 显示实际轨迹和参考轨迹"""
        if not self.running or not self.new_data:
            return
        
        if len(self.actual_positions) < 2 and len(self.ref_positions) < 2:
            return
        
        self.new_data = False
        
        # 清空所有图
        self.ax_3d.cla()
        self.ax_xy.cla()
        self.ax_xz.cla()
        self.ax_yz.cla()
        
        # 绘制参考轨迹（绿色）
        if len(self.ref_positions) > 0:
            ref_array = np.array(self.ref_positions)
            rx, ry, rz = ref_array[:, 0], ref_array[:, 1], ref_array[:, 2]
            
            # 3D
            self.ax_3d.plot(rx, ry, rz, 'g-', linewidth=2, alpha=0.6, label='Reference')
            self.ax_3d.scatter(rx[-1], ry[-1], rz[-1], c='g', s=50, marker='s', 
                              edgecolors='black', linewidths=1.5)
            
            # XY
            self.ax_xy.plot(rx, ry, 'g-', linewidth=2, alpha=0.6, label='Reference')
            self.ax_xy.scatter(rx[-1], ry[-1], c='g', s=50, marker='s', 
                              edgecolors='black', linewidths=1.5)
            
            # XZ
            self.ax_xz.plot(rx, rz, 'g-', linewidth=2, alpha=0.6, label='Reference')
            self.ax_xz.scatter(rx[-1], rz[-1], c='g', s=50, marker='s', 
                              edgecolors='black', linewidths=1.5)
            
            # YZ
            self.ax_yz.plot(ry, rz, 'g-', linewidth=2, alpha=0.6, label='Reference')
            self.ax_yz.scatter(ry[-1], rz[-1], c='g', s=50, marker='s', 
                              edgecolors='black', linewidths=1.5)
        
        # 绘制实际轨迹（蓝色）
        if len(self.actual_positions) > 0:
            actual_array = np.array(self.actual_positions)
            ax, ay, az = actual_array[:, 0], actual_array[:, 1], actual_array[:, 2]
            
            # 3D
            self.ax_3d.plot(ax, ay, az, 'b-', linewidth=1.5, alpha=0.7, label='Actual')
            self.ax_3d.scatter(ax[-1], ay[-1], az[-1], c='r', s=50, marker='o', 
                              edgecolors='black', linewidths=1.5, label='Current')
            
            # XY
            self.ax_xy.plot(ax, ay, 'b-', linewidth=1.5, alpha=0.7, label='Actual')
            self.ax_xy.scatter(ax[-1], ay[-1], c='r', s=50, marker='o', 
                              edgecolors='black', linewidths=1.5)
            
            # XZ
            self.ax_xz.plot(ax, az, 'b-', linewidth=1.5, alpha=0.7, label='Actual')
            self.ax_xz.scatter(ax[-1], az[-1], c='r', s=50, marker='o', 
                              edgecolors='black', linewidths=1.5)
            
            # YZ
            self.ax_yz.plot(ay, az, 'b-', linewidth=1.5, alpha=0.7, label='Actual')
            self.ax_yz.scatter(ay[-1], az[-1], c='r', s=50, marker='o', 
                              edgecolors='black', linewidths=1.5)
        
        # 计算跟踪误差
        tracking_error = 0.0
        if len(self.actual_positions) > 0 and len(self.ref_positions) > 0:
            actual_pos = np.array(self.actual_positions[-1])
            ref_pos = np.array(self.ref_positions[-1])
            tracking_error = np.linalg.norm(actual_pos - ref_pos)
        
        # 设置标题和图例
        self.ax_3d.set_xlabel('X (m)', fontsize=10)
        self.ax_3d.set_ylabel('Y (m)', fontsize=10)
        self.ax_3d.set_zlabel('Z (m)', fontsize=10)
        self.ax_3d.set_title(f'3D Trajectory (Actual: {len(self.actual_positions)} | Ref: {len(self.ref_positions)} | Error: {tracking_error:.4f}m)', 
                            fontsize=11, fontweight='bold')
        self.ax_3d.legend(fontsize=9)
        self.ax_3d.grid(True, alpha=0.3)
        
        self.ax_xy.set_xlabel('X (m)', fontsize=10)
        self.ax_xy.set_ylabel('Y (m)', fontsize=10)
        self.ax_xy.set_title('XY Plane', fontsize=11, fontweight='bold')
        self.ax_xy.legend(fontsize=9)
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        self.ax_xz.set_xlabel('X (m)', fontsize=10)
        self.ax_xz.set_ylabel('Z (m)', fontsize=10)
        self.ax_xz.set_title('XZ Plane', fontsize=11, fontweight='bold')
        self.ax_xz.legend(fontsize=9)
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        self.ax_yz.set_xlabel('Y (m)', fontsize=10)
        self.ax_yz.set_ylabel('Z (m)', fontsize=10)
        self.ax_yz.set_title('YZ Plane', fontsize=11, fontweight='bold')
        self.ax_yz.legend(fontsize=9)
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
        
        # 更新显示
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)
    
    def save_trajectory(self, filename='trajectory.txt'):
        """保存轨迹数据到文件（包含实际和参考轨迹）"""
        if len(self.actual_positions) == 0 and len(self.ref_positions) == 0:
            rospy.logwarn("No trajectory data to save")
            return
        
        # 保存实际轨迹
        if len(self.actual_positions) > 0:
            actual_data = np.column_stack([
                self.actual_timestamps, 
                np.array(self.actual_positions), 
                np.array(self.actual_orientations)
            ])
            actual_file = filename.replace('.txt', '_actual.txt')
            np.savetxt(actual_file, actual_data, 
                      header='timestamp x y z qx qy qz qw', 
                      fmt='%.6f', delimiter='\t')
            rospy.loginfo(f"Saved {len(self.actual_positions)} actual points to {actual_file}")
        
        # 保存参考轨迹
        if len(self.ref_positions) > 0:
            ref_data = np.column_stack([
                self.ref_timestamps,
                np.array(self.ref_positions),
                np.array(self.ref_orientations)
            ])
            ref_file = filename.replace('.txt', '_reference.txt')
            np.savetxt(ref_file, ref_data,
                      header='timestamp x y z qx qy qz qw',
                      fmt='%.6f', delimiter='\t')
            rospy.loginfo(f"Saved {len(self.ref_positions)} reference points to {ref_file}")
    
    def shutdown(self):
        """清理资源并保存数据"""
        rospy.loginfo("Shutting down trajectory plotter...")
        self.running = False
        
        # 停止定时器
        if self.timer is not None:
            self.timer.stop()
        
        # 取消订阅
        self.joint_sub.unregister()
        self.ref_sub.unregister()
        
        # 保存数据
        self.save_trajectory()
        
        # 关闭matplotlib
        plt.ioff()
        plt.close('all')
        
        rospy.loginfo("Trajectory plotter stopped.")
    
    def run(self):
        """运行绘图器"""
        rospy.loginfo("Trajectory plotter running. Press Ctrl+C to stop and save.")
        rospy.loginfo("Green = Reference trajectory | Blue = Actual trajectory")
        
        # 创建matplotlib定时器
        self.timer = self.fig.canvas.new_timer(interval=int(self.update_interval * 1000))
        self.timer.add_callback(self.update_plot)
        self.timer.start()
        
        # 注册ROS关闭钩子
        rospy.on_shutdown(self.shutdown)
        
        try:
            while not rospy.is_shutdown() and self.running:
                plt.pause(0.1)
                
                # 检查窗口是否关闭
                if not plt.fignum_exists(self.fig.number):
                    rospy.loginfo("Plot window closed by user.")
                    break
                    
        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt received.")
        finally:
            if self.running:
                self.shutdown()


if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotter(update_interval=0.1)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()