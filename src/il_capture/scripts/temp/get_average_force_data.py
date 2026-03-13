#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
接收力传感器数据1秒，计算各轴力和力矩的均值
"""

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
import sys


class ForceDataAverager:
    def __init__(self, duration=1.0):
        """
        初始化力传感器数据平均器
        
        Args:
            duration: 采集时间（秒）
        """
        rospy.init_node('force_data_averager', anonymous=True)
        
        self.duration = duration
        self.force_list = []
        self.torque_list = []
        self.start_time = None
        self.collecting = False
        
        # 订阅力传感器话题
        self.wrench_sub = rospy.Subscriber('/netft_data', WrenchStamped, 
                                          self.wrench_callback, queue_size=100)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Force Data Averager Initialized")
        rospy.loginfo(f"Collection duration: {duration}s")
        rospy.loginfo("=" * 60)
    
    def wrench_callback(self, msg):
        """力传感器数据回调"""
        if not self.collecting:
            return
        
        # 提取力和力矩
        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])
        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])
        
        # 数据有效性检查
        if np.any(np.isnan(force)) or np.any(np.isnan(torque)):
            rospy.logwarn_throttle(1.0, "Received invalid data (NaN)")
            return
        
        if np.any(np.isinf(force)) or np.any(np.isinf(torque)):
            rospy.logwarn_throttle(1.0, "Received invalid data (Inf)")
            return
        
        # 保存数据
        self.force_list.append(force)
        self.torque_list.append(torque)
        
        # 检查是否超时
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed_time >= self.duration:
            self.collecting = False
            self.compute_and_display()
    
    def collect_data(self):
        """开始采集数据"""
        rospy.loginfo("Starting data collection...")
        rospy.loginfo("Please keep the sensor steady!")
        
        # 清空之前的数据
        self.force_list = []
        self.torque_list = []
        
        # 开始采集
        self.start_time = rospy.Time.now()
        self.collecting = True
        
        # 等待采集完成
        rate = rospy.Rate(10)  # 10Hz检查频率
        while self.collecting and not rospy.is_shutdown():
            rate.sleep()
    
    def compute_and_display(self):
        """计算并显示均值"""
        if len(self.force_list) == 0:
            rospy.logerr("No data collected!")
            return
        
        # 转换为numpy数组
        force_array = np.array(self.force_list)
        torque_array = np.array(self.torque_list)
        
        # 计算均值
        force_mean = np.mean(force_array, axis=0)
        torque_mean = np.mean(torque_array, axis=0)
        
        # 计算标准差
        force_std = np.std(force_array, axis=0)
        torque_std = np.std(torque_array, axis=0)
        
        # 计算最大最小值
        force_min = np.min(force_array, axis=0)
        force_max = np.max(force_array, axis=0)
        torque_min = np.min(torque_array, axis=0)
        torque_max = np.max(torque_array, axis=0)
        
        # 显示结果
        rospy.loginfo("=" * 60)
        rospy.loginfo("FORCE/TORQUE SENSOR DATA ANALYSIS")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Samples collected: {len(self.force_list)}")
        rospy.loginfo(f"Collection duration: {self.duration:.2f}s")
        rospy.loginfo(f"Sample rate: {len(self.force_list)/self.duration:.1f} Hz")
        rospy.loginfo("")
        
        rospy.loginfo("FORCE (N):")
        rospy.loginfo(f"  Mean:   X={force_mean[0]:8.4f}, Y={force_mean[1]:8.4f}, Z={force_mean[2]:8.4f}")
        rospy.loginfo(f"  Std:    X={force_std[0]:8.4f}, Y={force_std[1]:8.4f}, Z={force_std[2]:8.4f}")
        rospy.loginfo(f"  Min:    X={force_min[0]:8.4f}, Y={force_min[1]:8.4f}, Z={force_min[2]:8.4f}")
        rospy.loginfo(f"  Max:    X={force_max[0]:8.4f}, Y={force_max[1]:8.4f}, Z={force_max[2]:8.4f}")
        rospy.loginfo("")
        
        rospy.loginfo("TORQUE (Nm):")
        rospy.loginfo(f"  Mean:   X={torque_mean[0]:8.4f}, Y={torque_mean[1]:8.4f}, Z={torque_mean[2]:8.4f}")
        rospy.loginfo(f"  Std:    X={torque_std[0]:8.4f}, Y={torque_std[1]:8.4f}, Z={torque_std[2]:8.4f}")
        rospy.loginfo(f"  Min:    X={torque_min[0]:8.4f}, Y={torque_min[1]:8.4f}, Z={torque_min[2]:8.4f}")
        rospy.loginfo(f"  Max:    X={torque_max[0]:8.4f}, Y={torque_max[1]:8.4f}, Z={torque_max[2]:8.4f}")
        rospy.loginfo("=" * 60)
        
        # 计算力和力矩的大小
        force_magnitude = np.linalg.norm(force_mean)
        torque_magnitude = np.linalg.norm(torque_mean)
        
        rospy.loginfo("MAGNITUDE:")
        rospy.loginfo(f"  Force:  {force_magnitude:.4f} N")
        rospy.loginfo(f"  Torque: {torque_magnitude:.4f} Nm")
        rospy.loginfo("=" * 60)
        
        # 格式化输出（便于复制）
        print("\nFormatted output for copying:")
        print(f"Force (N):  [{force_mean[0]:.6f}, {force_mean[1]:.6f}, {force_mean[2]:.6f}]")
        print(f"Torque (Nm): [{torque_mean[0]:.6f}, {torque_mean[1]:.6f}, {torque_mean[2]:.6f}]")
    
    def run(self):
        """运行主循环"""
        try:
            # 等待话题准备好
            rospy.loginfo("Waiting for force sensor data...")
            rospy.wait_for_message('/netft_data', WrenchStamped, timeout=5.0)
            rospy.loginfo("Force sensor connected!")
            rospy.sleep(0.5)  # 短暂延迟确保稳定
            
            # 开始采集
            self.collect_data()
            
            rospy.loginfo("Data collection complete.")
            
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for force sensor data: {e}")
            rospy.logerr("Please check if /netft_data topic is publishing")
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")


if __name__ == '__main__':
    try:
        # 从命令行参数读取采集时间（可选）
        duration = 1.0
        if len(sys.argv) > 1:
            try:
                duration = float(sys.argv[1])
                if duration <= 0:
                    rospy.logerr("Duration must be positive")
                    sys.exit(1)
            except ValueError:
                rospy.logerr("Invalid duration argument")
                sys.exit(1)
        
        averager = ForceDataAverager(duration=duration)
        averager.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()