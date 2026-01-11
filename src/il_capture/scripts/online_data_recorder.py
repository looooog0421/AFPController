#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 在线记录示教数据并保存为 HDF5 格式

import rospy
import h5py
import numpy as np
import os
import sys
import yaml
import threading
import tf.transformations as tf_trans
try:
    np.float = float
except AttributeError:
    pass
import ros_numpy  # 【新增】关键库
import datetime

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import PointCloud2
from il_capture.msg import KineticStateStamped
from gravity_compensator import GravityCompensator

# ============== 配置 ===============

# 有效空间范围 以机械臂基座为参考系的立方体区域
ROI_X = [-0.8, -0.4]  # 米
ROI_Y = [-0.45, 0.0]
ROI_Z = [0.0, 0.5]

TARGET_POINT_NUM = 10000  # 目标点云采样点数

# 变换矩阵
TRANS_MATRIX_PATH = "/home/lgx/Project/AFP/src/il_capture/config/gravity_tare.yaml"

# ===================================

def posquat2mat(pos, quat):
    """位置四元数转齐次变换矩阵"""
    T = tf_trans.quaternion_matrix(quat)
    T[0:3, 3] = pos
    return T


class OnlineDataRecorder:
    def __init__(self):
        rospy.init_node('online_data_recorder', anonymous=True)

        # 获取参数
        self.output_dir = rospy.get_param('~data_directory', '/home/lgx/Project/AFP/src/il_capture/data')
        self.task_name = rospy.get_param('~task_name', 'layup')
        
        # 文件名加上当前时间
        time_str = rospy.get_time()
        self.task_name = f"{self.task_name}_{int(time_str)}"

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"创建数据目录: {self.output_dir}")

        # 生成文件名：{task_name}_{year}{month}{day}_{hour}{minute}{second}.hdf5
        date_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.hdf5_path = os.path.join(self.output_dir, f"{self.task_name}_{date_str}.hdf5")

        # 初始化 HDF5 文件
        with h5py.File(self.hdf5_path, 'w') as f:
            f.create_group('data')
            f.attrs['env_name'] = "RealRobot-v0"
        rospy.loginfo(f"Data file: {self.hdf5_path}")

        # 加载标定参数 变换矩阵
        self.load_calibration(TRANS_MATRIX_PATH)

        # 初始化重力补偿器
        self.gravity_compensator = GravityCompensator(TRANS_MATRIX_PATH)

        # --- 运行状态 ---
        self.is_recording = False
        self.demo_count = 0
        self.lock = threading.Lock()

        # 缓存 ROS 数据
        self.lastest_wrench_msg = None
        self.lastest_mocap_msg = None
        # self.lastest_pcd_msg = None
        # 数据缓冲
        self.reset_buffers()

        # 订阅话题
        rospy.loginfo("订阅力传感器话题...")
        rospy.Subscriber('/netft_data', WrenchStamped, self.force_callback, queue_size=1)
        rospy.loginfo("订阅动捕话题...")
        rospy.Subscriber('/mimic_tool/kinetic_state', KineticStateStamped, self.mocap_callback, queue_size=1)

        # 订阅点云话题
        rospy.loginfo("订阅点云话题...")
        rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcd_callback, queue_size=1, buff_size=2**24)

        # 【调试】 发布处理后的点云数据
        # self.pcd_pub = rospy.Publisher('/debug/roi_pointcloud', PointCloud2, queue_size=1)

        rospy.loginfo(f"ROI Settings: X{ROI_X}, Y{ROI_Y}, Z{ROI_Z}")
        rospy.loginfo("Recorder 节点初始化完成。等待指令...")

        threading.Thread(target=self._console_control, daemon=True).start()

    
    def load_calibration(self, yaml_path):
        """加载变换矩阵"""
        rospy.loginfo(f"加载标定参数: {yaml_path}")
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            # 1. Robot Base <-> Mocap World
            T_robot_to_mocap = np.array(data['T_robot_to_mocap']).reshape(4,4)
            self.T_mocap_to_robot = np.linalg.inv(T_robot_to_mocap)

            if 'T_ee_to_tool' in data:
                self.T_ee_to_tool = np.array(data['T_ee_to_tool']).reshape(4,4)
            else:
                rospy.logwarn("未找到末端执行器到工具的变换矩阵，默认使用单位矩阵。")
                self.T_ee_to_tool = np.eye(4)

            # 2. Force Sensor <-> Robot End Effector
            # TODO： 是否要进行应用坐标变换？
            if 'T_sensor_to_ee' in data:
                self.T_sensor_to_ee = np.array(data['T_sensor_to_ee']).reshape(4,4)
            elif 'tool' in data and 'transform' in data['tool']:
                self.T_sensor_to_ee = np.array(data['tool']['transform']).reshape(4,4)
            else:
                self.T_sensor_to_ee = np.eye(4)
                rospy.logwarn("未找到力传感器到末端执行器的变换矩阵，默认使用单位矩阵。")

            # 3. Camera <-> Robot Base
            if 'T_cam_to_robot' in data:
                self.T_cam_to_robot = np.array(data['T_cam_to_robot']).reshape(4,4)
            else:
                self.T_cam_to_robot = np.eye(4)
                rospy.logwarn("未找到相机到机械臂基座的变换矩阵，默认使用单位矩阵。")

        except Exception as e:
            rospy.logerr(f"加载标定参数失败: {e}")
            sys.exit(1)

    def reset_buffers(self):
        """重置数据缓冲区"""
        self.buffer = {
            'timestamp': [],
            'ee_pos': [],
            'ee_quat': [],
            'ee_wrench': [],
            'pointclouds': []
        }

    def force_callback(self, msg: WrenchStamped):
        """力传感器回调"""
        self.lastest_wrench_msg = msg
        
    def mocap_callback(self, msg: KineticStateStamped):
        """动捕回调"""
        self.lastest_mocap_msg = msg
    
    def pcd_callback(self, pcd_msg: PointCloud2):
        """点云回调"""
        if not self.is_recording:
            return 
        
        with self.lock:
            if self.lastest_mocap_msg is None:
                rospy.logwarn("尚未收到动捕数据，无法处理点云")
                return
            
            if self.lastest_wrench_msg is None:
                rospy.logwarn("尚未收到力传感器数据，无法处理点云")
                return            
            
            
            # 1. 获取当前时间戳
            curr_time = pcd_msg.header.stamp.to_sec()

            # 2. 计算机械臂末端位姿
            tool_pos_msg = self.lastest_mocap_msg.pose.position
            tool_quat_msg = self.lastest_mocap_msg.pose.orientation

            tool_pos =  np.array([tool_pos_msg.x, tool_pos_msg.y, tool_pos_msg.z])
            tool_quat = np.array([tool_quat_msg.x, tool_quat_msg.y, tool_quat_msg.z, tool_quat_msg.w])
            T_tool_in_mocap = posquat2mat(tool_pos, tool_quat)
            
            T_ee_in_mocap = T_tool_in_mocap @ self.T_ee_to_tool

            T_ee_in_base = self.T_mocap_to_robot @ T_ee_in_mocap

            # 执行器末端在机械臂基座坐标系下的位姿
            ee_pos_in_base = T_ee_in_base[0:3, 3]
            ee_quat_in_base = tf_trans.quaternion_from_matrix(T_ee_in_base)
            
            # 3. 计算工具末端受到的力 （需要先进行重力补偿）

            ## 3.1 获取重力补偿后的力传感器数据 在力传感器坐标系下
            ft_msg = self.lastest_wrench_msg.wrench
            raw_f = np.array([ft_msg.force.x, ft_msg.force.y, ft_msg.force.z])
            raw_t = np.array([ft_msg.torque.x, ft_msg.torque.y, ft_msg.torque.z])

            comp_f, comp_t = self.gravity_compensator.get_compensated_wrench(
                raw_f, raw_t, tool_quat
            )
            comp_wrench = np.concatenate([comp_f, comp_t], axis=0)
            
            # 4. 处理点云数据
            
            pc = ros_numpy.numpify(pcd_msg)

            points = np.vstack([pc['x'].reshape(-1), pc['y'].reshape(-1), pc['z'].reshape(-1)]).T

            if 'rgb' in pc.dtype.names:
                pc = ros_numpy.point_cloud2.split_rgb_field(pc)
                r = pc['r'].reshape(-1)
                g = pc['g'].reshape(-1)
                b = pc['b'].reshape(-1)
                # 归一化 RGB
                colors = np.vstack([r, g, b]).T / 255.0
            else:
                # 如果没有 RGB 字段，默认填充全 1 (白色) 或者全 0 (黑色)
                # rospy.logwarn_throttle(10, "点云缺少 RGB 字段，使用默认颜色填充")
                num_points = len(points)
                colors = np.zeros((num_points, 3), dtype=np.float32) # 全白
                
            # 过滤无效点
            valid_mask = ~np.isnan(points).any(axis=1)
            points = points[valid_mask]
            colors = colors[valid_mask]

            if len(points) == 0:
                rospy.logwarn("点云数据全部无效，跳过该帧")
                return
            
            ones = np.ones((points.shape[0], 1))
            points_homo = np.hstack([points, ones])

            points_base = (self.T_cam_to_robot @ points_homo.T).T[:, :3] 

            # 空间裁剪
            roi_mask = (
                (points_base[:, 0] > ROI_X[0]) & (points_base[:, 0] < ROI_X[1]) &
                (points_base[:, 1] > ROI_Y[0]) & (points_base[:, 1] < ROI_Y[1]) &
                (points_base[:, 2] > ROI_Z[0]) & (points_base[:, 2] < ROI_Z[1])
            )

            roi_points = points_base[roi_mask]
            roi_colors = colors[roi_mask]

            # 采样
            n = len(roi_points)
            cloud_frame = np.zeros((TARGET_POINT_NUM, 6), dtype=np.float32)
            if n>=TARGET_POINT_NUM:
                idx = np.random.choice(n, TARGET_POINT_NUM, replace=False)
                cloud_frame = np.hstack([roi_points[idx], roi_colors[idx]])
            else:
                cloud_frame[:n, :3] = roi_points
                cloud_frame[:n, 3:] = roi_colors

            pointclouds = cloud_frame
            # --- 存入缓冲 ---
            self.buffer['timestamp'].append(curr_time)
            self.buffer['ee_pos'].append(ee_pos_in_base)
            self.buffer['ee_quat'].append(ee_quat_in_base)
            self.buffer['ee_wrench'].append(comp_wrench)
            self.buffer['pointclouds'].append(pointclouds)

            # TODO: 定时打印状态
            if len(self.buffer['timestamp']) % 60 == 0:
                print(f"\r[Demo {self.demo_count}] Frames: {len(self.buffer['timestamp'])}", end="")

    def _console_control(self):
        """
        终端交互
        """
        print("\n=== Data Recorder Control")
        print("[s] 开始示教记录")
        print("[q] 停止示教记录并保存")
        print("[e] 退出程序\n")

        while not rospy.is_shutdown():
            try:
                cmd = input("输入指令 (s/q/e): ").strip().lower()
            except EOFError:
                break

            if cmd == 's':
                if not self.is_recording:
                    self.reset_buffers()
                    self.is_recording = True
                    rospy.loginfo("开始示教记录...")
                else:
                    rospy.logwarn("已经在记录中，无法重复开始。")
            elif cmd == 'q':
                if self.is_recording:
                    self.is_recording = False
                    rospy.loginfo("停止记录，正在保存数据...")
                    self.save_hdf5()
                else:
                    rospy.logwarn("当前未在记录中，无法停止。")
            elif cmd == "e":
                rospy.loginfo("退出程序...")
                rospy.signal_shutdown("用户退出")
                break

    def save_hdf5(self):
        num_frames = len(self.buffer['timestamp'])
        if num_frames < 2:
            rospy.logwarn("没有记录到任何数据，跳过保存。")
            return

        rospy.loginfo(f"Processing Demo {self.demo_count} structure...")

        arr_pos = np.array(self.buffer['ee_pos'], dtype=np.float32) # (time, 3)
        arr_quat = np.array(self.buffer['ee_quat'], dtype=np.float32) # (time, 4)
        arr_wrench = np.array(self.buffer['ee_wrench'], dtype=np.float32) # (time, 6)
        arr_pcds = np.array(self.buffer['pointclouds'], dtype=np.float32) # (time, point_num, 6) point_num = TARGET_POINT_NUM

        # 2. 切片逻辑
        obs_pos = arr_pos[:-1]
        obs_quat = arr_quat[:-1]
        obs_wrench = arr_wrench[:-1]
        obs_pcds = arr_pcds[:-1]

        act_pos = arr_pos[1:]
        act_quat = arr_quat[1:]
        act_wrench = arr_wrench[1:]

        # 构建action向量
        act_no_wrench = np.concatenate([act_pos, act_quat], axis=1)
        act_with_wrench = np.concatenate([act_pos, act_quat, act_wrench], axis=1)

        # 辅助数据
        num_samples = obs_pos.shape[0]
        
        dones = np.zeros((num_samples,), dtype='int64')
        dones[-1] = 1

        rewards = np.zeros((num_samples,), dtype='float32')

        labels = np.zeros((num_samples,), dtype='int64')

        # 3. 写入 HDF5
        with h5py.File(self.hdf5_path, 'a') as f:
            data_grp = f['data']
            demo_grp = data_grp.create_group(f'demo_{self.demo_count}')

            # 根属性
            demo_grp.attrs['num_samples'] = num_samples

            # 顶级数据集
            demo_grp.create_dataset('action_without_wrench', data=act_no_wrench)
            demo_grp.create_dataset('action_with_wrench', data=act_with_wrench)
            demo_grp.create_dataset('dones', data=dones)
            demo_grp.create_dataset('rewards', data=rewards)

            # obs Group
            obs_grp = demo_grp.create_group("obs")
            obs_grp.create_dataset('robot0_eef_pos', data=obs_pos)
            obs_grp.create_dataset('robot0_eef_quat', data=obs_quat)
            obs_grp.create_dataset('robot0_eef_wrench', data=obs_wrench)
            obs_grp.create_dataset('label', data=labels)

            obs_grp.create_dataset('pointcloud', data=obs_pcds, compression="gzip", compression_opts=4)

            rospy.loginfo(f"Demo {self.demo_count} 保存完成，共 {num_samples} 帧数据。")
        self.demo_count += 1

if __name__ == "__main__":
    try:
        recorder = OnlineDataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass