#!/usr/bin/env python3

import rospy
import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter


class DemoTrajectoryPublisher:

    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node("demo_traj_publisher_200hz")

        # 2. 参数获取
        default_path = "/home/lgx/Project/AFP/src/il_capture/data/layup_1768983281_20260121_161441_90_selected_selected_corrected.hdf5"
        self.file_path = rospy.get_param("~file_path", default_path)
        self.frame_id = rospy.get_param("~frame_id", "base_link")

        # 3. 数据处理
        self.raw_traj_200hz, self.smooth_traj_200hz = self.load_and_process(self.file_path)
        
        # 最终执行使用平滑后的轨迹
        self.trajectory = self.smooth_traj_200hz
        self.total_steps = self.trajectory.shape[0]
        
        # 状态控制变量
        self.index = 0
        self.is_executing = False  # 标志位：是否开始顺着轨迹往下走
        self.is_finished = False   # 标志位：是否执行结束

        # 4. 初始化发布者
        self.pub = rospy.Publisher("/reference_trajectory",
                                   PoseStamped,
                                   queue_size=10)

        rospy.loginfo(f"Data loaded successfully. Final trajectory length: {self.total_steps}")

    # ===============================
    # 核心数据处理 Pipeline
    # ===============================
    def load_and_process(self, file_path):
        with h5py.File(file_path, "r") as f:
            data_group = f["data"]
            demo_names = list(data_group.keys())
            if len(demo_names) == 0:
                raise RuntimeError("No demo found in hdf5 file.")
            demo_name = demo_names[0]
            action_20hz = data_group[demo_name]["action_without_wrench"][:]

        raw_traj_200hz = self.upsample_20_to_200(action_20hz)
        
        # 这里使用了加大平滑力度的参数 (window=151)，你可以根据实际效果调整
        smooth_traj_200hz = self.smooth_trajectory(raw_traj_200hz, window_length=151, polyorder=3)
        
        return raw_traj_200hz, smooth_traj_200hz

    def upsample_20_to_200(self, traj_20hz):
        N = traj_20hz.shape[0]
        pos = traj_20hz[:, 0:3]
        quat = traj_20hz[:, 3:7]
        quat = quat / np.linalg.norm(quat, axis=1, keepdims=True)

        t_old = np.arange(N) / 20.0
        total_time = t_old[-1]
        num_new_steps = int(np.round(total_time * 200.0)) + 1 
        t_new = np.linspace(0, total_time, num_new_steps)

        pos_interp_func = interp1d(t_old, pos, axis=0, kind='linear')
        pos_new = pos_interp_func(t_new)

        rotations = R.from_quat(quat)
        slerp = Slerp(t_old, rotations)
        rot_new = slerp(t_new)
        quat_new = rot_new.as_quat()

        return np.hstack([pos_new, quat_new])

    def smooth_trajectory(self, traj_200hz, window_length=151, polyorder=3):
        if window_length % 2 == 0: window_length += 1
        if window_length > traj_200hz.shape[0]:
            window_length = traj_200hz.shape[0] - 1 if traj_200hz.shape[0] % 2 == 0 else traj_200hz.shape[0]

        smoothed_traj = np.zeros_like(traj_200hz)
        pos = traj_200hz[:, 0:3]
        smoothed_traj[:, 0:3] = savgol_filter(pos, window_length, polyorder, axis=0)

        quat = traj_200hz[:, 3:7]
        rotvecs = R.from_quat(quat).as_rotvec() 
        smoothed_rotvecs = savgol_filter(rotvecs, window_length, polyorder, axis=0)
        smoothed_traj[:, 3:7] = R.from_rotvec(smoothed_rotvecs).as_quat()

        return smoothed_traj

    # ===============================
    # 可视化模块 (升级版：包含姿态可视化)
    # ===============================
    def visualize_trajectory(self):
        rospy.loginfo("Opening Matplotlib figure for visualization...")
        
        # 提取位置
        raw_pos = self.raw_traj_200hz[:, :3]
        smooth_pos = self.smooth_traj_200hz[:, :3]
        
        # 提取四元数
        raw_quat = self.raw_traj_200hz[:, 3:7]
        smooth_quat = self.smooth_traj_200hz[:, 3:7]
        
        time_axis = np.arange(self.total_steps) / 200.0

        # 创建更大的画布并使用 GridSpec 布局
        fig = plt.figure(figsize=(16, 8))
        from matplotlib.gridspec import GridSpec
        gs = GridSpec(2, 2, figure=fig)

        # ---------------------------------------------------------
        # 1. 3D 位置与姿态轨迹 (占据左侧一整列)
        # ---------------------------------------------------------
        ax1 = fig.add_subplot(gs[:, 0], projection='3d')
        ax1.plot(raw_pos[:, 0], raw_pos[:, 1], raw_pos[:, 2], color='gray', linestyle='--', alpha=0.6, label='Raw Path')
        ax1.plot(smooth_pos[:, 0], smooth_pos[:, 1], smooth_pos[:, 2], color='black', linewidth=2, label='Smooth Path')
        ax1.scatter(*smooth_pos[0], color='g', s=80, label='Start')
        ax1.scatter(*smooth_pos[-1], color='r', s=80, label='End')
        
        # 稀疏采样画出 3D 坐标轴 (箭头)，比如整条轨迹画 20 个坐标系
        step = max(1, self.total_steps // 20)
        arrow_length = 0.02  # 箭头长度 (2cm)，可根据你的实际尺度调整
        
        for i in range(0, self.total_steps, step):
            p = smooth_pos[i]
            q = smooth_quat[i]
            rot_matrix = R.from_quat(q).as_matrix()
            
            x_dir = rot_matrix[:, 0] * arrow_length
            y_dir = rot_matrix[:, 1] * arrow_length
            z_dir = rot_matrix[:, 2] * arrow_length
            
            # 画出 X(红), Y(绿), Z(蓝) 轴
            ax1.quiver(p[0], p[1], p[2], x_dir[0], x_dir[1], x_dir[2], color='r', alpha=0.8)
            ax1.quiver(p[0], p[1], p[2], y_dir[0], y_dir[1], y_dir[2], color='g', alpha=0.8)
            ax1.quiver(p[0], p[1], p[2], z_dir[0], z_dir[1], z_dir[2], color='b', alpha=0.8)

        # 调整 3D 视图比例
        max_range = np.array([smooth_pos[:, 0].max() - smooth_pos[:, 0].min(),
                              smooth_pos[:, 1].max() - smooth_pos[:, 1].min(),
                              smooth_pos[:, 2].max() - smooth_pos[:, 2].min()]).max() / 2.0
        mid_x = (smooth_pos[:, 0].max() + smooth_pos[:, 0].min()) * 0.5
        mid_y = (smooth_pos[:, 1].max() + smooth_pos[:, 1].min()) * 0.5
        mid_z = (smooth_pos[:, 2].max() + smooth_pos[:, 2].min()) * 0.5

        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)
        try: ax1.set_box_aspect([1, 1, 1])
        except AttributeError: pass
        ax1.set_title('3D Position & Orientation Trajectory')
        ax1.legend()

        # ---------------------------------------------------------
        # 2. 位置 vs 时间 (右上角)
        # ---------------------------------------------------------
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(time_axis, raw_pos[:, 0], color='r', linestyle=':', alpha=0.4)
        ax2.plot(time_axis, smooth_pos[:, 0], color='r', label='X')
        ax2.plot(time_axis, raw_pos[:, 1], color='g', linestyle=':', alpha=0.4)
        ax2.plot(time_axis, smooth_pos[:, 1], color='g', label='Y')
        ax2.plot(time_axis, raw_pos[:, 2], color='b', linestyle=':', alpha=0.4)
        ax2.plot(time_axis, smooth_pos[:, 2], color='b', label='Z')
        ax2.set_title('Position vs. Time')
        ax2.set_ylabel('Position (m)')
        ax2.grid(True)
        ax2.legend()

        # ---------------------------------------------------------
        # 3. 姿态 (欧拉角) vs 时间 (右下角)
        # ---------------------------------------------------------
        ax3 = fig.add_subplot(gs[1, 1])
        # 将四元数转换为欧拉角进行直观显示 (使用 XYZ 顺序，即 Roll-Pitch-Yaw)
        raw_euler = R.from_quat(raw_quat).as_euler('xyz', degrees=True)
        smooth_euler = R.from_quat(smooth_quat).as_euler('xyz', degrees=True)
        
        # 处理可能的万向锁跳变（为了画图好看，展开跳变）
        raw_euler = np.unwrap(raw_euler, period=360, axis=0)
        smooth_euler = np.unwrap(smooth_euler, period=360, axis=0)

        ax3.plot(time_axis, raw_euler[:, 0], color='r', linestyle=':', alpha=0.4)
        ax3.plot(time_axis, smooth_euler[:, 0], color='r', label='Roll (X)')
        ax3.plot(time_axis, raw_euler[:, 1], color='g', linestyle=':', alpha=0.4)
        ax3.plot(time_axis, smooth_euler[:, 1], color='g', label='Pitch (Y)')
        ax3.plot(time_axis, raw_euler[:, 2], color='b', linestyle=':', alpha=0.4)
        ax3.plot(time_axis, smooth_euler[:, 2], color='b', label='Yaw (Z)')
        
        ax3.set_title('Orientation (Euler Angles) vs. Time')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (degrees)')
        ax3.grid(True)
        ax3.legend()

        plt.tight_layout()
        plt.show()

    # ===============================
    # 定时器回调函数：后台执行发布 (200Hz)
    # ===============================
    def timer_callback(self, event):
        if self.is_finished or rospy.is_shutdown():
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.frame_id

        # 只要 self.is_executing 为 False，self.index 就始终为 0，即持续发布起点
        row = self.trajectory[self.index]

        pose_msg.pose.position.x = float(row[0])
        pose_msg.pose.position.y = float(row[1])
        pose_msg.pose.position.z = float(row[2])
        pose_msg.pose.orientation.x = float(row[3])
        pose_msg.pose.orientation.y = float(row[4])
        pose_msg.pose.orientation.z = float(row[5])
        pose_msg.pose.orientation.w = float(row[6])

        self.pub.publish(pose_msg)

        # 如果通过了安全确认，开始推进轨迹
        if self.is_executing:
            self.index += 1
            if self.index >= self.total_steps:
                self.is_finished = True
                rospy.loginfo("Trajectory successfully finished.")
                rospy.signal_shutdown("Execution Complete")


if __name__ == "__main__":
    try:
        node = DemoTrajectoryPublisher()
        
        # 1. 阻塞可视化
        node.visualize_trajectory()
        
        # 2. 图表关闭后，启动 200Hz 的后台定时器
        # 此时 self.is_executing 默认为 False，定时器会死死锁在起点 (index=0) 持续发布
        timer_duration = 1.0 / 200.0
        rospy.Timer(rospy.Duration(timer_duration), node.timer_callback)
        rospy.loginfo("\n>>> Currently publishing START POSE at 200Hz. Robot should be holding its position.")
        
        # 3. 阻塞等待用户指令
        user_input = input("\n[SAFETY CHECK] Robot is holding the start pose.\nIs the pose correct? Type 'y' to START moving, or any other key to ABORT: ")
        
        if user_input.strip().lower() == 'y':
            rospy.loginfo("Execution confirmed! Tracking trajectory...")
            # 释放执行锁，定时器回调中的 index 开始递增
            node.is_executing = True
            
            # 保持主线程存活，直到轨迹走完触发 rospy.signal_shutdown()
            rospy.spin() 
        else:
            rospy.logwarn("Execution aborted by user.")
            rospy.signal_shutdown("User Abort")
            
    except rospy.ROSInterruptException:
        pass