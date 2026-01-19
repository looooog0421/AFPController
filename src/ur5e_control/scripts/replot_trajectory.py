#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
读取trajectory_actual.txt和trajectory_reference.txt文件，对比可视化轨迹和速度信息
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
from scipy.ndimage import uniform_filter1d


class TrajectoryReplotter:
    def __init__(self, actual_file='trajectory_actual.txt', reference_file='trajectory_reference.txt'):
        """初始化轨迹重新绘图器
        
        Args:
            actual_file: 实际轨迹数据文件路径
            reference_file: 参考轨迹数据文件路径
        """
        self.actual_file = actual_file
        self.reference_file = reference_file
        
        # 实际轨迹数据
        self.actual_data = None
        self.actual_timestamps = None
        self.actual_positions = None
        self.actual_velocities = None
        self.actual_speed = None
        
        # 参考轨迹数据
        self.ref_data = None
        self.ref_timestamps = None
        self.ref_positions = None
        self.ref_velocities = None
        self.ref_speed = None
        
        # 创建图形
        self.fig = plt.figure(figsize=(18, 12))
        
        print("=" * 60)
        print("Trajectory Replotter (Comparison Mode) Initialized")
        print(f"Actual trajectory: {actual_file}")
        print(f"Reference trajectory: {reference_file}")
        print("=" * 60)
    
    def compute_velocity(self, timestamps, positions):
        """计算平滑速度
        
        Args:
            timestamps: 时间戳数组
            positions: 位置数组 (N x 3)
        
        Returns:
            velocities: 速度数组 (N x 3)
            speed: 速度大小数组 (N,)
        """
        # 对位置进行平滑
        window_size = 5
        positions_smooth = uniform_filter1d(positions, size=window_size, axis=0, mode='nearest')
        
        # 使用中心差分计算速度
        vx = np.zeros(len(timestamps))
        vy = np.zeros(len(timestamps))
        vz = np.zeros(len(timestamps))
        
        # 中间点使用中心差分
        for i in range(1, len(timestamps) - 1):
            dt_central = timestamps[i+1] - timestamps[i-1]
            if dt_central > 0:
                vx[i] = (positions_smooth[i+1, 0] - positions_smooth[i-1, 0]) / dt_central
                vy[i] = (positions_smooth[i+1, 1] - positions_smooth[i-1, 1]) / dt_central
                vz[i] = (positions_smooth[i+1, 2] - positions_smooth[i-1, 2]) / dt_central
        
        # 首尾点使用前向/后向差分
        if len(timestamps) > 1:
            dt0 = timestamps[1] - timestamps[0]
            if dt0 > 0:
                vx[0] = (positions_smooth[1, 0] - positions_smooth[0, 0]) / dt0
                vy[0] = (positions_smooth[1, 1] - positions_smooth[0, 1]) / dt0
                vz[0] = (positions_smooth[1, 2] - positions_smooth[0, 2]) / dt0
            
            dt_end = timestamps[-1] - timestamps[-2]
            if dt_end > 0:
                vx[-1] = (positions_smooth[-1, 0] - positions_smooth[-2, 0]) / dt_end
                vy[-1] = (positions_smooth[-1, 1] - positions_smooth[-2, 1]) / dt_end
                vz[-1] = (positions_smooth[-1, 2] - positions_smooth[-2, 2]) / dt_end
        
        velocities = np.column_stack([vx, vy, vz])
        
        # 对速度再进行一次平滑
        velocity_smooth_window = 3
        velocities = uniform_filter1d(velocities, size=velocity_smooth_window, axis=0, mode='nearest')
        
        # 计算总速度
        speed = np.sqrt(np.sum(velocities**2, axis=1))
        
        return velocities, speed
    
    def load_trajectory(self):
        """加载轨迹数据"""
        success = True
        
        # 加载实际轨迹
        if os.path.exists(self.actual_file):
            try:
                self.actual_data = np.loadtxt(self.actual_file, skiprows=1)
                if self.actual_data.size > 0:
                    self.actual_timestamps = self.actual_data[:, 0]
                    self.actual_positions = self.actual_data[:, 1:4]
                    self.actual_velocities, self.actual_speed = self.compute_velocity(
                        self.actual_timestamps, self.actual_positions
                    )
                    print(f"✓ Loaded {len(self.actual_timestamps)} actual trajectory points")
                    print(f"  Time range: {self.actual_timestamps[0]:.2f}s - {self.actual_timestamps[-1]:.2f}s")
                    print(f"  Max velocity: {np.nanmax(self.actual_speed):.4f} m/s")
                else:
                    print("⚠ Actual trajectory file is empty")
            except Exception as e:
                print(f"⚠ Error loading actual trajectory: {e}")
                success = False
        else:
            print(f"⚠ Actual trajectory file not found: {self.actual_file}")
        
        # 加载参考轨迹
        if os.path.exists(self.reference_file):
            try:
                self.ref_data = np.loadtxt(self.reference_file, skiprows=1)
                if self.ref_data.size > 0:
                    self.ref_timestamps = self.ref_data[:, 0]
                    self.ref_positions = self.ref_data[:, 1:4]
                    self.ref_velocities, self.ref_speed = self.compute_velocity(
                        self.ref_timestamps, self.ref_positions
                    )
                    print(f"✓ Loaded {len(self.ref_timestamps)} reference trajectory points")
                    print(f"  Time range: {self.ref_timestamps[0]:.2f}s - {self.ref_timestamps[-1]:.2f}s")
                    print(f"  Max velocity: {np.nanmax(self.ref_speed):.4f} m/s")
                else:
                    print("⚠ Reference trajectory file is empty")
            except Exception as e:
                print(f"⚠ Error loading reference trajectory: {e}")
        else:
            print(f"⚠ Reference trajectory file not found: {self.reference_file}")
        
        # 至少需要一个轨迹
        if self.actual_data is None and self.ref_data is None:
            print("❌ No trajectory data loaded")
            return False
        
        return success
    
    def setup_plots(self):
        """设置图形布局 (4行3列)"""
        # 第1行: 3D + XY + XZ
        self.ax_3d = self.fig.add_subplot(4, 3, 1, projection='3d')
        self.ax_xy = self.fig.add_subplot(4, 3, 2)
        self.ax_xz = self.fig.add_subplot(4, 3, 3)
        
        # 第2行: YZ + 速度X + 速度Y
        self.ax_yz = self.fig.add_subplot(4, 3, 4)
        self.ax_vx = self.fig.add_subplot(4, 3, 5)
        self.ax_vy = self.fig.add_subplot(4, 3, 6)
        
        # 第3行: 速度Z + 总速度 + 跟踪误差
        self.ax_vz = self.fig.add_subplot(4, 3, 7)
        self.ax_speed = self.fig.add_subplot(4, 3, 8)
        self.ax_error = self.fig.add_subplot(4, 3, 9)
        
        # 第4行: 误差XYZ分量
        self.ax_error_x = self.fig.add_subplot(4, 3, 10)
        self.ax_error_y = self.fig.add_subplot(4, 3, 11)
        self.ax_error_z = self.fig.add_subplot(4, 3, 12)
    
    def plot_trajectory(self):
        """绘制轨迹对比"""
        # 绘制参考轨迹（绿色）
        if self.ref_positions is not None:
            rx, ry, rz = self.ref_positions[:, 0], self.ref_positions[:, 1], self.ref_positions[:, 2]
            
            # 3D
            self.ax_3d.plot(rx, ry, rz, 'g-', linewidth=2, alpha=0.7, label='Reference')
            self.ax_3d.scatter(rx[0], ry[0], rz[0], c='g', s=100, marker='s', 
                              edgecolors='black', linewidths=2, zorder=5)
            
            # XY
            self.ax_xy.plot(rx, ry, 'g-', linewidth=2, alpha=0.7, label='Reference')
            self.ax_xy.scatter(rx[0], ry[0], c='g', s=80, marker='s', edgecolors='black', linewidths=2)
            
            # XZ
            self.ax_xz.plot(rx, rz, 'g-', linewidth=2, alpha=0.7, label='Reference')
            self.ax_xz.scatter(rx[0], rz[0], c='g', s=80, marker='s', edgecolors='black', linewidths=2)
            
            # YZ
            self.ax_yz.plot(ry, rz, 'g-', linewidth=2, alpha=0.7, label='Reference')
            self.ax_yz.scatter(ry[0], rz[0], c='g', s=80, marker='s', edgecolors='black', linewidths=2)
        
        # 绘制实际轨迹（蓝色）
        if self.actual_positions is not None:
            ax, ay, az = self.actual_positions[:, 0], self.actual_positions[:, 1], self.actual_positions[:, 2]
            
            # 3D
            self.ax_3d.plot(ax, ay, az, 'b-', linewidth=1.5, alpha=0.8, label='Actual')
            self.ax_3d.scatter(ax[0], ay[0], az[0], c='b', s=100, marker='o', 
                              edgecolors='black', linewidths=2, zorder=5)
            self.ax_3d.scatter(ax[-1], ay[-1], az[-1], c='r', s=100, marker='o', 
                              edgecolors='black', linewidths=2, label='End', zorder=5)
            
            # XY
            self.ax_xy.plot(ax, ay, 'b-', linewidth=1.5, alpha=0.8, label='Actual')
            self.ax_xy.scatter(ax[0], ay[0], c='b', s=80, marker='o', edgecolors='black', linewidths=2)
            self.ax_xy.scatter(ax[-1], ay[-1], c='r', s=80, marker='o', edgecolors='black', linewidths=2)
            
            # XZ
            self.ax_xz.plot(ax, az, 'b-', linewidth=1.5, alpha=0.8, label='Actual')
            self.ax_xz.scatter(ax[0], az[0], c='b', s=80, marker='o', edgecolors='black', linewidths=2)
            self.ax_xz.scatter(ax[-1], az[-1], c='r', s=80, marker='o', edgecolors='black', linewidths=2)
            
            # YZ
            self.ax_yz.plot(ay, az, 'b-', linewidth=1.5, alpha=0.8, label='Actual')
            self.ax_yz.scatter(ay[0], az[0], c='b', s=80, marker='o', edgecolors='black', linewidths=2)
            self.ax_yz.scatter(ay[-1], az[-1], c='r', s=80, marker='o', edgecolors='black', linewidths=2)
        
        # 设置标签和图例
        self.ax_3d.set_xlabel('X (m)', fontsize=10)
        self.ax_3d.set_ylabel('Y (m)', fontsize=10)
        self.ax_3d.set_zlabel('Z (m)', fontsize=10)
        self.ax_3d.set_title('3D Trajectory Comparison', fontsize=12, fontweight='bold')
        self.ax_3d.legend(fontsize=9)
        self.ax_3d.grid(True, alpha=0.3)
        
        self.ax_xy.set_xlabel('X (m)', fontsize=10)
        self.ax_xy.set_ylabel('Y (m)', fontsize=10)
        self.ax_xy.set_title('XY Plane', fontsize=12, fontweight='bold')
        self.ax_xy.legend(fontsize=9)
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        self.ax_xz.set_xlabel('X (m)', fontsize=10)
        self.ax_xz.set_ylabel('Z (m)', fontsize=10)
        self.ax_xz.set_title('XZ Plane', fontsize=12, fontweight='bold')
        self.ax_xz.legend(fontsize=9)
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')
        
        self.ax_yz.set_xlabel('Y (m)', fontsize=10)
        self.ax_yz.set_ylabel('Z (m)', fontsize=10)
        self.ax_yz.set_title('YZ Plane', fontsize=12, fontweight='bold')
        self.ax_yz.legend(fontsize=9)
        self.ax_yz.grid(True, alpha=0.3)
        self.ax_yz.set_aspect('equal')
    
    def plot_velocities(self):
        """绘制速度对比"""
        # X轴速度
        if self.ref_velocities is not None:
            self.ax_vx.plot(self.ref_timestamps, self.ref_velocities[:, 0], 
                           'g-', linewidth=2, alpha=0.7, label='Reference Vx')
        if self.actual_velocities is not None:
            self.ax_vx.plot(self.actual_timestamps, self.actual_velocities[:, 0], 
                           'b-', linewidth=1.5, alpha=0.8, label='Actual Vx')
        self.ax_vx.set_ylabel('Vx (m/s)', fontsize=10)
        self.ax_vx.set_title('X-axis Velocity', fontsize=12, fontweight='bold')
        self.ax_vx.grid(True, alpha=0.3)
        self.ax_vx.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax_vx.legend(fontsize=8)
        
        # Y轴速度
        if self.ref_velocities is not None:
            self.ax_vy.plot(self.ref_timestamps, self.ref_velocities[:, 1], 
                           'g-', linewidth=2, alpha=0.7, label='Reference Vy')
        if self.actual_velocities is not None:
            self.ax_vy.plot(self.actual_timestamps, self.actual_velocities[:, 1], 
                           'b-', linewidth=1.5, alpha=0.8, label='Actual Vy')
        self.ax_vy.set_ylabel('Vy (m/s)', fontsize=10)
        self.ax_vy.set_title('Y-axis Velocity', fontsize=12, fontweight='bold')
        self.ax_vy.grid(True, alpha=0.3)
        self.ax_vy.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax_vy.legend(fontsize=8)
        
        # Z轴速度
        if self.ref_velocities is not None:
            self.ax_vz.plot(self.ref_timestamps, self.ref_velocities[:, 2], 
                           'g-', linewidth=2, alpha=0.7, label='Reference Vz')
        if self.actual_velocities is not None:
            self.ax_vz.plot(self.actual_timestamps, self.actual_velocities[:, 2], 
                           'b-', linewidth=1.5, alpha=0.8, label='Actual Vz')
        self.ax_vz.set_xlabel('Time (s)', fontsize=10)
        self.ax_vz.set_ylabel('Vz (m/s)', fontsize=10)
        self.ax_vz.set_title('Z-axis Velocity', fontsize=12, fontweight='bold')
        self.ax_vz.grid(True, alpha=0.3)
        self.ax_vz.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax_vz.legend(fontsize=8)
    
    def plot_total_speed(self):
        """绘制总速度对比"""
        if self.ref_speed is not None:
            self.ax_speed.plot(self.ref_timestamps, self.ref_speed, 
                              'g-', linewidth=2, alpha=0.7, label='Reference')
            ref_max = np.nanmax(self.ref_speed)
            ref_mean = np.nanmean(self.ref_speed)
            self.ax_speed.axhline(y=ref_max, color='g', linestyle='--', linewidth=1, alpha=0.4)
            self.ax_speed.axhline(y=ref_mean, color='lightgreen', linestyle='--', linewidth=1, alpha=0.4)
        
        if self.actual_speed is not None:
            self.ax_speed.plot(self.actual_timestamps, self.actual_speed, 
                              'b-', linewidth=1.5, alpha=0.8, label='Actual')
            actual_max = np.nanmax(self.actual_speed)
            actual_mean = np.nanmean(self.actual_speed)
            self.ax_speed.axhline(y=actual_max, color='r', linestyle='--', linewidth=1, alpha=0.5, 
                                 label=f'Actual Max: {actual_max:.4f} m/s')
            self.ax_speed.axhline(y=actual_mean, color='orange', linestyle='--', linewidth=1, alpha=0.5,
                                 label=f'Actual Mean: {actual_mean:.4f} m/s')
        
        self.ax_speed.set_xlabel('Time (s)', fontsize=10)
        self.ax_speed.set_ylabel('Speed (m/s)', fontsize=10)
        self.ax_speed.set_title('Total Speed Comparison', fontsize=12, fontweight='bold')
        self.ax_speed.grid(True, alpha=0.3)
        self.ax_speed.legend(fontsize=8, loc='upper right')
    
    def plot_tracking_error(self):
        """绘制跟踪误差"""
        if self.actual_positions is None or self.ref_positions is None:
            self.ax_error.text(0.5, 0.5, 'No tracking error\n(missing trajectory)', 
                              ha='center', va='center', fontsize=12)
            self.ax_error.set_title('Tracking Error', fontsize=12, fontweight='bold')
            
            # 清空误差分量图
            for ax in [self.ax_error_x, self.ax_error_y, self.ax_error_z]:
                ax.text(0.5, 0.5, 'N/A', ha='center', va='center', fontsize=12)
            return
        
        # 时间对齐：使用实际轨迹的时间戳，插值参考轨迹
        from scipy.interpolate import interp1d
        
        try:
            # 检查时间范围重叠
            ref_interp_x = interp1d(self.ref_timestamps, self.ref_positions[:, 0], 
                                   kind='linear', fill_value='extrapolate')
            ref_interp_y = interp1d(self.ref_timestamps, self.ref_positions[:, 1], 
                                   kind='linear', fill_value='extrapolate')
            ref_interp_z = interp1d(self.ref_timestamps, self.ref_positions[:, 2], 
                                   kind='linear', fill_value='extrapolate')
            
            # 在实际轨迹的时间点插值参考轨迹
            ref_interp_pos = np.column_stack([
                ref_interp_x(self.actual_timestamps),
                ref_interp_y(self.actual_timestamps),
                ref_interp_z(self.actual_timestamps)
            ])
            
            # 计算误差
            error_vec = self.actual_positions - ref_interp_pos
            error_norm = np.linalg.norm(error_vec, axis=1)
            
            # 绘制总误差
            self.ax_error.plot(self.actual_timestamps, error_norm * 1000, 'r-', linewidth=1.5)
            self.ax_error.fill_between(self.actual_timestamps, error_norm * 1000, alpha=0.3, color='red')
            self.ax_error.set_xlabel('Time (s)', fontsize=10)
            self.ax_error.set_ylabel('Error (mm)', fontsize=10)
            self.ax_error.set_title(f'Tracking Error (Max: {np.max(error_norm)*1000:.2f}mm, Mean: {np.mean(error_norm)*1000:.2f}mm)', 
                                   fontsize=11, fontweight='bold')
            self.ax_error.grid(True, alpha=0.3)
            
            # 绘制误差分量
            self.ax_error_x.plot(self.actual_timestamps, error_vec[:, 0] * 1000, 'r-', linewidth=1.5)
            self.ax_error_x.fill_between(self.actual_timestamps, error_vec[:, 0] * 1000, alpha=0.3, color='red')
            self.ax_error_x.set_ylabel('Error X (mm)', fontsize=10)
            self.ax_error_x.set_title('X-axis Error', fontsize=11, fontweight='bold')
            self.ax_error_x.grid(True, alpha=0.3)
            self.ax_error_x.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            
            self.ax_error_y.plot(self.actual_timestamps, error_vec[:, 1] * 1000, 'g-', linewidth=1.5)
            self.ax_error_y.fill_between(self.actual_timestamps, error_vec[:, 1] * 1000, alpha=0.3, color='green')
            self.ax_error_y.set_ylabel('Error Y (mm)', fontsize=10)
            self.ax_error_y.set_title('Y-axis Error', fontsize=11, fontweight='bold')
            self.ax_error_y.grid(True, alpha=0.3)
            self.ax_error_y.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            
            self.ax_error_z.plot(self.actual_timestamps, error_vec[:, 2] * 1000, 'b-', linewidth=1.5)
            self.ax_error_z.fill_between(self.actual_timestamps, error_vec[:, 2] * 1000, alpha=0.3, color='blue')
            self.ax_error_z.set_xlabel('Time (s)', fontsize=10)
            self.ax_error_z.set_ylabel('Error Z (mm)', fontsize=10)
            self.ax_error_z.set_title('Z-axis Error', fontsize=11, fontweight='bold')
            self.ax_error_z.grid(True, alpha=0.3)
            self.ax_error_z.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            
        except Exception as e:
            print(f"⚠ Error computing tracking error: {e}")
            self.ax_error.text(0.5, 0.5, f'Error computing\ntracking error:\n{str(e)}', 
                              ha='center', va='center', fontsize=10)
    
    def plot(self):
        """绘制所有图形"""
        if self.actual_data is None and self.ref_data is None:
            return False
        
        self.setup_plots()
        self.plot_trajectory()
        self.plot_velocities()
        self.plot_total_speed()
        self.plot_tracking_error()
        
        plt.tight_layout()
        
        # 打印统计信息
        print("\n" + "=" * 60)
        print("Plotting Statistics:")
        print("=" * 60)
        
        if self.actual_positions is not None:
            print(f"Actual Trajectory:")
            print(f"  Points: {len(self.actual_timestamps)}")
            print(f"  Duration: {self.actual_timestamps[-1] - self.actual_timestamps[0]:.2f}s")
            print(f"  Max speed: {np.nanmax(self.actual_speed):.4f} m/s")
            print(f"  Mean speed: {np.nanmean(self.actual_speed):.4f} m/s")
        
        if self.ref_positions is not None:
            print(f"\nReference Trajectory:")
            print(f"  Points: {len(self.ref_timestamps)}")
            print(f"  Duration: {self.ref_timestamps[-1] - self.ref_timestamps[0]:.2f}s")
            print(f"  Max speed: {np.nanmax(self.ref_speed):.4f} m/s")
            print(f"  Mean speed: {np.nanmean(self.ref_speed):.4f} m/s")
        
        print("=" * 60)
        
        return True
    
    def save_figure(self, filename='trajectory_comparison.png'):
        """保存图形"""
        try:
            self.fig.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"✓ Figure saved to {filename}")
        except Exception as e:
            print(f"❌ Error saving figure: {e}")
    
    def show(self):
        """显示图形"""
        plt.show()


if __name__ == '__main__':
    # 检查命令行参数
    if len(sys.argv) == 3:
        actual_file = sys.argv[1]
        reference_file = sys.argv[2]
    elif len(sys.argv) == 2:
        # 如果只提供一个参数，尝试自动推断
        base_file = sys.argv[1]
        if '_actual' in base_file:
            actual_file = base_file
            reference_file = base_file.replace('_actual', '_reference')
        elif '_reference' in base_file:
            reference_file = base_file
            actual_file = base_file.replace('_reference', '_actual')
        else:
            actual_file = base_file.replace('.txt', '_actual.txt')
            reference_file = base_file.replace('.txt', '_reference.txt')
    else:
        # 默认路径
        actual_file = '/home/lgx/Project/AFP/trajectory_actual.txt'
        reference_file = '/home/lgx/Project/AFP/trajectory_reference.txt'
    
    # 创建重新绘图器
    replotter = TrajectoryReplotter(actual_file, reference_file)
    
    # 加载数据
    if replotter.load_trajectory():
        # 绘制
        if replotter.plot():
            # 保存图形
            replotter.save_figure()
            # 显示
            replotter.show()
    else:
        print("Failed to load trajectory data")
        sys.exit(1)