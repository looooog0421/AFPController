import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class RobotDataLoader:
    def __init__(self, file_path):
        self.df = pd.read_csv(file_path)
        # 将时间戳归一化从0开始
        self.df['time_relative'] = self.df['time_stamp'] - self.df['time_stamp'].iloc[0]
        self.process_extra_data()

    def process_extra_data(self):
        """计算一些绘图常用的派生数据，比如跟踪误差"""
        # 1. 计算末端位置误差 (Actual vs Reference)
        for axis in ['x', 'y', 'z']:
            self.df[f'err_{axis}'] = self.df[f'ref_pose_{axis}'] - self.df[f'ee_pose_{axis}']
        
        # 2. 将四元数转换为欧拉角 (以 ee_pose 为例，方便观察姿态)
        # 结果为 [roll, pitch, yaw] 角度单位
        quats = self.df[['ee_pose_qx', 'ee_pose_qy', 'ee_pose_qz', 'ee_pose_qw']].values
        euler_angles = R.from_quat(quats).as_euler('xyz', degrees=True)
        self.df[['roll', 'pitch', 'yaw']] = euler_angles

    def get_column(self, name):
        return self.df[name].values

    def get_time(self):
        return self.df['time_relative'].values
    
class RobotDataPlotter:
    @staticmethod
    def plot_trajectory_comparison(loader):
        """对比参考轨迹与实际轨迹 (X, Y, Z 三轴)"""
        time = loader.get_time()
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].set_title("End-Effector Trajectory Tracking")
        
        for i, axis in enumerate(['x', 'y', 'z']):
            axes[i].plot(time, loader.get_column(f'ref_pose_{axis}'), 'r--', label='Ref')
            axes[i].plot(time, loader.get_column(f'ee_pose_{axis}'), 'b-', label='Actual')
            axes[i].plot(time, loader.get_column(f'pin_{axis}'), 'g:', label='Pinocchio') # 验证FK
            axes[i].set_ylabel(f'{axis.upper()} [m]')
            axes[i].legend(loc='upper right')
            axes[i].grid(True)
        
        axes[2].set_xlabel("Time [s]")
        plt.tight_layout()
        plt.show()

    @staticmethod
    def plot_tracking_errors(loader):
        """绘制三轴跟踪误差"""
        time = loader.get_time()
        plt.figure(figsize=(10, 5))
        for axis in ['x', 'y', 'z']:
            plt.plot(time, loader.get_column(f'err_{axis}'), label=f'Error {axis.upper()}')
        
        plt.title("Position Tracking Errors")
        plt.xlabel("Time [s]")
        plt.ylabel("Error [m]")
        plt.legend()
        plt.grid(True)
        plt.show()

    @staticmethod
    def plot_forces(loader):
        """绘制末端受力 (F_x, F_y, F_z)"""
        time = loader.get_time()
        plt.figure(figsize=(10, 5))
        for axis in ['fx', 'fy', 'fz']:
            plt.plot(time, loader.get_column(axis), label=axis.upper())
        
        plt.title("End-Effector Forces")
        plt.xlabel("Time [s]")
        plt.ylabel("Force [N]")
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # 1. 加载文件
    data_file = "/home/lgx/Project/AFP/src/il_capture/data/experiments/600.csv" # 替换为你的文件名
    try:
        loader = RobotDataLoader(data_file)
        
        # 2. 调用绘图类执行不同的绘图任务
        plotter = RobotDataPlotter()
        
        # 任务 A: 查看轨迹对比（Ref vs Actual vs Pinocchio）
        plotter.plot_trajectory_comparison(loader)
        
        # 任务 B: 查看误差曲线
        plotter.plot_tracking_errors(loader)
        
        # 任务 C: 查看力传感器数据
        plotter.plot_forces(loader)
        
    except FileNotFoundError:
        print(f"Error: {data_file} not found. Please check the file path.")