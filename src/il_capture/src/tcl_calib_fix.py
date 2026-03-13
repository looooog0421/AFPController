import h5py
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def load_hdf5_data(file_path, demo_name='demo_0'):
    """从 HDF5 文件中读取末端位置和四元数"""
    print(f"正在读取 {file_path} 中的 {demo_name} 数据...")
    with h5py.File(file_path, 'r') as f:
        pos_path = f'data/{demo_name}/obs/robot0_eef_pos'
        quat_path = f'data/{demo_name}/obs/robot0_eef_quat'
        
        pos_data = f[pos_path][:]
        quat_data = f[quat_path][:]
        
    return pos_data, quat_data

def visualize_trajectory(pos_data, quat_data):
    """可视化全局位置和姿态，方便选择数据段 (已加入 unwrap 处理)"""
    rotations = R.from_quat(quat_data)
    
    # 消除欧拉角 2*pi 跳变
    euler_angles_rad = rotations.as_euler('xyz', degrees=False)
    euler_angles_rad_unwrapped = np.unwrap(euler_angles_rad, axis=0)
    euler_angles = np.rad2deg(euler_angles_rad_unwrapped)
    
    time_steps = np.arange(len(pos_data))
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    ax1.plot(time_steps, pos_data[:, 0], label='X')
    ax1.plot(time_steps, pos_data[:, 1], label='Y (Look for flat segment)', linewidth=2)
    ax1.plot(time_steps, pos_data[:, 2], label='Z (Look for flat segment)', linewidth=2)
    ax1.set_ylabel('Position (m)')
    ax1.set_title('End-Effector Position')
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(time_steps, euler_angles[:, 0], label='Roll (X)')
    ax2.plot(time_steps, euler_angles[:, 1], label='Pitch (Y)')
    ax2.plot(time_steps, euler_angles[:, 2], label='Yaw (Z)')
    ax2.set_ylabel('Euler Angles (degrees)')
    ax2.set_xlabel('Sample Index')
    ax2.set_title('End-Effector Orientation (Continuous Unwrapped Angles)')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    print("请在弹出的图表中观察合适的起点和终点 Index。关闭图表后继续...")
    plt.show(block=True)

def calculate_yz_joint_calibration_error(Y_records, Z_records, R_flange_records):
    """联合 Y 和 Z 轴的约束，求解 Δx, Δy, Δz"""
    N = len(Y_records)
    # 2N 个方程，5 个未知数 [dx, dy, dz, Cy, Cz]
    A = np.zeros((2 * N, 5))
    B = np.zeros((2 * N, 1))
    
    for i in range(N):
        # Y 轴方程参数
        r21, r22, r23 = R_flange_records[i, 1, :]
        A[2*i, 0:3] = [r21, r22, r23]
        A[2*i, 3] = -1.0  # -Cy
        A[2*i, 4] = 0.0
        B[2*i, 0] = -Y_records[i]
        
        # Z 轴方程参数
        r31, r32, r33 = R_flange_records[i, 2, :]
        A[2*i+1, 0:3] = [r31, r32, r33]
        A[2*i+1, 3] = 0.0
        A[2*i+1, 4] = -1.0 # -Cz
        B[2*i+1, 0] = -Z_records[i]
        
    # 最小二乘法求解
    X_sol, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
    delta_x, delta_y, delta_z, C_y, C_z = X_sol.flatten()
    return delta_x, delta_y, delta_z, C_y, C_z

def plot_yz_correction_comparison(pos_seg, rot_matrices_seg, dx, dy, dz, Cy, Cz):
    """绘制选取区间内的 Y 轴和 Z 轴修正前后对比图"""
    N = len(pos_seg)
    delta_p = np.array([dx, dy, dz])
    
    # 批量计算修正量并加到原始位置上
    correction_vectors = np.matmul(rot_matrices_seg, delta_p)
    pos_corrected_seg = pos_seg + correction_vectors
    
    time_steps = np.arange(N)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Y 轴对比
    ax1.plot(time_steps, pos_seg[:, 1], label='Original Y (Fluctuating)', color='red', linestyle='--')
    ax1.plot(time_steps, pos_corrected_seg[:, 1], label='Corrected Y (Smoothed)', color='blue')
    ax1.axhline(y=Cy, color='green', linestyle=':', label=f'Fitted Constant Cy = {Cy:.5f}m')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Y-Axis Fluctuation Correction')
    ax1.legend()
    ax1.grid(True)
    
    # Z 轴对比
    ax2.plot(time_steps, pos_seg[:, 2], label='Original Z (Fluctuating)', color='red', linestyle='--')
    ax2.plot(time_steps, pos_corrected_seg[:, 2], label='Corrected Z (Smoothed)', color='blue')
    ax2.axhline(y=Cz, color='green', linestyle=':', label=f'Fitted Constant Cz = {Cz:.5f}m')
    ax2.set_ylabel('Z Position (m)')
    ax2.set_xlabel('Sample Index (in Segment)')
    ax2.set_title('Z-Axis Fluctuation Correction')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show(block=True) # 阻塞等待关闭

def plot_3d_full_trajectory_comparison(pos_original, rot_matrices_all, dx, dy, dz):
    """全局三维轨迹修正与可视化对比"""
    # 构造标定误差向量
    delta_p = np.array([dx, dy, dz])
    
    # 将误差投影到基坐标系并加到整条原始轨迹上
    correction_vectors = np.matmul(rot_matrices_all, delta_p)
    pos_corrected = pos_original + correction_vectors
    
    # 初始化 3D 图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制原始轨迹
    ax.plot(pos_original[:, 0], pos_original[:, 1], pos_original[:, 2], 
            label='Original Trajectory (With Calib Error)', color='red', linestyle='--', alpha=0.6, linewidth=1.5)
    
    # 绘制修正后的轨迹
    ax.plot(pos_corrected[:, 0], pos_corrected[:, 1], pos_corrected[:, 2], 
            label='Corrected Trajectory (Smoothed)', color='blue', linewidth=2.5)
    
    ax.set_title('Full 3D Trajectory Comparison\nVerify the spatial geometry of the mold')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    
    # 强制让 X/Y/Z 轴的显示比例为 1:1:1，防止视觉上的轨迹拉伸变形
    ax.set_box_aspect([1, 1, 1]) 
    
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 1. 配置文件路径
    hdf5_file = '/home/lgx/Project/AFP/src/il_capture/data/90_all_corrected_cleaned.hdf5' 
    demo_id = 'demo_0' 
    
    # 2. 加载数据
    try:
        pos, quat = load_hdf5_data(hdf5_file, demo_id)
    except Exception as e:
        print(f"读取文件失败: {e}")
        exit()
        
    # 3. 可视化全局轨迹找区间 (已修复姿态跳变)
    visualize_trajectory(pos, quat)
    
    try:
        start_idx = int(input("请输入提取区间的起点 Index: "))
        end_idx = int(input("请输入提取区间的终点 Index: "))
    except ValueError:
        print("输入无效，请输入整数。")
        exit()
        
    # 4. 提取数据段
    pos_seg = pos[start_idx:end_idx]
    quat_seg = quat[start_idx:end_idx]
    rot_matrices_seg = R.from_quat(quat_seg).as_matrix()
    
    Y_seg = pos_seg[:, 1]
    Z_seg = pos_seg[:, 2]
    
    # 5. 联合计算 Y 和 Z 的约束误差
    print(f"\n正在计算区间 [{start_idx}:{end_idx}] 的 Y/Z 轴联合标定误差...")
    dx, dy, dz, Cy, Cz = calculate_yz_joint_calibration_error(Y_seg, Z_seg, rot_matrices_seg)
    
    print("\n--- 计算结果 ---")
    print("求得的三维标定补偿量 (法兰坐标系下):")
    print(f"Δx = {dx:.6f} m")
    print(f"Δy = {dy:.6f} m")
    print(f"Δz = {dz:.6f} m")
    print(f"\n拟合出的常量约束高度:")
    print(f"理论恒定 Y (Cy) = {Cy:.6f} m")
    print(f"理论恒定 Z (Cz) = {Cz:.6f} m")
    
    # 6. 绘制 Y 轴和 Z 轴的修正前后对比图 (局部区间)
    print("\n正在生成局部区间 Y/Z 修正对比图...")
    plot_yz_correction_comparison(pos_seg, rot_matrices_seg, dx, dy, dz, Cy, Cz)
    
    # 7. 对整条轨迹进行全局 3D 修正与可视化
    print("\n正在生成全局 3D 轨迹修正与对比图...")
    # 获取整条轨迹对应的旋转矩阵
    rot_matrices_all = R.from_quat(quat).as_matrix() 
    plot_3d_full_trajectory_comparison(pos, rot_matrices_all, dx, dy, dz)