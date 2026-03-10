import h5py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.gridspec as gridspec # 新增：用于复杂排版
import shutil
import os
import sys
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R

def smooth_trajectory_sg(data, window_length=31, polyorder=3):
    """通用的 S-G 平滑函数"""
    if len(data) < window_length:
        window_length = len(data) if len(data) % 2 != 0 else len(data) - 1
        if window_length <= polyorder:
            return data
    return savgol_filter(data, window_length, polyorder, axis=0)

def apply_smoothing_logic(obs_pos, obs_quat, action_pos, action_quat, window_length=31):
    """核心平滑逻辑：拼接 -> 解包 -> 平滑 -> 拆分"""
    full_pos = np.vstack([obs_pos, action_pos[-1:]])
    full_quat = np.vstack([obs_quat, action_quat[-1:]])
    
    # 1. 位置平滑
    smoothed_full_pos = smooth_trajectory_sg(full_pos, window_length=window_length)
    
    # 2. 姿态平滑
    euler_rad = R.from_quat(full_quat).as_euler('xyz', degrees=False)
    euler_unwrapped = np.unwrap(euler_rad, axis=0) # 原始连续欧拉角(弧度)
    smoothed_euler = smooth_trajectory_sg(euler_unwrapped, window_length=window_length) # 平滑后欧拉角(弧度)
    smoothed_full_quat = R.from_euler('xyz', smoothed_euler, degrees=False).as_quat()
    
    # 3. 重新切分
    smoothed_obs_pos = smoothed_full_pos[:-1]
    smoothed_obs_quat = smoothed_full_quat[:-1]
    smoothed_action_pos = smoothed_full_pos[1:]
    smoothed_action_quat = smoothed_full_quat[1:]
    
    # 修改点：多返回了 euler_unwrapped 和 smoothed_euler 供 2D 绘图使用
    return smoothed_obs_pos, smoothed_obs_quat, smoothed_action_pos, smoothed_action_quat, full_pos, smoothed_full_pos, euler_unwrapped, smoothed_euler

def set_axes_equal_and_ticks(ax):
    """强制 3D 坐标轴的物理比例保持 1:1:1，并统一网格步长"""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    
    tick_step = np.round((2 * plot_radius) / 5, decimals=2)
    if tick_step == 0:
        tick_step = 0.01 
        
    locator = ticker.MultipleLocator(base=tick_step)
    ax.xaxis.set_major_locator(locator)
    ax.yaxis.set_major_locator(locator)
    ax.zaxis.set_major_locator(locator)
    
    return tick_step

def preview_single_demo_3d(input_file, demo_name='demo_0', window_length=31):
    """综合预览：左侧 3D 物理轨迹 1:1，右侧 2D 姿态欧拉角平滑对比"""
    print(f"--- 阶段 1: 预览单条轨迹 ({demo_name}) 平滑效果 ---")
    with h5py.File(input_file, 'r') as f:
        demo_grp = f[f'data/{demo_name}']
        obs_pos = demo_grp['obs/robot0_eef_pos'][:]
        obs_quat = demo_grp['obs/robot0_eef_quat'][:]
        action_with_wrench = demo_grp['action_with_wrench'][:]
        action_pos = action_with_wrench[:, 0:3]
        action_quat = action_with_wrench[:, 3:7]
        
    # 获取所有计算结果 (增加了欧拉角数据)
    _, _, _, _, original_full_pos, smoothed_full_pos, euler_orig, euler_smooth = apply_smoothing_logic(
        obs_pos, obs_quat, action_pos, action_quat, window_length=window_length
    )
    
    # --- 构建排版 (GridSpec) ---
    fig = plt.figure(figsize=(16, 9))
    gs = gridspec.GridSpec(3, 2, width_ratios=[1.2, 1]) # 左边 3D 图稍微宽一点
    
    # 1. 左侧：3D 位置图
    ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax_3d.plot(original_full_pos[:, 0], original_full_pos[:, 1], original_full_pos[:, 2], 
            label='Original Pos', color='red', linestyle='--', alpha=0.6)
    ax_3d.plot(smoothed_full_pos[:, 0], smoothed_full_pos[:, 1], smoothed_full_pos[:, 2], 
            label=f'Smoothed Pos', color='blue', linewidth=2)
    
    tick_step = set_axes_equal_and_ticks(ax_3d)
    ax_3d.set_box_aspect([1, 1, 1]) 
    ax_3d.set_title(f'3D Position (1:1:1 True Scale)\nGrid Spacing: {tick_step} m / cell')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.legend()

    # 2. 右侧：2D 欧拉角图 (将弧度转为角度显示更直观)
    euler_orig_deg = np.rad2deg(euler_orig)
    euler_smooth_deg = np.rad2deg(euler_smooth)
    time_steps = np.arange(len(euler_orig_deg))
    
    # Roll
    ax_roll = fig.add_subplot(gs[0, 1])
    ax_roll.plot(time_steps, euler_orig_deg[:, 0], 'r--', alpha=0.6, label='Original')
    ax_roll.plot(time_steps, euler_smooth_deg[:, 0], 'b-', linewidth=2, label='Smoothed')
    ax_roll.set_title('Orientation Smoothing Validation (Unwrapped)')
    ax_roll.set_ylabel('Roll (deg)')
    ax_roll.legend()
    ax_roll.grid(True)
    
    # Pitch
    ax_pitch = fig.add_subplot(gs[1, 1], sharex=ax_roll)
    ax_pitch.plot(time_steps, euler_orig_deg[:, 1], 'r--', alpha=0.6)
    ax_pitch.plot(time_steps, euler_smooth_deg[:, 1], 'b-', linewidth=2)
    ax_pitch.set_ylabel('Pitch (deg)')
    ax_pitch.grid(True)
    
    # Yaw
    ax_yaw = fig.add_subplot(gs[2, 1], sharex=ax_roll)
    ax_yaw.plot(time_steps, euler_orig_deg[:, 2], 'r--', alpha=0.6)
    ax_yaw.plot(time_steps, euler_smooth_deg[:, 2], 'b-', linewidth=2)
    ax_yaw.set_ylabel('Yaw (deg)')
    ax_yaw.set_xlabel('Sample Index')
    ax_yaw.grid(True)
    
    plt.tight_layout()
    print(f"图表已生成。当前统一网格间距被设置为: {tick_step} m/格")
    print("请在弹出的图表中检查 XYZ 空间和 Roll/Pitch/Yaw 的平滑效果。关闭图表后在终端进行确认...")
    plt.show(block=True)

def process_entire_dataset(input_file, output_file, window_length=31):
    """确认无误后，批量处理整个数据集"""
    print(f"\n--- 阶段 2: 开始批量处理整个数据集 ---")
    print(f"正在复制原始数据到 {output_file}...")
    shutil.copy2(input_file, output_file)
    
    with h5py.File(output_file, 'r+') as f:
        data_group = f['data']
        demo_keys = list(data_group.keys())
        
        print(f"共发现 {len(demo_keys)} 个 demo，开始覆写平滑数据...")
        
        for demo in demo_keys:
            demo_grp = data_group[demo]
            
            obs_pos = demo_grp['obs/robot0_eef_pos'][:]
            obs_quat = demo_grp['obs/robot0_eef_quat'][:]
            action_with_wrench = demo_grp['action_with_wrench'][:]
            action_pos = action_with_wrench[:, 0:3]
            action_quat = action_with_wrench[:, 3:7]
            
            # 接收 8 个返回值，我们只用前 4 个覆盖 HDF5
            s_obs_pos, s_obs_quat, s_act_pos, s_act_quat, _, _, _, _ = apply_smoothing_logic(
                obs_pos, obs_quat, action_pos, action_quat, window_length=window_length
            )
            
            demo_grp['obs/robot0_eef_pos'][...] = s_obs_pos
            demo_grp['obs/robot0_eef_quat'][...] = s_obs_quat
            
            demo_grp['action_with_wrench'][:, 0:3] = s_act_pos
            demo_grp['action_with_wrench'][:, 3:7] = s_act_quat
            
            if 'action_without_wrench' in demo_grp:
                demo_grp['action_without_wrench'][:, 0:3] = s_act_pos
                demo_grp['action_without_wrench'][:, 3:7] = s_act_quat
                
        print(f"\n✅ 全部 {len(demo_keys)} 条轨迹处理完成！")
        print(f"✅ 平滑后的干净数据集已保存至: {output_file}")

if __name__ == "__main__":
    # 1. 配置文件路径
    input_hdf5 = '/home/lgx/Project/AFP/src/il_capture/data/120_all_corrected_cleaned.hdf5'
    
    base_name, ext = os.path.splitext(input_hdf5)
    output_hdf5 = f"{base_name}_smoothed{ext}"
    
    # 2. 参数设置
    WINDOW_LENGTH = 51 
    
    # 3. 预览单条轨迹
    try:
        preview_single_demo_3d(input_hdf5, demo_name='demo_10', window_length=WINDOW_LENGTH)
    except Exception as e:
        print(f"预览出错: {e}")
        sys.exit(1)
        
    # 4. 终端交互确认
    while True:
        user_input = input("\n平滑效果是否符合预期？\n- 输入 'y' 确认并应用到整个数据集\n- 输入 'n' 放弃并退出程序\n请输入 (y/n): ").strip().lower()
        
        if user_input == 'y':
            process_entire_dataset(input_hdf5, output_hdf5, window_length=WINDOW_LENGTH)
            break
        elif user_input == 'n':
            print("\n❌ 已取消批量处理。您可以修改代码中的 WINDOW_LENGTH 参数后重新运行测试。")
            sys.exit(0)
        else:
            print("输入无效，请重新输入 'y' 或 'n'。")