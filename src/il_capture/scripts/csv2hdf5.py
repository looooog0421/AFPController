import h5py
import numpy as np
import pandas as pd
import os
import glob
import json
import re
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
# =========== 配置 ===========
# CSV文件目录
csv_folder = "/home/lgx/Project/AFP/src/il_capture/data/2025-12-08-17-35-45"
# csv表头：time,ee_x,ee_y,ee_z,ee_qx,ee_qy,ee_qz,ee_qw,ee_fx,ee_fy,ee_fz,ee_tx,ee_ty,ee_tz,raw_fx,raw_fy,raw_fz,raw_tx,raw_ty,raw_tz



# 输出HDF5文件路径
output_hdf5_path = "/home/lgx/Project/AFP/src/il_capture/data/2025-12-08-17-35-45/layup_robomimic.hdf5"

# 目标频率
TARGET_FREQ = 30.0  # Hz
dt = 1.0 / TARGET_FREQ
# 训练集比例
train_ratio = 0.9

# ============================

def get_file_index(filename):
    """
    从文件名 segment_001.csv 中提取数字 1。
    确保 segment_2 不会排在 segment_10 后面。
    """
    match = re.search(r'segment_(\d+).csv', filename)
    if match:
        return int(match.group(1))
    return -1

def resample_data(df, target_freq):
    """
    重采样数据到目标频率
    """
    # 1. 获取原始时间轴
    original_time = df['time'].values
    original_time = original_time - original_time[0]  # 归一化时间从0开始

    # 2. 创建目标时间轴
    duration = original_time[-1]
    new_time = np.arange(0, duration, 1.0 / target_freq)

    # 处理特殊情况：数据太少
    if len(original_time) < 2:
        # 数据点太少，无法插值
        return None
    
    # 3. 准备插值的数据列
    pos = df[['ee_x', 'ee_y', 'ee_z']].values
    quat = df[['ee_qx', 'ee_qy', 'ee_qz', 'ee_qw']].values
    wrench = df[['ee_fx', 'ee_fy', 'ee_fz', 'ee_tx', 'ee_ty', 'ee_tz']].values

    # 4. 插值函数
    pos_interp = interp1d(original_time, pos, axis=0, kind='linear', fill_value="extrapolate")
    wrench_interp = interp1d(original_time, wrench, axis=0, kind='linear', fill_value="extrapolate")

    new_pos = pos_interp(new_time)
    new_wrench = wrench_interp(new_time)

    # 四元数插值使用 Slerp
    # 1. 创建 Rotation 对象
    rotations = R.from_quat(quat)
    
    # 2. 创建 Slerp 插值器 (注意：这里直接使用 Slerp 类，而不是 R.Slerp)
    try:
        slerp = Slerp(original_time, rotations)
        new_rotations = slerp(new_time)
        new_quat = new_rotations.as_quat()
    except Exception as e:
        print(f"Slerp failed: {e}. Falling back to linear interpolation.")
        # 如果 Slerp 失败 (例如 scipy 版本过低)，回退到线性插值 + 归一化
        f_quat = interp1d(original_time, quat, axis=0, kind='linear', fill_value="extrapolate")
        new_quat = f_quat(new_time)
        norms = np.linalg.norm(new_quat, axis=1, keepdims=True)
        new_quat = new_quat / norms
    
    return new_time, new_pos, new_quat, new_wrench

def create_dataset():
    # 1. 寻找&排序 CSV 文件
    search_path = os.path.join(csv_folder, "segment_*.csv")
    files = glob.glob(search_path)
    files.sort(key=get_file_index)

    if not files:
        print("未找到任何 CSV 文件。请检查路径是否正确。")
        return
    print(f"找到 {len(files)} 个 CSV 文件。")

    # 2. 创建 HDF5 文件
    f = h5py.File(output_hdf5_path, 'w')
    data_grp = f.create_group('data')

    # 3. 写入环境元数据（Env Metadata）
    env_args = {
        "env_name": "ForceMimicEnv",
        "type": 1,
        "env_kwargs": {
            "control_freq": int(TARGET_FREQ), # 这里写入 30
            "controller_configs": {"type": "OSC_POSE"}
        }
    }
    data_grp.attrs["env_args"] = json.dumps(env_args)

    total_samples = 0
    demo_keys = []

    # 4. 处理每个 CSV 文件
    for i, csv_file in enumerate(files):
        demo_key = f"demo_{i}"

        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            print(f"Skipping {csv_file}: {e}")
            continue

        # --- A. 重采样 (核心步骤) ---
        resampled_data = resample_data(df, TARGET_FREQ)
        if resampled_data is None:
            print(f"Skipping {csv_file}: Duration too short for 30Hz.")
            continue

        r_time, r_pos, r_quat, r_wrench = resampled_data

        # --- B. 构建Obs和Action ---

        # Observation: 0~time-1
        obs_pos = r_pos[:-1].astype(np.float32)  # 去掉最后一个时间点
        obs_quat = r_quat[:-1].astype(np.float32)
        obs_wrench = r_wrench[:-1].astype(np.float32)

        # Action: 1~time
        act_pos = r_pos[1:].astype(np.float32)  # 去掉第一个时间点
        act_quat = r_quat[1:].astype(np.float32)
        act_wrench = r_wrench[1:].astype(np.float32)

        # 拼接Action
        act_no_wrench = np.concatenate([act_pos, act_quat], axis=1)
        act_with_wrench = np.concatenate([act_pos, act_quat, act_wrench], axis=1)

        num_samples = obs_pos.shape[0]
        total_samples += num_samples

        # --- C. 构建辅助信息 ---
        labels = np.zeros((num_samples,), dtype=np.int32)  # 全0标签
        rewards = np.zeros((num_samples,), dtype=np.float32)  # 全0奖励
        dones = np.zeros((num_samples,), dtype=np.int64)
        dones[-1] = 1 # 最后一帧设为1

        # --- D. 写入 HDF5 ---
        demo_keys.append(demo_key)
        demo_grp = data_grp.create_group(demo_key)
        demo_grp.attrs['num_samples'] = num_samples

        # 写入数据集
        demo_grp.create_dataset('action_without_wrench', data=act_no_wrench)
        demo_grp.create_dataset('action_with_wrench', data=act_with_wrench)
        demo_grp.create_dataset('dones', data=dones)
        demo_grp.create_dataset('rewards', data=rewards)

        # obs Group
        obs_grp = demo_grp.create_group("obs")
        obs_grp.create_dataset("robot0_eef_pos", data=obs_pos)
        obs_grp.create_dataset("robot0_eef_quat", data=obs_quat)
        obs_grp.create_dataset("robot0_eef_wrench", data=obs_wrench)
        obs_grp.create_dataset("label", data=labels)

        # # States (占位)
        # states = np.concatenate([obs_pos, obs_quat, obs_wrench], axis=1)
        # demo_grp.create_dataset("states", data=states)

        print(f"Processed {os.path.basename(csv_file)} -> {demo_key} ({num_samples} frames @ {TARGET_FREQ}Hz)")

    # 5. 写入数据集总信息
    data_grp.attrs['total'] = total_samples

    if len(demo_keys) > 0:
        mask_grp = f.create_group('mask')
        split_idx = int(len(demo_keys) * train_ratio)

        mask_grp.create_dataset("train", data=np.array(demo_keys[:split_idx], dtype='S'))
        mask_grp.create_dataset("valid", data=np.array(demo_keys[split_idx:], dtype='S'))

        print("\n转换完成!")
        print(f"保存路径: {output_hdf5_path}")
        print(f"总帧数: {total_samples}")
    else:
        print("未生成有效数据。")

    f.close()

if __name__ == "__main__":
    create_dataset()