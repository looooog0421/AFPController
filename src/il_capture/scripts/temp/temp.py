import h5py
import numpy as np
import tqdm  # 进度条库，如果没有请 pip install tqdm

# 1. 定义你的矩阵 (这里仅为示例，请填入你的真实数值)

HAND_EYE_FILE = '/home/lgx/Project/hand_eye_calib/src/calibrate/result_hand_eye_bak.txt'
T_old = np.loadtxt(HAND_EYE_FILE)

# T_new_example = np.eye(4)
# T_new_example[:3, 3] = [0.2, 0.0, 0.5] # 假设新的正确平移是 (0.2, 0, 0.5)
HAND_EYE_FILE = '/home/lgx/Project/hand_eye_calib/src/calibrate/result_hand_eye.txt'
T_new = np.loadtxt(HAND_EYE_FILE)

raw_file_path = '/home/lgx/Project/AFP/src/il_capture/data/layup_1768983281_20260121_161441_90_selected_selected.hdf5'
corrected_file_path = '/home/lgx/Project/AFP/src/il_capture/data/layup_1768983281_20260121_161441_90_selected_selected_corrected.hdf5'
TARGET_POINTS = 2048


def process_point_cloud_data(pc_data, T_corr, target_points=2048):
    """ 处理点云核心逻辑：坐标变换 + Z轴过滤 + 重采样 """
    num_frames = pc_data.shape[0]
    channels = pc_data.shape[2]
    
    # 预分配
    processed_frames = np.zeros((num_frames, target_points, channels), dtype=pc_data.dtype)
    
    for i in range(num_frames):
        frame = pc_data[i]
        xyz = frame[:, :3]
        feats = frame[:, 3:]
        
        # 1. 坐标矫正
        ones = np.ones((xyz.shape[0], 1))
        xyz_homo = np.hstack((xyz, ones))
        xyz_corrected = (T_corr @ xyz_homo.T).T[:, :3]
        
        # 2. 过滤 Z < 0
        mask = xyz_corrected[:, 2] >= 0.0
        xyz_filtered = xyz_corrected[mask]
        feats_filtered = feats[mask]
        
        # 3. 重采样
        curr_n = xyz_filtered.shape[0]
        if curr_n > 0:
            if curr_n >= target_points:
                idx = np.random.choice(curr_n, target_points, replace=False)
            else:
                idx = np.random.choice(curr_n, target_points, replace=True)
            
            xyz_final = xyz_filtered[idx]
            feats_final = feats_filtered[idx]
            processed_frames[i] = np.hstack((xyz_final, feats_final))
        else:
            # 极端情况：全0
            pass 
            
    return processed_frames

def main():
    # 计算矫正矩阵
    T_corr = T_new @ np.linalg.inv(T_old)
    
    print(f"源文件: {raw_file_path}")
    print(f"目标文件: {corrected_file_path}")

    with h5py.File(raw_file_path, 'r') as f_src, h5py.File(corrected_file_path, 'w') as f_dst:
        if 'data_selected' not in f_src:
            print("错误：源文件中找不到 'data_selected' 组")
            return

        src_data_grp = f_src['data_selected']
        dst_data_grp = f_dst.create_group('data')

        # [重要] 1. 复制全局属性 (global attributes)
        for k, v in src_data_grp.attrs.items():
            dst_data_grp.attrs[k] = v

        demo_keys = list(src_data_grp.keys())
        print(f"开始处理 {len(demo_keys)} 个演示数据...")

        for demo_name in tqdm.tqdm(demo_keys):
            src_demo = src_data_grp[demo_name]
            dst_demo = dst_data_grp.create_group(demo_name)

            # [重要] 2. 复制 Demo 属性 (num_samples) - 修复之前的报错
            for k, v in src_demo.attrs.items():
                dst_demo.attrs[k] = v

            # 3. 复制数据内容
            for key in src_demo.keys():
                if key != 'obs':
                    # 非 obs 数据 (action, rewards 等) 直接复制
                    src_demo.copy(key, dst_demo)
                else:
                    # 进入 obs 组
                    src_obs = src_demo['obs']
                    dst_obs = dst_demo.create_group('obs')
                    
                    for obs_key in src_obs.keys():
                        if obs_key != 'pointcloud':
                            # 其他观测数据 (如 joint_pos) 直接复制
                            src_obs.copy(obs_key, dst_obs)
                        else:
                            # === [核心] 处理并写入 PointCloud ===
                            raw_pc = src_obs['pointcloud'][:]
                            
                            # 调用处理函数
                            new_pc = process_point_cloud_data(raw_pc, T_corr, TARGET_POINTS)
                            
                            # [关键] 写入新数据 (之前这里被注释了导致现在的报错)
                            dst_obs.create_dataset('pointcloud', data=new_pc)

    print("处理完成！请使用新的 .hdf5 文件进行训练。")

if __name__ == "__main__":
    main()