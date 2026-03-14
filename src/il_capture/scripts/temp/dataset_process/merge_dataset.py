### 合并两个 HDF5 轨迹数据集的脚本

import h5py

def merge_trajectory_hdf5(file1_path, file2_path, output_path):
    with h5py.File(file1_path, 'r') as f1, \
         h5py.File(file2_path, 'r') as f2, \
         h5py.File(output_path, 'w') as out_f:
        
        print(f"正在创建合并文件: {output_path}")
        
        # 1. 创建顶层 'data' 组
        out_data_group = out_f.create_group('data')
        
        # 2. 复制文件的全局属性（元数据，如 env_args 等）
        for k, v in f1.attrs.items():
            out_f.attrs[k] = v
        if 'data' in f1:
            for k, v in f1['data'].attrs.items():
                out_data_group.attrs[k] = v

        demo_counter = 0
        
        # 辅助函数：用于筛选并按数字顺序排列 demo_X
        def get_sorted_demos(hdf5_file):
            if 'data' not in hdf5_file:
                return []
            keys = list(hdf5_file['data'].keys())
            # 只保留以 'demo_' 开头的组，并按后面的数字进行排序
            demo_keys = [k for k in keys if k.startswith('demo_')]
            demo_keys.sort(key=lambda x: int(x.split('_')[1]))
            return demo_keys

        # 3. 复制第一个文件的所有 demo
        print(f"正在从 {file1_path} 复制数据...")
        f1_demos = get_sorted_demos(f1)
        for demo_name in f1_demos:
            new_demo_name = f"demo_{demo_counter}"
            # 整体复制整个 demo 组（包括里面的 obs, actions 等所有嵌套结构）
            f1.copy(f"data/{demo_name}", out_data_group, name=new_demo_name)
            demo_counter += 1
            
        # 4. 复制第二个文件的所有 demo，重命名顺延
        print(f"正在从 {file2_path} 复制数据并顺延编号...")
        f2_demos = get_sorted_demos(f2)
        for demo_name in f2_demos:
            new_demo_name = f"demo_{demo_counter}"
            f2.copy(f"data/{demo_name}", out_data_group, name=new_demo_name)
            demo_counter += 1
                
        print(f"✅ 合并完成！新文件共包含 {demo_counter} 个 demo。")

# --- 运行示例 ---
# 请将下面的文件名替换为你实际的文件路径
file1 = '/home/lgx/Project/AFP/src/il_capture/data/120_all_corrected_cleaned_smoothed.hdf5'
file2 = '/home/lgx/Project/AFP/src/il_capture/data/120_single_corrected_cleaned_smoothed.hdf5' # 假设你的第二个文件叫这个
output = '/home/lgx/Project/AFP/src/il_capture/data/120_mixed_corrected_cleaned_smoothed.hdf5'

merge_trajectory_hdf5(file1, file2, output)