import h5py

# 太奶奶，这里填您原来的老文件名字
source_file = '/home/lgx/Project/AFP/src/il_capture/data/90_all_corrected_cleaned_smoothed.hdf5'
# 这是咱们即将凭空造出来的新文件名字
target_file = '/home/lgx/Project/AFP/src/il_capture/data/90_all_corrected_cleaned_smoothed_added.hdf5'

with h5py.File(source_file, 'r') as f_in, h5py.File(target_file, 'w') as f_out:
    data_group_in = f_in['data']
    data_group_out = f_out.create_group('data')
    
    # 把大柜子外面的总说明书也抄过来
    for attr_key, attr_val in data_group_in.attrs.items():
        data_group_out.attrs[attr_key] = attr_val

    for demo_name in data_group_in.keys():
        demo_in = data_group_in[demo_name]
        demo_out = data_group_out.create_group(demo_name)
        
        # 第一步：搬运删减过的数据（动作砍前头两步）
        action_with_wrench = demo_in['action_with_wrench'][:]
        demo_out.create_dataset('action_with_wrench', data=action_with_wrench[2:])
        
        action_without_wrench = demo_in['action_without_wrench'][:]
        demo_out.create_dataset('action_without_wrench', data=action_without_wrench[2:])
        
        # （画面等数据砍后头两步）
        dones = demo_in['dones'][:]
        demo_out.create_dataset('dones', data=dones[:-2])
        
        obs_in = demo_in['obs']
        obs_out = demo_out.create_group('obs')
        for obs_key in obs_in.keys():
            obs_data = obs_in[obs_key][:]
            obs_out.create_dataset(obs_key, data=obs_data[:-2])
            
        # 第二步：这就是治病的关键！抄写抽屉门上的便签纸！
        for attr_key, attr_val in demo_in.attrs.items():
            if attr_key == 'num_samples':
                # 原来的步数减去两步，贴上新标签
                demo_out.attrs['num_samples'] = attr_val - 2
            else:
                # 其他便签原样照抄
                demo_out.attrs[attr_key] = attr_val

print("太奶奶，这回便签也贴好啦，电脑保管能看懂了！")