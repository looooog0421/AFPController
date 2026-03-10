import h5py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties # 新增这行
from matplotlib.ticker import MultipleLocator

font_path = '/home/lgx/文档/simsun.ttc'

# ==========================================
# 模块 1: 数据提取 (与绘图解耦，方便复用)
# ==========================================
def load_dataset_trajectories(file_path):
    """
    从 hdf5 数据集中提取点云和所有轨迹数据。
    返回:
        pc_data: np.ndarray, (N, 3) 第一帧的点云数据
        trajectories: list of np.ndarray, 包含所有demo的轨迹点 (T, 3)
    """
    trajectories = []
    
    with h5py.File(file_path, 'r') as f:
        demos = list(f['data'].keys())
        demos.sort(key=lambda x: int(x.split('_')[1]))
        
        # 提取第一个 demo 的第一帧点云作为场景上下文
        first_demo = demos[0]
        pc_data = f[f'data/{first_demo}/obs/pointcloud'][0]
        
        # 提取所有 demo 的 EEF 轨迹
        for demo in demos:
            pos_data = f[f'data/{demo}/obs/robot0_eef_pos'][:]
            trajectories.append(pos_data)
            
    return pc_data, trajectories

# ==========================================
# 模块 2: 学术级绘图 (高复用性，可配置)
# ==========================================
def plot_academic_3d_trajectories_wide_cn(pc_data, trajectories, save_path=None):
    cm_to_inch = 1 / 2.54
    fig_width = 9 * cm_to_inch  
    fig_height = 8.5 * cm_to_inch  
    
    # 策略 1: 全局只设 Times New Roman
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman"], 
        "axes.unicode_minus": False, 
        "mathtext.fontset": "stix",  
        "font.size": 10.5,           
        "axes.labelsize": 10.5,      
        "xtick.labelsize": 10.5,
        "ytick.labelsize": 10.5,
        "axes.linewidth": 1.0
    })
    
    cn_font = FontProperties(fname=font_path, size=10.5)
    
    fig = plt.figure(figsize=(fig_width, fig_height), dpi=300)
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. 绘制点云
    ax.scatter(pc_data[:, 0], pc_data[:, 1], pc_data[:, 2], 
               s=2, alpha=0.5, c='#B0B0B0', edgecolors='none', label='工作空间点云')

    # 2. 绘制轨迹与起点
    traj_color = '#1f77b4' 
    for i, traj in enumerate(trajectories):
        label_traj = '专家示教轨迹' if i == 0 else None
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                color=traj_color, alpha=0.4, linewidth=0.8, label=label_traj)
        
        label_start = '轨迹起点' if i == 0 else None
        ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], 
                   color='#d62728', s=8, alpha=0.7, marker='o', label=label_start)

    # ==========================================
    # 【核心修改区】: 强制 0.1m 间距与真实物理比例
    # ==========================================
    # A. 强制每个轴的刻度间隔为 0.1
    ax.xaxis.set_major_locator(MultipleLocator(0.1))
    ax.yaxis.set_major_locator(MultipleLocator(0.1))
    ax.zaxis.set_major_locator(MultipleLocator(0.1))
    
    # B. 提取所有点以计算 X, Y, Z 的真实物理跨度
    all_x = np.concatenate([pc_data[:, 0]] + [t[:, 0] for t in trajectories])
    all_y = np.concatenate([pc_data[:, 1]] + [t[:, 1] for t in trajectories])
    all_z = np.concatenate([pc_data[:, 2]] + [t[:, 2] for t in trajectories])
    
    range_x = all_x.max() - all_x.min()
    range_y = all_y.max() - all_y.min()
    range_z = all_z.max() - all_z.min()
    
    # C. 按照真实跨度设置 3D 框的长宽比，防止空间形变
    ax.set_box_aspect([range_x, range_y, range_z])

    # ==========================================
    # 标签与图例设置
    # ==========================================
    ax.set_xlabel(r'$\mathrm{X}$轴 ($\mathrm{m}$)', fontproperties=cn_font, labelpad=5)
    ax.set_ylabel(r'$\mathrm{Y}$轴 ($\mathrm{m}$)', fontproperties=cn_font, labelpad=5)
    ax.set_zlabel(r'$\mathrm{Z}$轴 ($\mathrm{m}$)', fontproperties=cn_font, labelpad=8) 
    
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    
    ax.xaxis._axinfo["grid"].update({"color": "#E0E0E0", "linewidth": 0.5})
    ax.yaxis._axinfo["grid"].update({"color": "#E0E0E0", "linewidth": 0.5})
    ax.zaxis._axinfo["grid"].update({"color": "#E0E0E0", "linewidth": 0.5})

    # 图例排布在左上角，上下排列
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 1.05), 
              ncol=1, frameon=False, handletextpad=0.5, prop=cn_font)

    ax.view_init(elev=20, azim=-75) 
    fig.subplots_adjust(left=0.0, right=0.85, bottom=0.1, top=0.95)

    if save_path:
        plt.savefig(save_path, dpi=300, format='pdf') 
        print(f"宽版图表已保存至: {save_path}")
    
    plt.show()

# ==========================================
# 运行示例
# ==========================================
if __name__ == "__main__":
    file_path = '/home/lgx/Project/AFP/src/il_capture/data/120_all_corrected_cleaned.hdf5'
    
    # 1. 获取数据 (这部分代码可以在其他地方复用)
    print("Loading data...")
    pc_data, trajectories = load_dataset_trajectories(file_path)
    
    # 2. 绘制并保存用于论文的图 (可以保存为 .pdf 或 .png)
    print("Plotting academic figure...")
    save_path = "dataset_trajectories_academic.pdf" 
    plot_academic_3d_trajectories_wide_cn(pc_data, trajectories, save_path=save_path)