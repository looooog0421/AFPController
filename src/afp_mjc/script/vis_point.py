import numpy as np
import plotly.graph_objects as go
import os
import glob
import tqdm

def visualize_point_clouds_with_plotly(folder_path, sample_step=5):
    """
    Args:
        folder_path: 包含 .npy 文件的文件夹路径
        sample_step: 降采样步长。1 表示全量点云，5 表示每 5 个点取 1 个（为了浏览器流畅度建议设为 5-10）
    """
    # 获取文件夹中所有的 .npy 文件
    npy_files = glob.glob(os.path.join(folder_path, '*.npy'))
    if not npy_files:
        print(f"在文件夹 {folder_path} 中未找到任何 .npy 文件。")
        return

    for npy_file in tqdm.tqdm(npy_files, desc="生成 Plotly 可视化"):
        # 1. 加载数据
        point_cloud_data = np.load(npy_file)  # Nx6 (x, y, z, r, g, b)

        # 2. 降采样 (Browser rendering optimization)
        # 如果点云很大（例如 640x480 = 30万点），Plotly 可能会卡顿，建议切片采样
        points = point_cloud_data[::sample_step, :3]
        colors = point_cloud_data[::sample_step, 3:]

        # 3. 处理颜色
        # Plotly 的 Scatter3d 接受 'rgb(r, g, b)' 格式的颜色字符串列表
        # 注意：这里的 r,g,b 需要是 0-255 的整数
        color_strings = [
            f'rgb({int(r)}, {int(g)}, {int(b)})' 
            for r, g, b in colors
        ]

        # 4. 创建 Plotly 图形
        fig = go.Figure(data=[go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            mode='markers',
            marker=dict(
                size=2,                # 点的大小
                color=color_strings,   # 每个点的颜色
                opacity=0.8            # 透明度
            )
        )])

        # 5. 设置布局 (保持 1:1:1 的物理比例)
        fig.update_layout(
            title=f"File: {os.path.basename(npy_file)} (Points: {len(points)})",
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z',
                aspectmode='data'  # 重要：保持 XYZ 轴比例一致，防止物体变形
            ),
            margin=dict(r=0, l=0, b=0, t=40)
        )

        # 6. 显示或保存
        # 选项 A: 直接在浏览器弹出 (会阻塞，或者开很多标签页)
        fig.show()

        # 选项 B: 保存为 HTML 文件 (推荐)
        # save_name = npy_file.replace('.npy', '.html')
        # fig.write_html(save_name)
        # print(f"已保存: {save_name}")

if __name__ == "__main__":
    folder_path = "/home/lgx/Project/AFP/src/afp_mjc/script"
    
    # sample_step=10 意味着只显示 10% 的点，极大地提高交互流畅度
    visualize_point_clouds_with_plotly(folder_path, sample_step=10)