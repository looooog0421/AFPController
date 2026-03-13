import h5py
import numpy as np
import open3d as o3d
import time
import os
from collections import OrderedDict
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R

class DataInspector:
    def __init__(self, h5_path, voxel_size=0.05, play_dt=0.05, ee_offset=[0, 0, 0.237]):
        self.h5_path = h5_path
        self.voxel_size = voxel_size
        self.play_dt = play_dt
        self.ee_offset = np.array(ee_offset)  # 执行器末端在机械臂末端坐标系的偏移

        self.good_demos = []
        self.bad_demos = []
        self.undecided_demos = []

        self.f = h5py.File(h5_path, 'r')
        self.demos = sorted(self.f['data'].keys())
        print(f"检测到 {len(self.demos)} 组演示数据。")

    def run(self):
        for demo in self.demos:
            print(f"\n检查 demo: {demo}")
            decision = self.inspect_demo(demo)

            if decision == "EXIT":
                print("收到 ESC，退出程序。")
                return
            
            if decision is True:
                self.good_demos.append(demo)
                print(f"✅ 保留 {demo}")
            elif decision is False:
                self.bad_demos.append(demo)
                print(f"❌ 删除 {demo}")
            else:
                self.undecided_demos.append(demo)
                print(f"⚠️ 未决 {demo}")


        self.f.close()
        self.save_cleaned_dataset()

    # ================= 辅助函数：计算执行器末端位姿 =================
    def compute_ee_tool_pos(self, eef_pos, eef_quat):
        """计算执行器末端位置
        Args:
            eef_pos: (T, 3) 机械臂末端位置
            eef_quat: (T, 4) 机械臂末端四元数 (x, y, z, w)
        Returns:
            tool_pos: (T, 3) 执行器末端位置
        """
        T = eef_pos.shape[0]
        tool_pos = np.zeros_like(eef_pos)
        
        for t in range(T):
            # 将四元数转换为旋转矩阵
            rot = R.from_quat(eef_quat[t])  # scipy expects (x, y, z, w)
            # 将偏移量变换到世界坐标系
            offset_world = rot.apply(self.ee_offset)
            tool_pos[t] = eef_pos[t] + offset_world
        
        return tool_pos
    
    # ================= 辅助函数：数据平滑 =================
    def smooth_data(self, data, window_length=15, polyorder=3):
        """使用Savitzky-Golay滤波器平滑数据
        Args:
            data: (T, D) 待平滑数据
            window_length: 窗口长度（必须为奇数）
            polyorder: 多项式阶数
        Returns:
            smoothed: (T, D) 平滑后的数据
        """
        if data.shape[0] < window_length:
            window_length = data.shape[0] if data.shape[0] % 2 == 1 else data.shape[0] - 1
            if window_length < polyorder + 2:
                return data  # 数据太少，无法平滑
        
        smoothed = np.zeros_like(data)
        for i in range(data.shape[1]):
            smoothed[:, i] = savgol_filter(data[:, i], window_length, polyorder)
        return smoothed
    
    def smooth_force_data(self, data, window_length=31, polyorder=2):
        """使用Savitzky-Golay滤波器对力数据进行强平滑
        Args:
            data: (T, D) 待平滑力数据
            window_length: 窗口长度（更大的窗口 = 更强的平滑）
            polyorder: 多项式阶数（较低的阶数 = 更强的平滑）
        Returns:
            smoothed: (T, D) 平滑后的力数据
        """
        if data.shape[0] < window_length:
            window_length = data.shape[0] if data.shape[0] % 2 == 1 else data.shape[0] - 1
            if window_length < polyorder + 2:
                return data  # 数据太少，无法平滑
        
        smoothed = np.zeros_like(data)
        for i in range(data.shape[1]):
            smoothed[:, i] = savgol_filter(data[:, i], window_length, polyorder)
        return smoothed

    def _replace_dataset(self, group, name, data):
        if name in group:
            del group[name]
        group.create_dataset(name, data=data)

    def _save_smoothed_demo(self, f_src, f_dst, demo, dst_group_name="data_selected"):
        dst_obs = f_dst[f"{dst_group_name}/{demo}/obs"]
        src_obs = f_src[f"data/{demo}/obs"]

        if 'robot0_eef_pos' in src_obs:
            eef_pos = src_obs['robot0_eef_pos'][:]
            eef_pos_smooth = self.smooth_data(eef_pos)

            if 'robot0_eef_pos_raw' not in dst_obs:
                self._replace_dataset(dst_obs, 'robot0_eef_pos_raw', eef_pos)
            self._replace_dataset(dst_obs, 'robot0_eef_pos', eef_pos_smooth)

        force_key = None
        if 'robot0_eef_wrench' in src_obs:
            force_key = 'robot0_eef_wrench'
        elif 'robot0_eef_force' in src_obs:
            force_key = 'robot0_eef_force'

        if force_key is not None:
            force_data = src_obs[force_key][:]
            force_smooth = self.smooth_force_data(force_data)
            raw_name = f"{force_key}_raw"
            if raw_name not in dst_obs:
                self._replace_dataset(dst_obs, raw_name, force_data)
            self._replace_dataset(dst_obs, force_key, force_smooth)
    # ================= 核心可视化 =================
    def inspect_demo(self, demo_name):
        group = self.f['data'][demo_name]['obs']

        if 'robot0_eef_pos' not in group or 'pointcloud' not in group:
            print("缺失关键字段，自动跳过")
            return None

        eef_pos = group['robot0_eef_pos'][:]  # (T, 3) 机械臂末端位置
        eef_quat = group['robot0_eef_quat'][:] if 'robot0_eef_quat' in group else None  # (T, 4)
        force_data = group['robot0_eef_wrench'][:] if 'robot0_eef_wrench' in group else None  # (T, 3)
        
        pcd_ds = group['pointcloud']          # HDF5 dataset，不读数据
        
        T = eef_pos.shape[0]
        
        # 计算执行器末端位置
        if eef_quat is not None:
            tool_pos = self.compute_ee_tool_pos(eef_pos, eef_quat)
            tool_pos_smooth = self.smooth_data(tool_pos)
        else:
            # 如果没有四元数，简单添加偏移（假设无旋转）
            tool_pos = eef_pos + self.ee_offset
            tool_pos_smooth = self.smooth_data(tool_pos)
        
        # 平滑机械臂末端位置
        eef_pos_smooth = self.smooth_data(eef_pos)
        
        # 平滑力数据（使用更强的滤波）
        if force_data is not None:
            force_smooth = self.smooth_force_data(force_data)
        else:
            force_smooth = None

        # ---- Open3D 初始化 ----
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(f"Inspecting {demo_name}", 1280, 800)

        # 轨迹线 - 机械臂末端（红色）
        lines = [[i, i + 1] for i in range(T - 1)]
        line_set_eef = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(eef_pos),
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set_eef.colors = o3d.utility.Vector3dVector([[1, 0, 0]] * len(lines))  # 红色
        
        # 轨迹线 - 机械臂末端平滑（粉色）
        line_set_eef_smooth = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(eef_pos_smooth),
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set_eef_smooth.colors = o3d.utility.Vector3dVector([[1, 0.5, 0.5]] * len(lines))  # 粉色

        # 轨迹线 - 执行器末端（蓝色）
        line_set_tool = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(tool_pos),
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set_tool.colors = o3d.utility.Vector3dVector([[0, 0, 1]] * len(lines))  # 蓝色
        
        # 轨迹线 - 执行器末端平滑（青色）
        line_set_tool_smooth = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(tool_pos_smooth),
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set_tool_smooth.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 1]] * len(lines))  # 青色

        # 点云（只创建一次）
        pcd = o3d.geometry.PointCloud()
        
        # 末端球 - 机械臂末端（红色）
        sphere_eef = o3d.geometry.TriangleMesh.create_sphere(0.008)
        sphere_eef.paint_uniform_color([1, 0, 0])
        
        # 末端球 - 执行器末端（蓝色）
        sphere_tool = o3d.geometry.TriangleMesh.create_sphere(0.008)
        sphere_tool.paint_uniform_color([0, 0, 1])

        vis.add_geometry(line_set_eef)
        vis.add_geometry(line_set_eef_smooth)
        vis.add_geometry(line_set_tool)
        vis.add_geometry(line_set_tool_smooth)
        vis.add_geometry(pcd)
        vis.add_geometry(sphere_eef)
        vis.add_geometry(sphere_tool)

        state = {
            "t": 0,
            "playing": False,
            "decision": None,
            "exit_all": False
        }
        
        # ---- 创建力数据可视化窗口 ----
        if force_data is not None:
            fig, axes = plt.subplots(3, 1, figsize=(10, 8))
            fig.canvas.manager.set_window_title(f'Force Data - {demo_name}')
            time_steps = np.arange(T)
            
            force_lines = []
            force_smooth_lines = []
            force_markers = []
            
            for i, (ax, label, color) in enumerate(zip(axes, ['Force X', 'Force Y', 'Force Z'], ['r', 'g', 'b'])):
                line_raw, = ax.plot(time_steps, force_data[:, i], alpha=0.5, color=color, label='原始')
                line_smooth, = ax.plot(time_steps, force_smooth[:, i], linewidth=2, color=color, label='平滑')
                marker, = ax.plot([0], [force_data[0, i]], 'o', markersize=10, color='black')
                
                force_lines.append(line_raw)
                force_smooth_lines.append(line_smooth)
                force_markers.append(marker)
                
                ax.set_ylabel(label)
                ax.legend()
                ax.grid(True)
            
            axes[-1].set_xlabel('时间步')
            fig.tight_layout()
            plt.ion()
            plt.show()
        else:
            fig = None
            force_markers = None

        # ---- 点云读取函数（lazy）----
        def load_pcd_frame(t):
            # 原始形状: (10000, 6)
            pts = pcd_ds[t]
            print("Number of points:", len(pcd.points))
            if pts.shape[0] == 0:
                # 直接跳过这一帧
                return None

            # -------- 1. 只取 xyz --------
            if pts.ndim != 2 or pts.shape[1] < 3:
                raise ValueError(f"点云格式异常: {pts.shape}")

            pts = pts[:, :3]

            # -------- 2. 过滤非法点 --------
            mask = np.isfinite(pts).all(axis=1)
            pts = pts[mask]

            if pts.shape[0] == 0:
                raise ValueError("该帧点云为空")

            # -------- 3. Open3D 要求 --------
            pts = np.ascontiguousarray(pts, dtype=np.float64)

            temp = o3d.geometry.PointCloud()
            temp.points = o3d.utility.Vector3dVector(pts)

            if self.voxel_size > 0:
                temp = temp.voxel_down_sample(self.voxel_size)

            return temp

        def update():
            t = state["t"]
            temp = load_pcd_frame(t)
            if temp is None:
                print(f"Frame {t} 点云为空，跳过")
                return
            
            # 更新点云
            pcd.points = temp.points
            vis.update_geometry(pcd)
            
            # 更新机械臂末端球体
            sphere_eef.translate(eef_pos[t] - sphere_eef.get_center(), relative=True)
            vis.update_geometry(sphere_eef)
            
            # 更新执行器末端球体
            sphere_tool.translate(tool_pos[t] - sphere_tool.get_center(), relative=True)
            vis.update_geometry(sphere_tool)
            
            # 更新力数据图表
            if force_markers is not None:
                for i, marker in enumerate(force_markers):
                    marker.set_xdata([t])
                    marker.set_ydata([force_data[t, i]])
                fig.canvas.draw()
                fig.canvas.flush_events()

        # ---- 键盘回调 ----
        def next_frame(vis):
            if state["t"] < T - 1:
                state["t"] += 1
                update()

        def prev_frame(vis):
            if state["t"] > 0:
                state["t"] -= 1
                update()

        def toggle(vis):
            state["playing"] = not state["playing"]

        def mark_good(vis):
            state["decision"] = True
            vis.close()

        def exit_program(vis):
            state["exit_all"] = True
            state["decision"] = None
            vis.close()
        
        def mark_bad(vis):
            state["decision"] = False
            vis.close()

        vis.register_key_callback(262, next_frame)
        vis.register_key_callback(263, prev_frame)
        vis.register_key_callback(32, toggle)
        vis.register_key_callback(ord('Y'), mark_good)
        vis.register_key_callback(ord('N'), mark_bad)
        vis.register_key_callback(256, exit_program)  # ESC

        print("→ / ← 切帧 | Space 播放 | Y 保留 | N 删除 | ESC 退出")

        update()

        # ---- 主循环 ----
        while state["decision"] is None and not state["exit_all"]:
            vis.poll_events()
            vis.update_renderer()

            if state["playing"]:
                if state["t"] < T - 1:
                    state["t"] += 1
                    update()
                    time.sleep(self.play_dt)
                else:
                    state["playing"] = False

        vis.destroy_window()
        
        # 关闭matplotlib窗口
        if fig is not None:
            plt.close(fig)

        if state["exit_all"]:
            return "EXIT"
        return state["decision"]

    # ================= 数据保存 =================
    def save_cleaned_dataset(self):
        if not self.good_demos:
            print("没有选择任何演示数据，未生成新文件。")
            return

        new_path = self.h5_path.replace(".hdf5", "_selected.hdf5")
        print(f"\n生成新文件: {new_path}")
        print(f"保留的演示: {len(self.good_demos)} 组")
        print(f"删除的演示: {len(self.bad_demos)} 组")
        print(f"未决的演示: {len(self.undecided_demos)} 组")

        with h5py.File(self.h5_path, 'r') as f_src, \
             h5py.File(new_path, 'w') as f_dst:

            dst_data = f_dst.create_group('data_selected')
            for demo in self.good_demos:
                print(f"  复制 {demo}...")
                f_src.copy(f"data/{demo}", dst_data)
                self._save_smoothed_demo(f_src, f_dst, demo, dst_group_name="data_selected")

        print(f"\n✅ 数据筛选完成，已保存到: {new_path}")

# ================= 使用 =================
if __name__ == "__main__":
    path = "/home/lgx/Project/AFP/src/il_capture/data/layup_1768983281_20260121_161441_90_selected.hdf5"
    inspector = DataInspector(path, voxel_size=0.01)
    inspector.run()
