import mujoco
from mujoco import viewer
import numpy as np
import os
import time
import rospy
import sys
import open3d as o3d
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, WrenchStamped 
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from sensor_msgs import point_cloud2
import threading

ROI_X = [-0.8, -0.4]
ROI_Y = [-0.45, 0.0]
ROI_Z = [0.01, 0.5]

class MujocoSim:
    def __init__(self, model_path):
        # 加载Mujoco模型
        full_path = os.path.abspath(model_path)
        self.model = mujoco.MjModel.from_xml_path(full_path)
        self.data = mujoco.MjData(self.model)

        ## 加载渲染器
        self.height = 480
        self.width = 640
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "depth_camera")

        self.ctrl_q = np.zeros(self.model.nu)
        
        self.ee_body_name = "afp_roll_link"
        self.force_sensor_name = "ee_force_sensor"
        self.torque_sensor_name = "ee_torque_sensor"
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        self.shadow_joint_sub = rospy.Subscriber(
                            "/joint_states",
                            JointState,
                            self.shadow_joint_callback
                            )
        
        # 发布位置和力传感器数据
        self.ee_pose_pub = rospy.Publisher("/mujoco/ee_pose", PoseStamped, queue_size=10)
        self.ee_wrench_pub = rospy.Publisher("/mujoco/ee_wrench", WrenchStamped, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/mujoco/depth_camera/pointcloud", PointCloud2, queue_size=10)

        # 定时发布传感器消息
        wrench_freq = 500
        self.pointcloud_freq = 30
        rospy.Timer(rospy.Duration(1.0 / wrench_freq), lambda event: self.publish_wrench())

        self.data_lock = threading.Lock()
        self.last_pc_time = 0

        # 预计算相机内参和外参（这些参数在仿真中通常是固定的）
        self._precompute_camera_params()

        self.run_simulation()

    def _precompute_camera_params(self):
        """预计算相机的内参和坐标系变换矩阵"""
        # 内参矩阵计算
        fov = self.model.cam_fovy[self.camera_id]
        theta = np.deg2rad(fov)
        fx = self.width / 2 / np.tan(theta / 2)
        fy = self.height / 2 / np.tan(theta / 2)
        cx = (self.width - 1) / 2.0
        cy = (self.height - 1) / 2.0
        self.intr = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        
        # 从成像坐标系到MuJoCo相机坐标系的变换
        # 成像坐标系：X右，Y下，Z前（深度方向）
        # MuJoCo相机坐标系：X右，Y下，Z后（看向-Z）
        # 需要绕X轴旋转180度：[1,0,0; 0,-1,0; 0,0,-1]
        self.image_to_camera = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        print(f"相机内参矩阵:\n{self.intr}")
        print(f"FOV: {fov}°, fx: {fx:.2f}, fy: {fy:.2f}")
        print("坐标系变换: 成像坐标系 -> MuJoCo相机坐标系 (绕X轴180°)")

    def run_simulation(self):
        print("启动实时仿真与点云发布...")
        start_time = time.time()
        pc_interval = 1.0 / self.pointcloud_freq

        with viewer.launch_passive(self.model, self.data) as sim:
            while sim.is_running() and not rospy.is_shutdown():
                wall_time = time.time() - start_time
                while self.data.time < wall_time:
                    with self.data_lock:
                        self.data.ctrl[:self.model.nu] = self.ctrl_q[:self.model.nu]
                        mujoco.mj_step(self.model, self.data)

                if wall_time - self.last_pc_time >= pc_interval:
                    self.pointcloud_publisher()
                    self.publish_state()
                    self.last_pc_time = wall_time

                sim.sync()
                time.sleep(0.001)

    def pointcloud_publisher(self):
        """发布相机点云数据 - 使用issue代码的逻辑"""
        with self.data_lock:
            # 先更新物理状态
            mujoco.mj_forward(self.model, self.data)
            
            # 渲染RGB和深度图
            rgb, depth = self.render_rgbd()
            
            # 调试：打印深度图信息
            print(f"深度图形状: {depth.shape}, RGB形状: {rgb.shape}")
            print(f"深度范围: min={depth.min():.4f}, max={depth.max():.4f}")
            print(f"有效深度点数: {np.sum((depth > 0) & (depth < 3.0))}")
            
            # 获取相机外参（每次调用时更新，因为相机可能移动）
            cam_pos = self.data.cam_xpos[self.camera_id]
            cam_rot = self.data.cam_xmat[self.camera_id].reshape(3, 3)
            extr = np.eye(4)
            extr[:3, :3] = cam_rot.T
            extr[:3, 3] = cam_pos
            
            print(f"相机位置: {cam_pos}")
            print(f"相机旋转矩阵:\n{cam_rot}")
            
            # 转换为点云（世界坐标系）
            xyzrgb = self.rgbd_to_pointcloud(rgb, depth, self.intr, extr)
            
        if xyzrgb.shape[0] == 0:
            print("警告: 生成的点云为空，请检查：")
            print("  1. 相机是否正确配置")
            print("  2. 场景中是否有可见物体")
            print("  3. 深度范围设置是否合理")
            return
            
        points_world = xyzrgb[:, :3]
        colors = xyzrgb[:, 3:]
        
        print(f"点云统计 - 点数: {points_world.shape[0]}, "
              f"X范围: [{points_world[:, 0].min():.3f}, {points_world[:, 0].max():.3f}], "
              f"Y范围: [{points_world[:, 1].min():.3f}, {points_world[:, 1].max():.3f}], "
              f"Z范围: [{points_world[:, 2].min():.3f}, {points_world[:, 2].max():.3f}]")
        
        # 可选：ROI裁剪
        # mask_roi = (points_world[:, 0] > ROI_X[0]) & (points_world[:, 0] < ROI_X[1]) & \
        #            (points_world[:, 1] > ROI_Y[0]) & (points_world[:, 1] < ROI_Y[1]) & \
        #            (points_world[:, 2] > ROI_Z[0]) & (points_world[:, 2] < ROI_Z[1])
        # points_world = points_world[mask_roi]
        
        # 发布点云
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        pc_msg = point_cloud2.create_cloud_xyz32(header, points_world)
        self.pointcloud_pub.publish(pc_msg)

    def render_rgbd(self):
        """渲染RGB和深度图 - 参考issue代码"""
        self.renderer.update_scene(self.data, camera=self.camera_id)
        
        # 先渲染深度
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render()
        
        # 再渲染RGB
        self.renderer.disable_depth_rendering()
        rgb = self.renderer.render()
        
        return rgb, depth

    def rgbd_to_pointcloud(self, rgb, depth, intr, extr, depth_trunc=3.0):
        """
        将RGB-D图像转换为点云
        
        坐标系变换链：
        1. 反投影 -> 成像坐标系 (X右, Y下, Z前)
        2. 绕X轴180° -> MuJoCo相机坐标系 (X右, Y下, Z后)
        3. 外参变换 -> 世界坐标系
        
        参数:
            rgb: RGB图像 (H, W, 3)
            depth: 深度图 (H, W)
            intr: 相机内参矩阵 (3, 3)
            extr: 相机外参矩阵 (4, 4)
            depth_trunc: 深度截断值
        
        返回:
            xyzrgb: (N, 6) 数组，包含xyz坐标和rgb颜色
        """
        # 检查深度图是否需要从归一化值转换
        if depth.max() <= 1.0 and depth.min() >= 0.0:
            print("检测到归一化深度图[0,1]，转换为实际距离...")
            znear = self.model.vis.map.znear
            zfar = self.model.vis.map.zfar
            depth = znear / (1.0 - depth * (1.0 - znear / zfar))
            print(f"  转换后深度范围: [{depth.min():.4f}, {depth.max():.4f}]米")
        
        # 生成像素网格
        cc, rr = np.meshgrid(np.arange(self.width), np.arange(self.height), sparse=True)
        
        # 过滤有效深度
        valid = (depth > 0) & (depth < depth_trunc)
        print(f"有效深度像素: {np.sum(valid)} / {depth.size} ({100*np.sum(valid)/depth.size:.1f}%)")
        
        z = np.where(valid, depth, np.nan)
        
        # 步骤1: 反投影到成像坐标系
        # 在成像坐标系中，Z是深度（正值，向前）
        x = np.where(valid, z * (cc - intr[0, 2]) / intr[0, 0], 0)
        y = np.where(valid, z * (rr - intr[1, 2]) / intr[1, 1], 0)
        
        # 组合xyz（成像坐标系）
        xyz_image = np.vstack([e.flatten() for e in [x, y, z]]).T
        
        # 提取对应的颜色
        color = rgb.transpose([2, 0, 1]).reshape((3, -1)).T / 255.0
        
        # 移除无效点
        mask = np.isnan(xyz_image[:, 2])
        xyz_image = xyz_image[~mask]
        color = color[~mask]
        
        print(f"过滤后的3D点数: {xyz_image.shape[0]}")
        
        if xyz_image.shape[0] == 0:
            print("错误: 没有有效的3D点！")
            return np.zeros((0, 6))
        
        # 步骤2: 从成像坐标系转换到MuJoCo相机坐标系（绕X轴180度）
        xyz_camera = (self.image_to_camera @ xyz_image.T).T
        
        # 步骤3: 从MuJoCo相机坐标系转换到世界坐标系
        xyz_h = np.hstack([xyz_image, np.ones((xyz_image.shape[0], 1))])
        

        xyz_world = (extr @ xyz_h.T).T
        
        # 组合xyz和rgb
        xyzrgb = np.hstack([xyz_world[:, :3], color])
        
        return xyzrgb

    def shadow_joint_callback(self, msg):
        """接收joint_states消息，更新机械臂当前关节位置"""
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for j, joint_name in enumerate(self.joint_names):
            if joint_name in name_to_index:
                idx = name_to_index[joint_name]
                self.ctrl_q[j] = msg.position[idx]

    def publish_state(self):
        """发布机械臂末端位姿"""
        try:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_name)
            pos = self.data.xpos[body_id]
            quat_mjc = self.data.xquat[body_id]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            pose_msg.pose.orientation.w = quat_mjc[0]
            pose_msg.pose.orientation.x = quat_mjc[1]
            pose_msg.pose.orientation.y = quat_mjc[2]
            pose_msg.pose.orientation.z = quat_mjc[3]

            self.ee_pose_pub.publish(pose_msg)
        except Exception as e:
            pass

    def publish_wrench(self):
        """发布力传感器数据"""
        try:
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = self.ee_body_name

            force_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.force_sensor_name)
            torque_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.torque_sensor_name)

            if force_id != -1:
                f_adr = self.model.sensor_adr[force_id]
                force = self.data.sensordata[f_adr : f_adr+3]
                wrench_msg.wrench.force.x = force[0]
                wrench_msg.wrench.force.y = force[1]
                wrench_msg.wrench.force.z = force[2]

            if torque_id != -1:
                t_adr = self.model.sensor_adr[torque_id]
                torque = self.data.sensordata[t_adr : t_adr+3]
                wrench_msg.wrench.torque.x = torque[0]
                wrench_msg.wrench.torque.y = torque[1]
                wrench_msg.wrench.torque.z = torque[2]
                
            self.ee_wrench_pub.publish(wrench_msg)
        except Exception as e:
            pass


if __name__ == "__main__":
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    rospy.init_node("mujoco_sim_node")
    try:
        sim = MujocoSim(model_path)
        print("仿真窗口已关闭，正在清理 ROS 节点...")
        rospy.signal_shutdown("Sim finished")
        sys.exit(0)
    except rospy.ROSInterruptException:
        pass