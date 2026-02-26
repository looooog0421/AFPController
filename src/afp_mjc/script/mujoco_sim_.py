#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import mujoco
from mujoco import viewer
import numpy as np
import os
import time
import rospy
import sys
import open3d as o3d
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped 
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from sensor_msgs import point_cloud2
import threading
from typing import Literal

# === 新增：适配真实控制器接口所需的 ROS 依赖 ===
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerResponse
# ===============================================

ROI_X = [-0.8, -0.4]
ROI_Y = [-0.45, 0.0]
ROI_Z = [0.01, 0.5]

def resample_pointcloud(points: np.ndarray, target_num: int = 2048, method: Literal['random', 'farthest'] = 'farthest') -> np.ndarray:
    # [保留原有的点云处理函数，此处省略详细实现以保持整洁，请保留你原文件中的实现]
    if not isinstance(points, np.ndarray): raise TypeError("points必须是numpy数组")
    if points.ndim != 2 or points.shape[1] != 3: raise ValueError(f"points形状必须是(N, 3)，当前形状: {points.shape}")
    n_points = points.shape[0]
    # if n_points == 0: raise ValueError("输入点云为空")
    if n_points == target_num or n_points == 0: return points.copy()
    if n_points > target_num:
        if method == 'random': return random_downsample(points, target_num)
        elif method == 'farthest': return farthest_point_sample(points, target_num)
        else: raise ValueError(f"未知的采样方法: {method}")
    else: return upsample_pointcloud(points, target_num)

def random_downsample(points: np.ndarray, target_num: int) -> np.ndarray:
    indices = np.random.choice(points.shape[0], target_num, replace=False)
    return points[indices]

def farthest_point_sample(points: np.ndarray, target_num: int) -> np.ndarray:
    n_points = points.shape[0]
    sampled_indices = np.zeros(target_num, dtype=np.int32)
    distances = np.ones(n_points) * np.inf
    current_idx = np.random.randint(0, n_points)
    for i in range(target_num):
        sampled_indices[i] = current_idx
        current_point = points[current_idx]
        dist = np.sum((points - current_point) ** 2, axis=1)
        distances = np.minimum(distances, dist)
        current_idx = np.argmax(distances)
    return points[sampled_indices]
  
def upsample_pointcloud(points: np.ndarray, target_num: int) -> np.ndarray:
    n_points = points.shape[0]
    repeat_times = target_num // n_points
    extra_points = target_num % n_points
    upsampled = np.repeat(points, repeat_times, axis=0)
    if extra_points > 0:
        extra_indices = np.random.choice(n_points, extra_points, replace=False)
        upsampled = np.vstack([upsampled, points[extra_indices]])
    scale = np.std(points, axis=0).mean()
    noise = np.random.randn(*upsampled.shape) * scale * 0.01 
    upsampled = upsampled + noise
    return upsampled


class MujocoSim:
    def __init__(self, model_path):
        full_path = os.path.abspath(model_path)
        self.model = mujoco.MjModel.from_xml_path(full_path)
        self.data = mujoco.MjData(self.model)

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
        
        self.ee_pose_pub = rospy.Publisher("/mujoco/ee_pose", PoseStamped, queue_size=10)
        self.ee_wrench_pub = rospy.Publisher("/mujoco/ee_wrench", WrenchStamped, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/mujoco/depth_camera/pointcloud", PointCloud2, queue_size=10)
        
        # === 核心修改区：模拟真实驱动接口 ===
        # 1. 主动发布高频关节状态
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        
        self.current_mode = "position"  # 控制模式状态机
        self.target_vel = np.zeros(6)   # 缓存速度指令
        
        # 2. 提供假的切换控制器服务，骗过控制器的安全检查
        self.switch_srv = rospy.Service('/controller_manager/switch_controller', SwitchController, self.handle_switch_controller)
        
        # 3. 订阅高频速度伺服话题
        self.vel_sub = rospy.Subscriber("/joint_group_vel_controller/command", Float64MultiArray, self.vel_cmd_callback)
        
        # 4. 提供位置轨迹 Action Server (包含内部插补逻辑)
        self.traj_as = actionlib.SimpleActionServer(
            "/scaled_pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.traj_execute_cb,
            auto_start=False
        )
        self.traj_as.start()
        # ==================================

        self.data_lock = threading.Lock()
        self.last_pc_time = 0

        # 定时发布传感器消息
        wrench_freq = 500
        joint_state_freq = 500  # 必须高频发布，控制器的 IK 依赖它
        self.pointcloud_freq = 30
        
        rospy.Timer(rospy.Duration(1.0 / wrench_freq), lambda event: self.publish_wrench())
        rospy.Timer(rospy.Duration(1.0 / joint_state_freq), lambda event: self.publish_joint_states())

        self._precompute_camera_params()
        self.run_simulation()

    # ============ 新增的接口模拟回调函数 ============
    def handle_switch_controller(self, req):
        """处理控制器的切换请求"""
        rospy.loginfo(f"Simulating controller switch: start={req.start_controllers}, stop={req.stop_controllers}")
        if "joint_group_vel_controller" in req.start_controllers:
            self.current_mode = "velocity"
            self.target_vel = np.zeros(6)
        elif "scaled_pos_joint_traj_controller" in req.start_controllers:
            self.current_mode = "position"
            # 切换回位置控制时，将当前真实位置锁定，防止跳变
            with self.data_lock:
                self.ctrl_q[:6] = self.data.qpos[:6]
        return SwitchControllerResponse(ok=True)

    def vel_cmd_callback(self, msg):
        """接收高频速度指令"""
        if self.current_mode == "velocity" and len(msg.data) >= 6:
            self.target_vel = np.array(msg.data[:6])

    def traj_execute_cb(self, goal):
        """处理大范围位置轨迹跟踪 (模拟真实的底层样条插补器)"""
        if self.current_mode != "position":
            self.traj_as.set_aborted(text="Robot is currently in velocity control mode!")
            return

        points = goal.trajectory.points
        if not points:
            self.traj_as.set_succeeded(FollowJointTrajectoryResult())
            return

        t_start = rospy.Time.now().to_sec()
        prev_time = 0.0
        with self.data_lock:
            prev_q = self.data.qpos[:6].copy()
            
        for pt in points:
            target_q = np.array(pt.positions[:6])
            curr_time = pt.time_from_start.to_sec()
            duration = curr_time - prev_time
            
            if duration <= 0:
                with self.data_lock:
                    self.ctrl_q[:6] = target_q
                prev_q = target_q
                prev_time = curr_time
                continue
                
            loop_rate = rospy.Rate(100) # 100Hz 软件插补
            while not rospy.is_shutdown():
                if self.traj_as.is_preempt_requested():
                    self.traj_as.set_preempted()
                    return
                
                t_now = rospy.Time.now().to_sec() - t_start
                if t_now >= curr_time:
                    break
                    
                # 线性插补计算
                alpha = max(0.0, min(1.0, (t_now - prev_time) / duration))
                interp_q = prev_q + alpha * (target_q - prev_q)
                
                with self.data_lock:
                    self.ctrl_q[:6] = interp_q
                loop_rate.sleep()
                
            with self.data_lock:
                self.ctrl_q[:6] = target_q
            prev_q = target_q
            prev_time = curr_time
            
        self.traj_as.set_succeeded(FollowJointTrajectoryResult())

    def publish_joint_states(self):
        """高频发布关节状态供控制器使用"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        with self.data_lock:
            msg.position = self.data.qpos[:6].tolist()
            msg.velocity = self.data.qvel[:6].tolist()
            msg.effort = self.data.qfrc_actuator[:6].tolist()
        self.joint_state_pub.publish(msg)
    # ================================================

    def _precompute_camera_params(self):
        fov = self.model.cam_fovy[self.camera_id]
        theta = np.deg2rad(fov)
        fx = self.width / 2 / np.tan(theta / 2)
        fy = self.height / 2 / np.tan(theta / 2)
        cx = (self.width - 1) / 2.0
        cy = (self.height - 1) / 2.0
        self.intr = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.image_to_camera = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    def run_simulation(self):
        start_time = time.time()
        pc_interval = 1.0 / self.pointcloud_freq

        with viewer.launch_passive(self.model, self.data) as sim:
            while sim.is_running() and not rospy.is_shutdown():
                wall_time = time.time() - start_time
                
                # === 物理步进核心修改：融合速度积分 ===
                dt = self.model.opt.timestep
                while self.data.time < wall_time:
                    with self.data_lock:
                        if self.current_mode == "velocity":
                            # 速度模式下，将速度指令积分到目标位置 (假设XML中配置的是位置执行器)
                            self.ctrl_q[:6] += self.target_vel * dt
                            
                        self.data.ctrl[:self.model.nu] = self.ctrl_q[:self.model.nu]
                        mujoco.mj_step(self.model, self.data)
                # ====================================

                if wall_time - self.last_pc_time >= pc_interval:
                    self.pointcloud_publisher()
                    self.publish_state()
                    self.last_pc_time = wall_time

                sim.sync()
                time.sleep(0.001)

    def pointcloud_publisher(self):
        with self.data_lock:
            mujoco.mj_forward(self.model, self.data)
            rgb, depth = self.render_rgbd()
            cam_pos = self.data.cam_xpos[self.camera_id]
            cam_rot = self.data.cam_xmat[self.camera_id].reshape(3, 3)
            extr = np.eye(4)
            extr[:3, :3] = cam_rot.T
            extr[:3, 3] = cam_pos
            xyzrgb = self.rgbd_to_pointcloud(rgb, depth, self.intr, extr)
            
        if xyzrgb.shape[0] == 0: return
        points_world = xyzrgb[:, :3]
        colors = xyzrgb[:, 3:]
        mask_roi = (points_world[:, 0] > ROI_X[0]) & (points_world[:, 0] < ROI_X[1]) & \
                   (points_world[:, 1] > ROI_Y[0]) & (points_world[:, 1] < ROI_Y[1]) & \
                   (points_world[:, 2] > ROI_Z[0]) & (points_world[:, 2] < ROI_Z[1])
        points_world = points_world[mask_roi]
        
        points_world = resample_pointcloud(points_world, target_num=2048, method='farthest')
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        pc_msg = point_cloud2.create_cloud_xyz32(header, points_world)
        self.pointcloud_pub.publish(pc_msg)

    def render_rgbd(self):
        self.renderer.update_scene(self.data, camera=self.camera_id)
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()
        rgb = self.renderer.render()
        return rgb, depth

    def rgbd_to_pointcloud(self, rgb, depth, intr, extr, depth_trunc=2.0):
        if depth.max() <= 1.0 and depth.min() >= 0.0:
            znear = self.model.vis.map.znear
            zfar = self.model.vis.map.zfar
            depth = znear / (1.0 - depth * (1.0 - znear / zfar))
        cc, rr = np.meshgrid(np.arange(self.width), np.arange(self.height), sparse=True)
        valid = (depth > 0) & (depth < depth_trunc)
        z = np.where(valid, depth, np.nan)
        x = np.where(valid, z * (cc - intr[0, 2]) / intr[0, 0], 0)
        y = np.where(valid, z * (rr - intr[1, 2]) / intr[1, 1], 0)
        xyz_image = np.vstack([e.flatten() for e in [x, y, z]]).T
        color = rgb.transpose([2, 0, 1]).reshape((3, -1)).T / 255.0
        mask = np.isnan(xyz_image[:, 2])
        xyz_image = xyz_image[~mask]
        color = color[~mask]
        if xyz_image.shape[0] == 0: return np.zeros((0, 6))
        xyz_camera = (self.image_to_camera @ xyz_image.T).T
        xyz_h = np.hstack([xyz_image, np.ones((xyz_image.shape[0], 1))])
        
        extr_ori = np.array([[ 0.99784269,  -0.01867954, 0.06293679],
                            [-0.05087261,  -0.82596274, 0.56142456],
                            [ 0.04149629,  -0.56341515,  -0.82513115]])
        extr[:3, :3] = extr_ori
        xyz_world = (extr @ xyz_h.T).T
        xyzrgb = np.hstack([xyz_world[:, :3], color])
        return xyzrgb

    def publish_state(self):
        try:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_name)
            pos = self.data.xpos[body_id]
            quat_mjc = self.data.xquat[body_id]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pos
            pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z = quat_mjc
            self.ee_pose_pub.publish(pose_msg)
        except Exception: pass

    def publish_wrench(self):
        try:
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = self.ee_body_name
            force_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.force_sensor_name)
            torque_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.torque_sensor_name)
            if force_id != -1:
                f_adr = self.model.sensor_adr[force_id]
                wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z = self.data.sensordata[f_adr : f_adr+3]
            if torque_id != -1:
                t_adr = self.model.sensor_adr[torque_id]
                wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z = self.data.sensordata[t_adr : t_adr+3]
            self.ee_wrench_pub.publish(wrench_msg)
        except Exception: pass

if __name__ == "__main__":
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    rospy.init_node("mujoco_sim_node")
    try:
        sim = MujocoSim(model_path)
        print("仿真窗口已关闭，正在清理 ROS 节点...")
        rospy.signal_shutdown("Sim finished")
        sys.exit(0)
    except rospy.ROSInterruptException: pass