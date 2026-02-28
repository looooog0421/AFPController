#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
import tf.transformations as tf_trans
from il_capture.msg import KineticStateStamped

class EMAFilter:
    """
    指数移动平均 (EMA) 滤波器，用于对序列数据进行平滑处理。
    """
    def __init__(self, alpha: float, initial_value: np.ndarray = None):
        if not 0 < alpha <= 1:
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha
        # 初始化平滑值。如果未提供，则设置为零向量。
        self.smoothed_value = initial_value if initial_value is not None else np.zeros(3)

    def update(self, raw_data: np.ndarray) -> np.ndarray:
        """
        应用 EMA 滤波公式:S_t = alpha * Y_t + (1 - alpha) * S_{t-1}
        """
        # 确保 raw_data 是 numpy 数组
        raw_data = np.asarray(raw_data)
        
        # 只有在初始化时（如果__init__没有设置）才将smoothed_value设置为raw_data
        if self.smoothed_value is None or np.all(self.smoothed_value == 0.0):
            self.smoothed_value = raw_data.copy()
        else:
            self.smoothed_value = self.alpha * raw_data + (1 - self.alpha) * self.smoothed_value
        
        return self.smoothed_value
    

class RigidBodyTracker:
    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('rigid_body_tracker', anonymous=True)

        # 2. 定义局部坐标 (Local Coordinates)
        sqrt_3 = np.sqrt(3)

        self.local_markers = np.array([
            [0.0, 0.0, -0.1175],               # Marker 1
            [-0.04, 0.04 * sqrt_3, -0.000003],    # Marker 2
            [0.03, 0.03 * sqrt_3, -0.000003], # Marker 3
            [0.035, -0.03 * sqrt_3, -0.000003],  # Marker 4
            [-0.03, -0.03 * sqrt_3, -0.000003]    # Marker 5
        ])
        # 3. 创建订阅者列表
        # ROS 1 中 message_filters.Subscriber 不需要传 node 对象，也没有 QoS 参数
        self.subs = []

        client_node_name = "vrpn_client_node"
        self.rigid_name = "mimic_tool"
        topoc_base = f"/{client_node_name}/{self.rigid_name}_Marker"

        # 订阅 Pose
        for i in range(1, 6):
            topic_name = "{}{}/pose".format(topoc_base, i)
            sub = message_filters.Subscriber(topic_name, PoseStamped)
            self.subs.append(sub)
            rospy.loginfo(f"Subscribed to topic: {topic_name}")

        # 订阅 Accel
        for i in range(1, 6):
            topic_name = "{}{}/accel".format(topoc_base, i)
            sub = message_filters.Subscriber(topic_name, TwistStamped)
            self.subs.append(sub)
            rospy.loginfo(f"Subscribed to topic: {topic_name}")

        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs,
            queue_size=10,
            slop=0.05
        )

        self.sync.registerCallback(self.sync_callback)

        self.publish_rate = 200  # 发布频率
        self.publish_interval = rospy.Duration(1.0 / self.publish_rate)
        self.last_publish_time = rospy.Time(0)

        self.last_pose = None
        self.last_quat = None
        self.last_time = None

        # --- LPF 参数 ---
        self.LPF_ALPHA = 0.8  # EMA 滤波器的 alpha 参数
        self.v_filter = EMAFilter(alpha=self.LPF_ALPHA)
        self.omega_filter = EMAFilter(alpha=self.LPF_ALPHA)
        
        self.a_filter = EMAFilter(alpha=self.LPF_ALPHA)
        self.alpha_dot_filter = EMAFilter(alpha=self.LPF_ALPHA)

        self.pub = rospy.Publisher('/mimic_tool/kinetic_state', KineticStateStamped, queue_size=1)
        

        rospy.loginfo("Rigid Body Tracker Initialized. Publishing pose and twist.")

    def sync_callback(self, *msgs: KineticStateStamped):
        """
        只有当五个marker的PoseStamped消息同时到达时才会调用此回调函数
        """
        current_time = msgs[0].header.stamp

        # 获取当前五个点的世界坐标
        current_point_pos = []
        for i in range(5):
            position = msgs[i].pose.position
            current_point_pos.append([(position.x)/1000, (position.y)/1000, (position.z)/1000])  # 转换为米

        # 验证是否为5个点
        if len(current_point_pos) != 5:
            rospy.logwarn("Expected 5 marker points, but received {}".format(len(current_point_pos)))
            return

        current_point_accel = []
        for i in range(5, 10):
            acc = msgs[i].twist.linear
            current_point_accel.append([(acc.x)/1000, (acc.y)/1000, (acc.z)/1000])  # 转换为米/s²


        current_point_pos = np.array(current_point_pos)
        # 计算刚体变换矩阵
        try:
            T = self.calculate_rigid_transform(current_point_pos, self.local_markers)
        except Exception as e:
            rospy.logerr(f"Error in calculating rigid transform: {e}")
            return
        
        p_curr = T[:3, 3]
        q_curr = self.rot2quat(T[:3, :3])
        R_curr = T[:3, :3]


        # v_W, omega_W = self.calculate_twist(p_curr, q_curr, current_time)

        v_W, omega_W, a_W, alpha_dot_W = self.calculate_dynamics_with_measurements(
            p_curr, q_curr, R_curr, current_point_accel, current_time
        )

        self.publish_all(T, q_curr, v_W, omega_W, a_W, alpha_dot_W, current_time)


    def rot2quat(self, R):
        """
        将旋转矩阵转换为四元数
        输入:
        R: 3x3 numpy array, 旋转矩阵
        
        输出:
        q: 4x1 numpy array, 四元数 (w, x, y, z)
        """
        r11, r12, r13 = R[0]
        r21, r22, r23 = R[1]
        r31, r32, r33 = R[2]

        # 计算trace
        trace = np.trace(R)

        # 四元数分量初始化
        w, x, y, z = 0, 0, 0, 0

        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2  # S=4*w
            w = 0.25 * S
            x = (r32 - r23) / S
            y = (r13 - r31) / S
            z = (r21 - r12) / S
        else:
            i = np.argmax(np.diag(R))

            if i == 0: # R[0, 0] is largest (X)
                S = np.sqrt(1.0 + r11 - r22 - r33) * 2
                x = 0.25 * S
                w = (r32 - r23) / S
                y = (r12 + r21) / S
                z = (r13 + r31) / S
            elif i == 1: # R[1, 1] is largest (Y)
                S = np.sqrt(1.0 + r22 - r11 - r33) * 2
                y = 0.25 * S
                w = (r13 - r31) / S
                x = (r12 + r21) / S
                z = (r23 + r32) / S
            else: # R[2, 2] is largest (Z)
                S = np.sqrt(1.0 + r33 - r11 - r22) * 2
                z = 0.25 * S
                w = (r21 - r12) / S
                x = (r13 + r31) / S
                y = (r23 + r32) / S
        return np.array([w, x, y, z]) # 返回 (w, x, y, z) 顺序的数组

    def calculate_rigid_transform(self, global_points, local_points=None):
        """
        输入:
        local_points: Nx3 numpy array, 刚体局部坐标
        world_points: Nx3 numpy array, 世界坐标系下的对应点
        
        输出:
        T: 4x4 numpy array, 刚体从局部坐标系到世界坐标系的变换矩阵，或者说刚体在世界坐标系下的位姿
        """
        if local_points is None:
            local_points = np.array([
                [0, 0, 0.1175],
                [0.03 * np.sqrt(3), 0.03, -0.003],
                [0.035 * np.sqrt(3), -0.035, -0.003],
                [-0.03 * np.sqrt(3), -0.03, -0.003],
                [-0.04 * np.sqrt(3), 0.04, -0.003]
            ])
        
        A = np.array(local_points)
        B = np.array(global_points)

        assert A.shape == B.shape
        N = A.shape[0]

        # 计算质心
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # 去质心
        AA = A - centroid_A
        BB = B - centroid_B

        # 计算协方差矩阵
        H = np.dot(AA.T, BB)

        # SVD分解
        U, S, Vt = np.linalg.svd(H)

        # 计算旋转矩阵
        R = np.dot(Vt.T, U.T)

        # 处理反射情况
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)

        # 计算平移向量
        t = centroid_B - np.dot(R, centroid_A)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T

    def calculate_rigid_quat_poss(self, global_points, local_points=None):
        """
        输入:
        local_points: Nx3 numpy array, 刚体局部坐标
        world_points: Nx3 numpy array, 世界坐标系下的对应点
        
        输出:
        q: 4x1 numpy array, 刚体在世界坐标系下的四元数表示 (w, x, y, z)
        p: 3x1 numpy array, 刚体在世界坐标系下的位置
        """
        T = self.calculate_rigid_transform(global_points, local_points)
        R = T[:3, :3]
        t = T[:3, 3]

        quat = self.rot2quat(R)
        pos = t

        return pos, quat

    def calculate_twist(self, current_pose, current_quat, cuerrent_time):
        """
        有限差分计算刚体速度 世界坐标系下
        """
        if self.last_pose is None:
            self.last_time = cuerrent_time
            self.last_pose = current_pose
            self.last_quat = current_quat

            return self.v_filter.smoothed_value, self.omega_filter.smoothed_value
        
        p_prev = self.last_pose
        q_prev = self.last_quat
        t_prev = self.last_time

        p_curr = current_pose
        q_curr = current_quat
        t_curr = cuerrent_time

        dt = (t_curr - t_prev).to_sec()

        MIN_DT = 1.0 / (self.publish_rate * 2)  # 最小时间间隔，防止过快采样导致噪声过大

        if dt < MIN_DT:
            return self.v_filter.smoothed_value, self.omega_filter.smoothed_value
        

        v_W = (p_curr - p_prev) / dt

        q_prev_xyzw = np.array([q_prev[1], q_prev[2], q_prev[3], q_prev[0]])
        q_curr_xyzw = np.array([q_curr[1], q_curr[2], q_curr[3], q_curr[0]])

        q_prev_inv = tf_trans.quaternion_inverse(q_prev_xyzw)
        q_delta_xyzw = tf_trans.quaternion_multiply(q_curr_xyzw, q_prev_inv)

        axis_vec = q_delta_xyzw[0:3]
        w_delta = q_delta_xyzw[3]

        v_norm = np.linalg.norm(axis_vec)

        angle = 2.0 * np.arctan2(v_norm, w_delta)

        if angle > np.pi:
            angle -= 2.0 * np.pi
        elif angle < -np.pi:
            angle += 2.0 * np.pi
        
        if v_norm < 1e-6:
            omega_W = np.zeros(3)
        else:
            axis_unit = axis_vec / v_norm
            omega_W = (angle / dt) * axis_unit

        smoothed_v = self.v_filter.update(v_W)
        smoothed_omega = self.omega_filter.update(omega_W)

        self.last_time = cuerrent_time
        self.last_pose = current_pose
        self.last_quat = current_quat

        return smoothed_v, smoothed_omega

    def calculate_dynamics_with_measurements(self, p_curr, q_curr, R_curr, marker_accels, current_time):
            """
            使用有限差分计算速度(V, Omega),使用最小二乘法融合Marker加速度计算(a, alpha)
            """
            # --- A. 初始化与速度计算 (保持原有的差分逻辑) ---
            if self.last_pose is None:
                self.last_time = current_time
                self.last_pose = p_curr
                self.last_quat = q_curr
                return (np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
            
            p_prev = self.last_pose
            q_prev = self.last_quat
            t_prev = self.last_time
            dt = (current_time - t_prev).to_sec()
            
            MIN_DT = 0.001 # 简单保护
            
            if dt < MIN_DT:
                return (self.v_filter.smoothed_value, self.omega_filter.smoothed_value, 
                        self.a_filter.smoothed_value, self.alpha_dot_filter.smoothed_value)

            # 1. 计算速度 (有限差分) - 速度通常还是得算，除非动捕也给了 Twist
            v_raw = (p_curr - p_prev) / dt
            
            # 计算角速度
            q_prev_inv = tf_trans.quaternion_inverse([q_prev[1], q_prev[2], q_prev[3], q_prev[0]])
            q_curr_xyzw = [q_curr[1], q_curr[2], q_curr[3], q_curr[0]]
            q_delta = tf_trans.quaternion_multiply(q_curr_xyzw, q_prev_inv)
            
            axis = q_delta[:3]
            norm = np.linalg.norm(axis)
            angle = 2 * np.atan2(norm, q_delta[3])
            if angle > np.pi: angle -= 2*np.pi
            elif angle < -np.pi: angle += 2*np.pi
            
            if norm < 1e-6:
                omega_raw = np.zeros(3)
            else:
                omega_raw = (angle / dt) * (axis / norm)
                
            # 滤波速度
            v_smooth = self.v_filter.update(v_raw)
            omega_smooth = self.omega_filter.update(omega_raw)
            
            # --- B. 加速度求解 (最小二乘法) ---
            # 目标: 解方程 A * [a_body; alpha_dot] = b
            # 变量: x = [ax, ay, az, alpha_x, alpha_y, alpha_z] (6维)
            
            A_mat = [] # 系数矩阵
            b_vec = [] # 观测向量
            
            # 刚体角速度 omega (使用平滑后的值更稳定)
            w = omega_smooth
            
            for i in range(5):
                # r_i: 从刚体中心指向 Marker i 的向量 (世界坐标系)
                # local_markers[i] 是局部坐标
                r_i = np.dot(R_curr, self.local_markers[i]) 
                
                # a_measured: 测量的 Marker 加速度
                a_meas = marker_accels[i]
                
                # 向心加速度项: w x (w x r)
                # w_cross_r = np.cross(w, r_i)
                # centripetal = np.cross(w, w_cross_r)
                # 优化写法:
                centripetal = np.cross(w, np.cross(w, r_i))
                
                # 构造方程: a_body + alpha x r = a_meas - centripetal
                # 移项: a_body - r x alpha = a_meas - centripetal
                # 这里的 r x alpha = [r]x * alpha. 
                # 所以 alpha 的系数矩阵是 -[r]x (叉乘矩阵的负数)
                
                # 构造叉乘矩阵 [r]x
                rx, ry, rz = r_i
                r_skew = np.array([
                    [0, -rz, ry],
                    [rz, 0, -rx],
                    [-ry, rx, 0]
                ])
                
                # b_i = a_meas - centripetal
                rhs = a_meas - centripetal
                
                # 将这个点的方程加入列表
                # 方程形式: I*a_body + (-[r]x)*alpha = rhs
                # 对于每个点，产生3行方程
                
                # 填充大矩阵的一部分
                # [ 1  0  0   0   rz -ry ]
                # [ 0  1  0  -rz  0   rx ]
                # [ 0  0  1   ry -rx  0  ]
                
                coeff_block = np.hstack((np.eye(3), -r_skew)) # 3x6
                
                A_mat.append(coeff_block)
                b_vec.append(rhs)
                
            # 堆叠所有方程 (15行6列)
            A_full = np.vstack(A_mat)
            b_full = np.hstack(b_vec) # 扁平化为 15 维向量
            
            # 最小二乘求解
            # x_sol = [ax, ay, az, alpha_x, alpha_y, alpha_z]
            x_sol, residuals, rank, s = np.linalg.lstsq(A_full, b_full, rcond=None)
            
            a_raw = x_sol[:3]
            alpha_dot_raw = x_sol[3:]
            
            # 滤波加速度
            a_smooth = self.a_filter.update(a_raw)
            alpha_dot_smooth = self.alpha_dot_filter.update(alpha_dot_raw)
            
            # 更新历史
            self.last_time = current_time
            self.last_pose = p_curr
            self.last_quat = q_curr
            
            return v_smooth, omega_smooth, a_smooth, alpha_dot_smooth

    def publish_pose(self, T, quat=None, current_time=None):
        
        if current_time is None:
            current_time = rospy.Time.now()

        # 打印输出位置
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        if quat is None:
            w, qx, qy, qz = self.rot2quat(T[:3, :3])
        else:
            w, qx, qy, qz = quat
        
        # 构造PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "world"  # 根据实际情况设置参考坐标

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        pose_msg.pose.orientation.w = w
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.pose_pub.publish(pose_msg)

    def publish_twist(self, v_W, omega_W, current_time=None):
        if current_time is None:
            current_time = rospy.Time.now()

        twist_msg = TwistStamped()
        twist_msg.header.stamp = current_time
        twist_msg.header.frame_id = "world"  # 根据实际情况设置参考坐标

        twist_msg.twist.linear.x = v_W[0]
        twist_msg.twist.linear.y = v_W[1]
        twist_msg.twist.linear.z = v_W[2]

        twist_msg.twist.angular.x = omega_W[0]
        twist_msg.twist.angular.y = omega_W[1]
        twist_msg.twist.angular.z = omega_W[2]

        self.twist_pub.publish(twist_msg) 

    def publish_accel(self, a_W, alpha_dot_W, current_time=None):
        if current_time is None:
            current_time = rospy.Time.now()

        accel_msg = AccelStamped()
        accel_msg.header.stamp = current_time
        accel_msg.header.frame_id = "world"  # 根据实际情况设置参考坐标

        accel_msg.accel.linear.x = a_W[0]
        accel_msg.accel.linear.y = a_W[1]
        accel_msg.accel.linear.z = a_W[2]

        accel_msg.accel.angular.x = alpha_dot_W[0]
        accel_msg.accel.angular.y = alpha_dot_W[1]
        accel_msg.accel.angular.z = alpha_dot_W[2]

        self.acc_pub.publish(accel_msg)

    def publish_all(self, T, quat, v_W, omega_W, a_W, alpha_dot_W, current_time=None):
        if current_time is None:
            current_time = rospy.Time.now()

        kinetic_msg = KineticStateStamped()
        kinetic_msg.Header.stamp = current_time
        kinetic_msg.Header.frame_id = "world"

        # 填充 Pose
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        if quat is None:
            w, qx, qy, qz = self.rot2quat(T[:3, :3])
        else:
            w, qx, qy, qz = quat

        kinetic_msg.pose.position.x = x
        kinetic_msg.pose.position.y = y
        kinetic_msg.pose.position.z = z

        kinetic_msg.pose.orientation.w = w
        kinetic_msg.pose.orientation.x = qx
        kinetic_msg.pose.orientation.y = qy
        kinetic_msg.pose.orientation.z = qz

        # 填充 Twist
        kinetic_msg.twist.linear.x = v_W[0]
        kinetic_msg.twist.linear.y = v_W[1]
        kinetic_msg.twist.linear.z = v_W[2]

        kinetic_msg.twist.angular.x = omega_W[0]
        kinetic_msg.twist.angular.y = omega_W[1]
        kinetic_msg.twist.angular.z = omega_W[2]

        # 填充 Accel
        kinetic_msg.accel.linear.x = a_W[0]
        kinetic_msg.accel.linear.y = a_W[1]
        kinetic_msg.accel.linear.z = a_W[2]

        kinetic_msg.accel.angular.x = alpha_dot_W[0]
        kinetic_msg.accel.angular.y = alpha_dot_W[1]
        kinetic_msg.accel.angular.z = alpha_dot_W[2]

        self.pub.publish(kinetic_msg)

if __name__ == "__main__":
    try:
        tracker = RigidBodyTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass