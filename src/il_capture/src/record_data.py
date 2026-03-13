#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import message_filters
import pinocchio as pin
import numpy as np
import pandas as pd
import sys
import select
import termios
import tty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

class RobotDataRecorder:
    def __init__(self,
                 joint_state_topic=None,
                 ee_pose_topic=None,
                 ee_wrench_topic=None,
                 reference_pose_topic=None,
                 velocity_command_topic=None,
                 tf_switch=False,
                 urdf_path=None,
                 ee_link_name="tool0_controller"):
        
        rospy.init_node('robot_data_recorder', anonymous=True)

        # 1. 运动学配置 (Pinocchio)
        self.urdf_path = urdf_path
        self.ee_link_name = ee_link_name
        self.model = None
        if self.urdf_path:
            # 预加载模型，避免在记录时产生 IO 开销
            self.model = pin.buildModelFromUrdf(self.urdf_path)
            print(f"Model links: {list(self.model.names)}")
            self.pin_data = self.model.createData()
            self.ee_id = self.model.getFrameId(self.ee_link_name)
            rospy.loginfo(f"Pinocchio model loaded. Target Link: {self.ee_link_name}")

        # 2. TF 配置
        self.tf_switch = tf_switch
        if self.tf_switch:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 3. 动态话题订阅
        self.is_recording = False
        self.raw_data_buffer = []  # 用于临时存储同步后的原始消息

        topics = [
            (joint_state_topic, 'joint_state'),
            (ee_pose_topic, 'ee_pose'),
            (ee_wrench_topic, 'ee_wrench'),
            (reference_pose_topic, 'ref_pose'),
            (velocity_command_topic, 'vel_cmd')
        ]

        self.subs = []
        self.active_labels = []
        for topic, label in topics:
            if topic:
                sub = message_filters.Subscriber(topic, self._get_msg_type(label))
                self.subs.append(sub)
                self.active_labels.append(label)

        # 4. 时间同步器 (Slop=0.05s 允许不同频率传感器对齐)
        if self.subs:
            self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=50, slop=0.05, allow_headerless=True)
            self.ts.registerCallback(self.main_callback)
        
        rospy.loginfo(f"Recorder Initialized. Active Topics: {self.active_labels}")

    def _get_msg_type(self, label):
        mapping = {
            'joint_state': JointState, 'ee_pose': PoseStamped,
            'ee_wrench': WrenchStamped, 'ref_pose': PoseStamped,
            'vel_cmd': Float64MultiArray
        }
        return mapping[label]

    def start_recording(self):
        self.raw_data_buffer = []
        self.is_recording = True
        rospy.loginfo(">>>> START RECORDING <<<<")

    def stop_recording(self, filename="robot_experiment.csv", do_calc=True):
        self.is_recording = False
        rospy.loginfo(f">>>> STOP RECORDING. Samples: {len(self.raw_data_buffer)} <<<<")
        if self.raw_data_buffer:
            self.process_and_save(filename, do_calc)

    def main_callback(self, *args):
        if self.is_recording:
            # 封装当前时刻的数据快照
            current_batch = dict(zip(self.active_labels, args))
            
            # 实时抓取 TF (因为 TF 不进入 message_filters 同步)
            tf_snapshot = None
            if self.tf_switch:
                try:
                    tf_snapshot = self.tf_buffer.lookup_transform('base', 'tool0_controller', rospy.Time(0))
                except: pass

            self.raw_data_buffer.append({
                'msgs': current_batch,
                'tf': tf_snapshot,
                'time': rospy.get_time()
            })

    def process_and_save(self, filename, do_calc):
        rospy.loginfo("Processing data... please wait.")
        processed_data = []

        for entry in self.raw_data_buffer:
            msgs = entry['msgs']
            row = {'time_stamp': entry['time']}

            # 1. 关节数据
            if 'joint_state' in msgs:
                js = msgs['joint_state']
                for i, (p, v, effort) in enumerate(zip(js.position, js.velocity, js.effort)):
                    row[f'q_{i}'] = p
                    row[f'dq_{i}'] = v
                    row[f'effort_{i}'] = effort

            # 2. 传感器 Pose & 参考 Pose
            for key in ['ee_pose', 'ref_pose']:
                if key in msgs:
                    p = msgs[key].pose
                    row.update({f'{key}_x': p.position.x, f'{key}_y': p.position.y, f'{key}_z': p.position.z,
                                f'{key}_qx': p.orientation.x, f'{key}_qy': p.orientation.y, 
                                f'{key}_qz': p.orientation.z, f'{key}_qw': p.orientation.w})

            # 3. 力矩数据
            if 'ee_wrench' in msgs:
                f = msgs['ee_wrench'].wrench.force
                t = msgs['ee_wrench'].wrench.torque
                row.update({'fx': f.x, 'fy': f.y, 'fz': f.z, 'tx': t.x, 'ty': t.y, 'tz': t.z})

            # 4. TF 数据
            if entry['tf']:
                trans = entry['tf'].transform.translation
                rot = entry['tf'].transform.rotation
                row.update({'tf_x': trans.x, 'tf_y': trans.y, 'tf_z': trans.z,
                            'tf_qx': rot.x, 'tf_qy': rot.y, 'tf_qz': rot.z, 'tf_qw': rot.w})

            # 5. 速度命令数据
            if 'vel_cmd' in msgs:
                cmd = msgs['vel_cmd'].data
                row.update({f'vel_cmd_{i}': v for i, v in enumerate(cmd)})

            # 5. 离线运动学计算 (Pinocchio)
            if do_calc and self.model and 'joint_state' in msgs:
                q = np.array(msgs['joint_state'].position)
                
                # 计算关节运动学
                pin.forwardKinematics(self.model, self.pin_data, q)
                
                # 【关键：必须刷新 Frame 位姿，否则 oMf 中的数据不会随 q 更新】
                pin.updateFramePlacements(self.model, self.pin_data)
                
                # 【修改：使用 oMf 访问 Frame 位姿】
                m_pose = self.pin_data.oMf[self.ee_id]
                
                row.update({'pin_x': m_pose.translation[0], 
                            'pin_y': m_pose.translation[1], 
                            'pin_z': m_pose.translation[2]})
                
                # 旋转部分保持不变
                quat_pin = pin.Quaternion(m_pose.rotation)
                row.update({'pin_qx': quat_pin.x, 'pin_qy': quat_pin.y, 
                            'pin_qz': quat_pin.z, 'pin_qw': quat_pin.w})

            processed_data.append(row)

        # 导出 CSV
        df = pd.DataFrame(processed_data)
        df.to_csv(filename, index=False)
        rospy.loginfo(f"Successfully saved to {filename}")

# --- 键盘控制逻辑 ---
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    # 实例化 (在此处配置你的话题名和 URDF 路径)
    recorder = RobotDataRecorder(
        joint_state_topic="/joint_states",
        ee_pose_topic="/mujoco/ee_pose",      # 示例话题
        ee_wrench_topic="/mujoco/ee_wrench",            # 示例话题
        reference_pose_topic="/reference_trajectory",     # 示例话题
        velocity_command_topic="/joint_group_vel_controller/command",       # 示例话题
        tf_switch=False,
        urdf_path="/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf", # 替换为真实的URDF路径
        ee_link_name="tool0"                  # 替换为URDF中的末端名称
    )

    # 导出路径
    export_dir = "/home/lgx/Project/AFP/src/il_capture/data/experiments/"
    timestamp = int(rospy.get_time())
    export_path = f"{export_dir}experiment_{timestamp}.csv"

    print("\n" + "="*30)
    print("CONTROL PANEL:")
    print("Press [s] to START recording")
    print("Press [e] to STOP and SAVE to CSV")
    print("Press [q] to QUIT")
    print("="*30 + "\n")

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key == 's':
                recorder.start_recording()
            elif key == 'e':
                recorder.stop_recording(export_path)
            elif key == 'q':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)