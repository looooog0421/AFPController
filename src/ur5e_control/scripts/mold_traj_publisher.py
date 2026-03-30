#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模具轨迹发送节点
1. 加载 T_BM
2. 在模具坐标系下定义起点 A、终点 B（中线）
3. 插值生成 N 个轨迹点
4. 按 200Hz 流式发送 PoseStamped 到 /reference_trajectory
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import sys
import os


# ─────────────────────────────────────────────
# 1. 加载 T_BM
# ─────────────────────────────────────────────
T_BM_PATH = os.path.join(os.path.dirname(__file__), "T_BM.npy")

if not os.path.exists(T_BM_PATH):
    print(f"[ERROR] T_BM.npy 不存在: {T_BM_PATH}")
    print("请先运行 mold_calibration.py 生成标定结果")
    sys.exit(1)

T_BM = np.load(T_BM_PATH)
R_BM = T_BM[:3, :3]
t_BM = T_BM[:3,  3]

print("=" * 50)
print("已加载 T_BM:")
print(np.round(T_BM, 4))
print("=" * 50)


# ─────────────────────────────────────────────
# 2. 在模具坐标系 {M} 下定义轨迹
#    单位：m，坐标系与 SW 一致
#    修改这里来改变铺放路径
# ─────────────────────────────────────────────

# 平面模具悬空测试：沿 X 方向跑一条近似中线
# 根据你的 SW 坐标范围：X: 60~210 mm, Y: 10~120 mm
# 工作面高度 z_work = -25 mm
# 悬空 10 mm 测试高度 = -35 mm

A_M = np.array([0.070, 0.065, -0.035])   # 起点（模具坐标系，m）
B_M = np.array([0.200, 0.065, -0.035])   # 终点（模具坐标系，m）

# 轨迹点数：按速度和频率计算
# 路径长度 ≈ 130 mm，安全测试先用 5 mm/s
FREQ   = 200.0   # Hz，与控制器一致
SPEED  = 0.005   # m/s，末端期望速度（可调）
L      = np.linalg.norm(B_M - A_M)
N      = max(10, int(round(L / (SPEED / FREQ))))

print(f"路径长度: {L*1000:.1f} mm")
print(f"轨迹点数: {N}")
print(f"预计时间: {N/FREQ:.1f} s")
print("=" * 50)


# ─────────────────────────────────────────────
# 3. 生成轨迹点并变换到 Base 坐标系
# ─────────────────────────────────────────────
ts = np.linspace(0.0, 1.0, N)
waypoints_M = np.outer(1 - ts, A_M) + np.outer(ts, B_M)   # (N, 3)
waypoints_B = (R_BM @ waypoints_M.T).T + t_BM              # (N, 3)

# 工具姿态：固定，Z 轴对准模具法向（T_BM 的第3列），X 轴沿铺放方向
z_tool = R_BM[:, 2]                          # 模具 Z 轴在 Base 下
x_dir  = B_M - A_M
x_dir /= np.linalg.norm(x_dir)
x_tool = R_BM @ x_dir                        # 铺放方向在 Base 下
y_tool = np.cross(z_tool, x_tool)
y_tool /= np.linalg.norm(y_tool)
x_tool = np.cross(y_tool, z_tool)            # 重新正交化

R_tool = np.column_stack([x_tool, y_tool, z_tool])
quat   = R.from_matrix(R_tool).as_quat()     # [x, y, z, w]

print(f"工具姿态四元数 [x,y,z,w]: {np.round(quat, 4)}")
print("=" * 50)


# ─────────────────────────────────────────────
# 4. ROS 发送
# ─────────────────────────────────────────────
def main():
    rospy.init_node("mold_traj_publisher")
    pub  = rospy.Publisher("/reference_trajectory", PoseStamped, queue_size=1)
    rate = rospy.Rate(FREQ)

    # 等待控制器订阅
    rospy.sleep(1.0)

    # ── 安全确认 ──
    rospy.loginfo(f"即将发送 {N} 个轨迹点，起点 Base 坐标: {np.round(waypoints_B[0]*1000, 1)} mm")
    rospy.loginfo("先持续发送起点，等待确认...")

    # 持续发送起点，等用户确认
    confirmed = False
    while not rospy.is_shutdown() and not confirmed:
        _publish_pose(pub, waypoints_B[0], quat)
        rate.sleep()
        # 非阻塞检查：如果有参数 /mold_traj/go 被设为 true 则开始
        if rospy.has_param("/mold_traj/go") and rospy.get_param("/mold_traj/go"):
            confirmed = True

    if rospy.is_shutdown():
        return

    rospy.loginfo("开始执行轨迹...")
    rospy.set_param("/mold_traj/go", False)

    for i, pt in enumerate(waypoints_B):
        if rospy.is_shutdown():
            break
        _publish_pose(pub, pt, quat)
        rate.sleep()

    rospy.loginfo("轨迹执行完成。")

    # 执行完后持续发送终点，保持机器人位置
    rospy.loginfo("持续发送终点位姿，保持位置...")
    while not rospy.is_shutdown():
        _publish_pose(pub, waypoints_B[-1], quat)
        rate.sleep()


def _publish_pose(pub, pos, quat):
    msg = PoseStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.pose.position.x = float(pos[0])
    msg.pose.position.y = float(pos[1])
    msg.pose.position.z = float(pos[2])
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    pub.publish(msg)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
