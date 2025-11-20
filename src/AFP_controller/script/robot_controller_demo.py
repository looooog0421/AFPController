#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5e_controller import UR5eController
import ur5e_controller
import numpy as np


if __name__ == "__main__":
    # 初始化控制器
    controller = UR5eController()

    # 等待机器人状态更新
    while controller.robot_state.joint_state.position.sum() == 0.0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for robot state update...")
        rospy.sleep(0.1)

    # 机器人回到默认位置
    controller.move2default(0.5)

    # 机器人末端走直线运动， x方向前进0.2m， 然后回到原点
    cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
    print("Current EE Position:", cur_ee_pos)
    target_pos = cur_ee_pos + np.array([0.2, 0.0, 0.0]) # 在x方向前进0.2m
    print("Target Position (up):", target_pos)
    controller.move_line(target_pos, velocity=0.5)
    target_pos = cur_ee_pos # 回到原点
    print("Target Position (down):", target_pos)
    controller.move_line(target_pos, velocity=0.5)

    # 延时等待
    print("Waiting for 1 second...")
    rospy.sleep(1.0)
    

    # 机器人走一个sin曲线轨迹 z轴
    cur_ee_pos = np.array(controller.robot_state.ee_state.robot_trans[:3, 3])
    A = 0.1
    T = 10.0 # 周期10秒
    freq = 100.0
    steps = int(T * freq)
    import time
    start_time = time.time()
    for step in range(steps):
        t = step / freq
        z_offset = A * np.sin(2 * np.pi * t / T)
        target_pos = cur_ee_pos + np.array([0.0, 0.0, z_offset]).T
        print("target_pos: ", target_pos)
        controller.move_to_cartesian(target_pos, velocity=0.5, wait4complete=False)
        controller.rate.sleep()
    end_time = time.time()
    print(f"Completed sin wave trajectory in {end_time - start_time:.2f} seconds.")
        
    rospy.spin()