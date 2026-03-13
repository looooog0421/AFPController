1. 启动机器人
    ```./start_ursim.sh```

2. 启动驱动
    ```roslaunch ur_robot_driver ur5e_bringup.launch```

3. 连接机器人与驱动

4. 切换控制器
    ```
    lgx@lgx-Default-string:~$ rosservice call /controller_manager/ \
    switch_controller   "{start_controllers: ['joint_group_vel_controller'], \
    stop_controllers: ['scaled_pos_joint_traj_controller'], \
    strictness: 2}"
    ```

    测试：
    ```
    rostopic pub -1 /joint_group_vel_controller/command std_msgs/Float64MultiArray   "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}"
    ```