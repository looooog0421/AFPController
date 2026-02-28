# MuJoCo仿真集成测试指南

## 概述

本指南说明如何使用MuJoCo仿真环境测试阻抗控制算法。系统包含三个ROS节点：

1. **mujoco_sim.py** - MuJoCo仿真节点（提供可视化）
2. **cartesian_impedance_controller_node.py** - 阻抗控制节点（核心算法）
3. **test_ros_impedance_control.py** - 测试轨迹生成节点

## 系统架构

```
测试节点 → 参考轨迹 → 控制节点 → 关节角度 → MuJoCo仿真
         /reference_trajectory    /joint_states
```

## 快速开始

### 1. 启动完整测试（推荐）

```bash
# Source环境
source /home/lgx/Project/AFP/devel/setup.bash

# 启动圆形轨迹测试（带MuJoCo可视化）
roslaunch afp_robot_control test_with_mujoco_sim.launch test_mode:=circular

# 或启动固定位置测试
roslaunch afp_robot_control test_with_mujoco_sim.launch test_mode:=fixed_position target_x:=0.6 target_y:=0.1 target_z:=0.3
```

### 2. 测试模式

#### 固定位置测试 (fixed_position)
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch \
    test_mode:=fixed_position \
    target_x:=0.5 \
    target_y:=0.0 \
    target_z:=0.4
```

#### 圆形轨迹测试 (circular)
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch test_mode:=circular
```
- 在XY平面绘制半径5cm的圆
- Z高度0.3m
- 周期10秒

#### XY平面运动测试 (xy_motion)
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch test_mode:=xy_motion
```
- 在XY平面做8字形运动

#### 接触测试 (approach_contact)
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch \
    test_mode:=approach_contact \
    approach_height:=0.5 \
    contact_threshold:=-1.0
```
- 从高处缓慢下降
- 检测到-1N的Z向力时停止
- 切换到力控模式

## 控制参数调整

### 阻抗参数
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch \
    Kp:=1500.0 \    # 位置刚度（默认1000）
    Kd:=60.0 \      # 阻尼（默认50）
    Ki:=15.0        # 积分增益（默认10）
```

### 控制频率
```bash
roslaunch afp_robot_control test_with_mujoco_sim.launch control_freq:=200
```

## 查看运行状态

### 1. 查看话题列表
```bash
rostopic list
```

应该看到：
- `/reference_trajectory` - 参考轨迹
- `/joint_states` - 关节状态（控制节点发布给MuJoCo）
- `/netft_data` - 力传感器数据（MuJoCo发布）
- `/joint_position_command` - 关节控制命令（Float32Array格式，调试用）
- `/impedance_debug` - 调试信息
- `/mujoco/ee_pose` - 末端位姿（MuJoCo发布）
- `/mujoco/ee_wrench` - 末端力/力矩（MuJoCo发布）
- `/mujoco/depth_camera/pointcloud` - 深度相机点云（MuJoCo发布）

### 2. 监控参考轨迹
```bash
rostopic echo /reference_trajectory
```

### 3. 查看控制命令
```bash
# 查看发送给MuJoCo的关节状态
rostopic echo /joint_states

# 或查看调试格式的命令
rostopic echo /joint_position_command
```

### 4. 查看调试信息
```bash
rostopic echo /impedance_debug
```

输出包含：
- 位置误差 (position_error)
- 速度误差 (velocity_error)
- 控制力 (control_force)
- 阻抗增益 (current_Kp, current_Kd, current_Ki)

## 故障排查

### MuJoCo窗口未打开

1. **检查afp_mjc包是否存在**
```bash
rospack find afp_mjc
```

2. **检查MuJoCo模型文件**
```bash
ls $(rospack find afp_mjc)/model/universal_robots_ur5e/scene.xml
```

3. **检查MuJoCo是否正确安装**
```bash
python3 -c "import mujoco; print(mujoco.__version__)"
```

### 机器人不动

1. **检查节点是否都在运行**
```bash
rosnode list
```
应该看到：
- `/cartesian_impedance_controller`
- `/impedance_test_node`
- `/mujoco_sim`

2. **查看控制节点日志**
```bash
rosnode info /cartesian_impedance_controller
```

3. **检查话题连接**
```bash
rostopic info /reference_trajectory
rostopic info /scaled_pos_joint_traj_controller/follow_joint_trajectory/goal
```

### 控制不稳定

调整阻抗参数：
```bash
# 降低刚度
roslaunch afp_robot_control test_with_mujoco_sim.launch Kp:=500.0

# 增加阻尼
roslaunch afp_robot_control test_with_mujoco_sim.launch Kd:=100.0
```

## 仅启动控制节点（不含测试）

如果需要单独运行控制节点（例如接收外部轨迹）：

```bash
# 启动MuJoCo仿真
rosrun afp_mjc mujoco_sim.py

# 启动阻抗控制节点
roslaunch afp_robot_control cartesian_impedance_controller.launch

# 手动发布轨迹（示例）
rostopic pub /reference_trajectory geometry_msgs/PoseStamped \
  "header:
    stamp: now
    frame_id: 'world'
  pose:
    position: {x: 0.5, y: 0.0, z: 0.4}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

## 与真实机器人的区别

| 特性 | MuJoCo仿真 | 真实UR5e |
|------|-----------|----------|
| 话题接口 | 相同 | 相同 |
| 控制算法 | 相同 | 相同 |
| 机器人模型 | MuJoCo XML | URDF |
| 延迟 | 几乎无延迟 | 网络+硬件延迟 |
| 力反馈 | 模拟 | 真实ATI传感器 |
| 安全性 | 无需担心 | 需要安全边界 |

## 从仿真切换到真实机器人

替换MuJoCo仿真节点为UR驱动：

```bash
# 启动真实UR5e驱动
roslaunch ur5e_control ur5e_bringup.launch

# 启动阻抗控制节点（相同）
roslaunch afp_robot_control cartesian_impedance_controller.launch

# 启动测试节点（相同）
roslaunch afp_robot_control test_ros_impedance_control.launch test_mode:=fixed_position
```

## 注意事项

1. **控制频率**：MuJoCo仿真支持高频控制（200Hz+），真实机器人受限于通信（通常125Hz）
2. **力控模式**：MuJoCo中的力传感器是模拟的，与真实传感器有差异
3. **安全性**：仿真中可以测试激进参数，真实机器人需谨慎

## 进一步开发

### 自定义轨迹

编辑 `test_ros_impedance_control.py` 添加新的轨迹生成函数：

```python
def generate_custom_trajectory(self):
    """自定义轨迹生成"""
    t = rospy.Time.now().to_sec() - self.start_time
    
    # 你的轨迹逻辑
    x = ...
    y = ...
    z = ...
    
    return [x, y, z]
```

### 修改阻抗控制算法

编辑 `CartesianImpedanceController` 类：
- 修改 `compute_control()` 方法
- 调整阻抗公式
- 添加新的传感器融合

### 添加新的测试场景

创建新的launch文件：
```xml
<launch>
    <include file="$(find afp_robot_control)/launch/test_with_mujoco_sim.launch">
        <arg name="test_mode" value="my_test" />
        <!-- 自定义参数 -->
    </include>
</launch>
```

## 相关文档

- [ROS接口文档](ROS_INTERFACE.md) - 完整的ROS API参考
- [快速开始](QUICKSTART.md) - 5分钟入门指南
- [测试对比](TEST_COMPARISON.md) - MuJoCo vs ROS测试说明
- [项目结构](PROJECT_STRUCTURE.md) - 完整代码结构
