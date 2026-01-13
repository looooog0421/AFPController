# ROS接口说明文档

## 📡 ROS Topic接口

### 订阅的Topics (Subscribers)

#### 1. `/joint_states` (sensor_msgs/JointState)
**功能**: 接收机器人关节状态

**消息内容**:
```python
header:
  stamp: 时间戳
  frame_id: "base_link"
name: ['shoulder_pan_joint', 'shoulder_lift_joint', ...]
position: [q1, q2, q3, q4, q5, q6]  # 关节角度 (rad)
velocity: [dq1, dq2, dq3, dq4, dq5, dq6]  # 关节速度 (rad/s)
effort: [tau1, tau2, ...]  # 关节力矩 (可选)
```

**来源**: UR机器人驱动节点 (`ur_hardware_interface`)

---

#### 2. `/netft_data` (geometry_msgs/WrenchStamped)
**功能**: 接收力/力矩传感器数据

**消息内容**:
```python
header:
  stamp: 时间戳
  frame_id: "netft_frame"
wrench:
  force:
    x: Fx  # 力 (N)
    y: Fy
    z: Fz
  torque:
    x: Mx  # 力矩 (N·m)
    y: My
    z: Mz
```

**来源**: NetFT传感器驱动节点 (`netft_utils`)

**坐标系处理**:
- 支持传感器坐标系旋转配置
- 自动零点标定（前100个样本）
- 重力补偿

---

#### 3. `/reference_trajectory` (geometry_msgs/PoseStamped) ⭐
**功能**: 接收笛卡尔空间参考轨迹

**消息内容**:
```python
header:
  stamp: 时间戳
  frame_id: "base_link"
pose:
  position:
    x: X位置 (m)
    y: Y位置 (m)
    z: Z位置 (m)
  orientation:
    w: 四元数w
    x: 四元数x
    y: 四元数y
    z: 四元数z
```

**来源**: 轨迹规划节点（用户自定义）

**示例发布代码**:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('trajectory_publisher')
pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)

rate = rospy.Rate(100)  # 100Hz
while not rospy.is_shutdown():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    
    # 设置目标位置
    msg.pose.position.x = 0.5
    msg.pose.position.y = 0.3
    msg.pose.position.z = 0.4
    
    # 设置目标姿态（四元数）
    msg.pose.orientation.w = 1.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    
    pub.publish(msg)
    rate.sleep()
```

---

#### 4. `/impedance_params_dynamic` (std_msgs/Float32MultiArray)
**功能**: 动态调整阻抗参数

**消息格式**:
```python
data: [
    kp_x, kp_y, kp_z,           # 位置刚度 (N/m)
    dp_x, dp_y, dp_z,           # 位置阻尼 (N·s/m)
    ko_x, ko_y, ko_z,           # 姿态刚度 (N·m/rad)
    do_x, do_y, do_z            # 姿态阻尼 (N·m·s/rad)
]
# 总共12个浮点数
```

**示例**:
```python
from std_msgs.msg import Float32MultiArray

msg = Float32MultiArray()
msg.data = [
    500, 500, 50,    # 位置刚度: XY刚硬，Z柔顺
    50, 50, 15,      # 位置阻尼
    50, 50, 50,      # 姿态刚度
    10, 10, 10       # 姿态阻尼
]
pub.publish(msg)
```

---

### 发布的Topics (Publishers)

#### 1. `/joint_position_command` (std_msgs/Float32MultiArray)
**功能**: 发送关节位置指令

**消息格式**:
```python
data: [q1, q2, q3, q4, q5, q6]  # 目标关节角度 (rad)
```

**接收方**: UR机器人位置控制器

**控制频率**: 
- 默认: 125 Hz（UR5e限制）
- 配置: `control_frequency` 参数

---

#### 2. `/impedance_debug` (std_msgs/Float32MultiArray)
**功能**: 调试信息输出（需启用debug模式）

**消息格式**:
```python
data: [
    # 位置误差 (3个)
    ex, ey, ez,
    # 姿态误差 (3个)
    eRx, eRy, eRz,
    # 力/力矩误差 (6个)
    eFx, eFy, eFz, eMx, eMy, eMz,
    # 控制力 (6个)
    Fx_ctrl, Fy_ctrl, Fz_ctrl, Mx_ctrl, My_ctrl, Mz_ctrl
]
```

**启用方法**:
```yaml
# config/impedance_controller_default.yaml
debug_enabled: true
```

---

## 🔧 完整系统架构

### ROS节点连接图

```
┌─────────────────────┐
│  ur_hardware_       │
│  interface          │──→ /joint_states
└─────────────────────┘

┌─────────────────────┐
│  netft_utils        │──→ /netft_data
└─────────────────────┘

┌─────────────────────┐
│  trajectory_        │──→ /reference_trajectory  ⭐ 用户实现
│  planner (你写的)   │
└─────────────────────┘

        │ │ │
        ↓ ↓ ↓
┌─────────────────────────────────┐
│ cartesian_impedance_controller  │
│ (本包提供)                      │
│  - 阻抗控制算法                 │
│  - 运动学计算                   │
│  - 力控制                       │
└─────────────────────────────────┘
        │
        ↓
    /joint_position_command
        │
        ↓
┌─────────────────────┐
│  ur_hardware_       │
│  interface          │
└─────────────────────┘
```

---

## 🚀 使用流程

### 1. 启动阻抗控制器
```bash
roslaunch afp_robot_control cartesian_impedance_controller.launch
```

### 2. 启动你的轨迹规划节点
```bash
rosrun your_package trajectory_planner.py
```

你的轨迹规划节点需要：
- 订阅 `/joint_states` 获取当前状态（可选）
- 发布到 `/reference_trajectory` 发送目标位姿

### 3. 监控系统状态
```bash
# 查看topics
rostopic list

# 查看阻抗控制器输出
rostopic echo /joint_position_command

# 查看参考轨迹
rostopic echo /reference_trajectory

# 查看力传感器数据
rostopic echo /netft_data
```

---

## 📝 编写轨迹规划节点示例

### 简单示例：固定位置控制
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def simple_position_control():
    rospy.init_node('simple_trajectory')
    pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(100)  # 100Hz
    
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        # 固定目标位置
        msg.pose.position.x = 0.5
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.4
        
        # 姿态保持向下
        msg.pose.orientation.w = 0.707
        msg.pose.orientation.x = 0.707
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    simple_position_control()
```

### 进阶示例：圆形轨迹
```python
#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

def circular_trajectory():
    rospy.init_node('circular_trajectory')
    pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(100)  # 100Hz
    
    # 圆形轨迹参数
    center = np.array([0.5, 0.0, 0.4])  # 圆心
    radius = 0.05  # 半径 5cm
    angular_vel = 0.5  # 角速度 rad/s
    
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        t = (current_time - start_time).to_sec()
        
        # 计算圆形轨迹上的点
        angle = angular_vel * t
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        
        msg = PoseStamped()
        msg.header.stamp = current_time
        msg.header.frame_id = "base_link"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # 固定姿态
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    circular_trajectory()
```

### 高级示例：从文件读取轨迹
```python
#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class TrajectoryPlayer:
    def __init__(self, trajectory_file):
        rospy.init_node('trajectory_player')
        self.pub = rospy.Publisher('/reference_trajectory', PoseStamped, queue_size=10)
        
        # 加载轨迹文件 (CSV格式: t, x, y, z, qw, qx, qy, qz)
        self.trajectory = np.loadtxt(trajectory_file, delimiter=',')
        self.current_idx = 0
        
        rospy.Timer(rospy.Duration(0.01), self.publish_trajectory)  # 100Hz
        
    def publish_trajectory(self, event):
        if self.current_idx >= len(self.trajectory):
            rospy.loginfo("Trajectory finished")
            return
        
        traj_point = self.trajectory[self.current_idx]
        
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.pose.position.x = traj_point[1]
        msg.pose.position.y = traj_point[2]
        msg.pose.position.z = traj_point[3]
        
        msg.pose.orientation.w = traj_point[4]
        msg.pose.orientation.x = traj_point[5]
        msg.pose.orientation.y = traj_point[6]
        msg.pose.orientation.z = traj_point[7]
        
        self.pub.publish(msg)
        self.current_idx += 1

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: trajectory_player.py <trajectory_file.csv>")
        sys.exit(1)
    
    player = TrajectoryPlayer(sys.argv[1])
    rospy.spin()
```

---

## ⚙️ 配置文件

### impedance_controller_default.yaml
```yaml
# 机器人配置
urdf_path: "/path/to/ur5e.urdf"
ee_frame: "flange"
control_freq: 125.0  # Hz

# 任务类型: standard, afp, hybrid
task_type: "standard"

# 标准策略参数
standard_target_wrench: [0, 0, 0, 0, 0, 0]  # [Fx, Fy, Fz, Mx, My, Mz]

# 阻抗参数
impedance_params:
  position_stiffness: [500, 500, 50]      # [kp_x, kp_y, kp_z] N/m
  position_damping: [50, 50, 15]          # [dp_x, dp_y, dp_z] N·s/m
  orientation_stiffness: [50, 50, 50]     # [ko_x, ko_y, ko_z] N·m/rad
  orientation_damping: [10, 10, 10]       # [do_x, do_y, do_z] N·m·s/rad

# 力传感器配置
sensor_rotation_axis: "Z"    # 坐标系旋转轴
sensor_rotation_angle: 0.0   # 旋转角度 (度)

# 调试
debug_enabled: false
```

---

## 🔍 故障排查

### 问题1: 阻抗控制器收不到参考轨迹
**检查**:
```bash
# 查看是否有节点在发布
rostopic info /reference_trajectory

# 查看消息内容
rostopic echo /reference_trajectory
```

**解决**: 确保你的轨迹规划节点正在运行并发布消息

---

### 问题2: 机器人不动
**检查**:
```bash
# 查看是否有控制指令输出
rostopic echo /joint_position_command

# 查看是否接收到所有必需的数据
# 阻抗控制器会等待所有数据源就绪
```

**解决**: 
1. 确认 `/joint_states`, `/netft_data`, `/reference_trajectory` 都在发布
2. 查看阻抗控制器日志

---

### 问题3: 控制效果不好
**调试**:
```bash
# 启用debug模式
rosparam set /cartesian_impedance_controller/debug_enabled true

# 查看调试信息
rostopic echo /impedance_debug
```

**调整参数**: 通过动态参数调整
```python
from std_msgs.msg import Float32MultiArray
pub = rospy.Publisher('/impedance_params_dynamic', Float32MultiArray, queue_size=1)

msg = Float32MultiArray()
msg.data = [
    100, 100, 20,   # 降低刚度
    30, 30, 10,
    30, 30, 30,
    8, 8, 8
]
pub.publish(msg)
```

---

## 📚 相关文档

- [README.md](../README.md) - 包主文档
- [QUICKSTART.md](../QUICKSTART.md) - 快速开始
- [tests/README.md](../tests/README.md) - 测试说明

---

## 🎯 总结

**核心要点**:
1. ✅ 阻抗控制器节点已完整实现ROS接口
2. ✅ 通过 `/reference_trajectory` topic接收轨迹
3. ✅ 你只需要编写轨迹规划节点发布目标位姿
4. ✅ 支持动态参数调整和调试

**你需要做的**:
1. 编写轨迹规划节点
2. 发布 `PoseStamped` 消息到 `/reference_trajectory`
3. 根据任务需求设计轨迹算法
