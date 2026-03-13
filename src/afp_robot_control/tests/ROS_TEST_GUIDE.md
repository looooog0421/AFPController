# ROS阻抗控制器测试节点使用说明

## 📋 功能概述

提供完整的ROS测试节点，用于测试笛卡尔阻抗控制器的各种功能。

## 🎯 测试模式

### 1. fixed_position - 固定位置保持
测试机器人保持固定位姿的能力。

**参数**:
- `target_position`: 目标位置 [x, y, z] (m)
- `target_orientation`: 目标姿态 [w, x, y, z] (四元数)

**运行**:
```bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position
```

---

### 2. circular - 圆形轨迹跟踪
测试轨迹跟踪能力，机器人末端沿圆形路径运动。

**参数**:
- `circle_radius`: 圆半径 (m)
- `circle_angular_vel`: 角速度 (rad/s)

**运行**:
```bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=circular circle_radius:=0.05
```

---

### 3. xy_motion - XY平面直线运动
测试平面内的直线运动，保持Z高度不变。

**参数**:
- `xy_start_y`: 起始Y坐标 (m)
- `xy_end_y`: 结束Y坐标 (m)
- `xy_duration`: 运动时间 (s)

**运行**:
```bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=xy_motion xy_start_y:=-0.1 xy_end_y:=0.1
```

---

### 4. approach_contact - 接近接触表面
测试接触检测和柔顺控制，机器人下降直到检测到接触。

**参数**:
- `approach_start_z`: 起始Z高度 (m)
- `approach_end_z`: 目标Z高度 (m)
- `approach_duration`: 下降时间 (s)

**运行**:
```bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=approach_contact approach_end_z:=0.41
```

---

## 🚀 完整测试流程

### Step 1: 启动阻抗控制器
```bash
# Terminal 1
roslaunch afp_robot_control cartesian_impedance_controller.launch
```

### Step 2: 运行测试节点
```bash
# Terminal 2
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position
```

### Step 3: 监控测试
```bash
# Terminal 3 - 查看参考轨迹
rostopic echo /reference_trajectory

# Terminal 4 - 查看控制指令
rostopic echo /joint_position_command

# Terminal 5 - 查看力传感器
rostopic echo /netft_data
```

---

## 🔧 参数调整

### 自定义测试参数

**方式1: 通过launch文件参数**
```bash
roslaunch afp_robot_control test_impedance_control.launch \
    test_mode:=circular \
    target_x:=0.6 \
    target_y:=0.0 \
    target_z:=0.45 \
    circle_radius:=0.08 \
    circle_angular_vel:=0.3
```

**方式2: 修改launch文件**
编辑 `launch/test_impedance_control.launch`，修改默认值。

**方式3: 通过rosparam**
```bash
rosparam set /impedance_controller_tester/circle_radius 0.08
```

---

## 📊 测试输出

### 终端输出示例

**固定位置模式**:
```
============================================================
Impedance Controller Tester Initialized
Test Mode: fixed_position
Control Frequency: 100.0 Hz
============================================================

============================================================
Test Mode: Fixed Position Control
Target Position: [0.5, 0.0, 0.4]
Target Orientation: [1.0, 0.0, 0.0, 0.0]
============================================================
Published impedance params: kp=[500, 500, 500], dp=[50, 50, 50]
Time: 1.0s, Force Z: -0.34N
Time: 2.0s, Force Z: -0.28N
Time: 3.0s, Force Z: -0.31N
...
Fixed position test completed!
```

**圆形轨迹模式**:
```
============================================================
Test Mode: Circular Trajectory
Center: [0.5, 0.0, 0.4], Radius: 0.05m
Angular Velocity: 0.5 rad/s
============================================================
Time: 0.0s, Angle: 0.0°, Pos: [0.550, 0.000, 0.400]
Time: 1.0s, Angle: 28.6°, Pos: [0.544, 0.024, 0.400]
Time: 2.0s, Angle: 57.3°, Pos: [0.527, 0.042, 0.400]
...
```

---

## 🐛 故障排查

### 问题1: 测试节点启动但无输出
**检查**:
```bash
rostopic list | grep reference_trajectory
```
**解决**: 确认阻抗控制器正在运行

---

### 问题2: 机器人不动
**检查**:
```bash
# 查看是否有控制指令
rostopic hz /joint_position_command

# 查看阻抗控制器日志
rosnode info /cartesian_impedance_controller
```
**解决**: 
1. 确认所有必需的topics都在发布（/joint_states, /netft_data）
2. 检查阻抗控制器配置

---

### 问题3: 轨迹不平滑
**原因**: 控制频率太低

**解决**: 提高控制频率
```bash
roslaunch afp_robot_control test_impedance_control.launch control_frequency:=200.0
```

---

## 📝 自定义测试模式

### 添加新的测试场景

编辑 `tests/test_ros_impedance_control.py`，添加新方法：

```python
def test_my_custom_mode(self):
    """自定义测试模式"""
    rospy.loginfo("Running custom test...")
    
    # 设置阻抗参数
    self.publish_impedance_params(
        position_stiffness=[500, 500, 50],
        position_damping=[50, 50, 15],
        orientation_stiffness=[50, 50, 50],
        orientation_damping=[10, 10, 10]
    )
    
    rate = rospy.Rate(self.control_frequency)
    
    while not rospy.is_shutdown():
        # 你的轨迹算法
        position = [0.5, 0.0, 0.4]
        msg = self.create_pose_msg(position, self.target_orientation)
        self.traj_pub.publish(msg)
        rate.sleep()
```

在 `run()` 方法中添加：
```python
elif self.test_mode == 'my_custom':
    self.test_my_custom_mode()
```

---

## 📈 性能测试

### 测量跟踪精度

添加订阅当前位姿：
```python
from geometry_msgs.msg import Pose

def __init__(self):
    # ... 现有代码 ...
    self.current_pose = None
    rospy.Subscriber('/current_pose', Pose, self.pose_callback)

def pose_callback(self, msg):
    self.current_pose = msg
```

计算误差：
```python
if self.current_pose:
    error_x = target_position[0] - self.current_pose.position.x
    error_y = target_position[1] - self.current_pose.position.y
    error_z = target_position[2] - self.current_pose.position.z
    error_norm = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    rospy.loginfo(f"Position Error: {error_norm*1000:.2f}mm")
```

---

## 🎓 使用建议

### 测试顺序

1. **先测试固定位置** → 验证基本功能
2. **测试XY运动** → 验证轨迹跟踪
3. **测试圆形轨迹** → 验证连续运动
4. **测试接触接近** → 验证力控制

### 参数调优

从保守参数开始：
```python
# 初始参数（安全）
position_stiffness=[100, 100, 20]
position_damping=[30, 30, 10]

# 逐步增加刚度
position_stiffness=[300, 300, 50]
position_stiffness=[500, 500, 100]
```

---

## 🔗 相关文档

- [ROS接口说明](../ROS_INTERFACE.md) - 完整接口文档
- [README.md](../README.md) - 包主文档
- [tests/README.md](README.md) - 测试说明

---

## 💡 提示

1. **测试前备份数据**: 记录参数和结果
2. **从慢速开始**: 低刚度、慢速度
3. **监控力传感器**: 避免过大接触力
4. **使用Ctrl+C安全停止**: 随时可以中断
