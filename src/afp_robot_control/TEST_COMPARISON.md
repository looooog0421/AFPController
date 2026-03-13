# 测试类型对比说明

## 📊 两种测试方式

### 🖥️ MuJoCo仿真测试（离线开发）

**文件位置**: `tests/position_control_only.py`, `tests/force_control_impedance.py`

**特点**:
- ✅ 包含完整的MuJoCo仿真环境
- ✅ 可视化机器人运动
- ✅ 不需要真实硬件
- ✅ 不需要ROS环境
- ✅ 适合算法开发和调试

**运行方式**:
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/tests
python3 position_control_only.py
# 或
python3 force_control_impedance.py
```

**架构**:
```
Python脚本
  │
  ├─→ MuJoCo模型加载
  ├─→ MuJoCo viewer可视化
  ├─→ 阻抗控制器（本地）
  └─→ 仿真循环
```

---

### 🤖 ROS实机测试（在线部署）

**文件位置**: `tests/test_ros_impedance_control.py`, `launch/test_impedance_control.launch`

**特点**:
- ❌ 没有MuJoCo仿真
- ✅ 通过ROS连接真实机器人
- ✅ 发布轨迹到 `/reference_trajectory`
- ✅ 订阅传感器数据
- ✅ 适合真实系统测试

**运行方式**:
```bash
# Terminal 1: 启动阻抗控制器
roslaunch afp_robot_control cartesian_impedance_controller.launch

# Terminal 2: 运行测试
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position
```

**架构**:
```
test_ros_impedance_control.py
  │
  └─→ 发布 /reference_trajectory
        │
        ↓
cartesian_impedance_controller_node.py
  │
  ├─→ 订阅 /joint_states
  ├─→ 订阅 /netft_data
  ├─→ 阻抗控制计算
  └─→ 发布 /joint_position_command
        │
        ↓
      真实UR5e机器人
```

---

## 🔄 集成测试：MuJoCo + ROS

如果你想要**既有MuJoCo可视化，又能测试ROS接口**，有两种方案：

### 方案1: MuJoCo作为ROS节点（推荐用于算法验证）

创建一个MuJoCo仿真节点，模拟机器人硬件：

```python
#!/usr/bin/env python3
"""MuJoCo仿真节点 - 替代真实机器人"""
import rospy
import mujoco
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class MuJoCoSimNode:
    def __init__(self):
        # 加载MuJoCo模型
        self.model = mujoco.MjModel.from_xml_path("scene.xml")
        self.data = mujoco.MjData(self.model)
        
        # 发布关节状态
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        
        # 订阅控制指令
        rospy.Subscriber('/joint_position_command', Float32MultiArray, self.cmd_callback)
        
    def cmd_callback(self, msg):
        # 接收控制指令并应用到MuJoCo
        self.data.ctrl[:] = msg.data
        
    def run(self):
        rate = rospy.Rate(500)  # 500Hz
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while not rospy.is_shutdown():
                # 仿真步进
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                
                # 发布状态
                joint_msg = JointState()
                joint_msg.position = self.data.qpos[:6]
                joint_msg.velocity = self.data.qvel[:6]
                self.joint_pub.publish(joint_msg)
                
                rate.sleep()
```

**使用流程**:
```bash
# Terminal 1: MuJoCo仿真节点（替代真实机器人）
rosrun afp_robot_control mujoco_sim_node.py

# Terminal 2: 阻抗控制器
roslaunch afp_robot_control cartesian_impedance_controller.launch

# Terminal 3: 测试节点
roslaunch afp_robot_control test_impedance_control.launch
```

---

### 方案2: 独立仿真测试（当前方案）

继续使用独立的MuJoCo测试，不涉及ROS：

```bash
# 开发阶段：用MuJoCo测试
python3 tests/position_control_only.py

# 部署阶段：用ROS测试
roslaunch afp_robot_control test_impedance_control.launch
```

---

## 📝 使用建议

### 开发流程

1. **算法开发** → 使用MuJoCo仿真测试
   ```bash
   python3 tests/position_control_only.py
   python3 tests/force_control_impedance.py
   ```
   
2. **ROS集成验证** → 可选创建MuJoCo ROS节点
   ```bash
   # 如果需要验证ROS通信，创建mujoco_sim_node.py
   ```

3. **真机部署** → 使用ROS测试
   ```bash
   roslaunch afp_robot_control test_impedance_control.launch
   ```

---

## 🎯 快速选择指南

| 需求 | 推荐方案 | 文件 |
|-----|---------|------|
| 调试阻抗控制算法 | MuJoCo仿真 | `position_control_only.py` |
| 测试力控制功能 | MuJoCo仿真 | `force_control_impedance.py` |
| 验证ROS通信 | ROS测试 | `test_ros_impedance_control.py` |
| 真机运行 | ROS测试 | `test_ros_impedance_control.py` |
| 完整集成测试 | MuJoCo ROS节点 | 需要创建 |

---

## 💡 为什么ROS测试没有MuJoCo？

**原因**:
1. ROS测试节点的目的是**测试真实机器人**
2. 它只是一个**轨迹发布器**，负责发送目标位姿
3. 实际控制由 `cartesian_impedance_controller_node.py` 完成
4. MuJoCo仿真是**独立的开发工具**，不是部署系统的一部分

**如果需要MuJoCo可视化真机运行**:
- 可以录制真机数据然后回放到MuJoCo
- 或者创建一个MuJoCo ROS节点作为硬件模拟器

---

## 🚀 创建MuJoCo ROS节点（可选）

如果你需要，我可以帮你创建一个完整的MuJoCo ROS仿真节点，它会：
- 模拟UR5e机器人硬件
- 提供与真机相同的ROS接口
- 包含MuJoCo可视化
- 可以直接替换真实机器人进行测试

只需要告诉我是否需要这个功能！

---

更多信息:
- [ROS接口文档](ROS_INTERFACE.md)
- [测试说明](tests/README.md)
- [快速开始](QUICKSTART.md)
