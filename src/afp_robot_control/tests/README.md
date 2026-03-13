# 测试程序说明

本目录包含AFP机器人控制系统的测试程序，分为**MuJoCo仿真测试**和**ROS实机测试**。

---

## 🤖 ROS实机测试 ⭐⭐⭐

### test_ros_impedance_control.py
**测试内容**：ROS环境下的阻抗控制完整功能测试

**功能特点**：
- 4种测试模式：固定位置、圆形轨迹、XY运动、接触接近
- 自动发布参考轨迹到 `/reference_trajectory`
- 动态调整阻抗参数
- 监控关节状态和力传感器
- 接触检测和柔顺切换

**快速启动**：
```bash
# Terminal 1: 启动阻抗控制器
roslaunch afp_robot_control cartesian_impedance_controller.launch

# Terminal 2: 运行测试（固定位置模式）
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position

# 其他模式
roslaunch afp_robot_control test_impedance_control.launch test_mode:=circular
roslaunch afp_robot_control test_impedance_control.launch test_mode:=xy_motion
roslaunch afp_robot_control test_impedance_control.launch test_mode:=approach_contact
```

**详细说明**：参见 [ROS_TEST_GUIDE.md](ROS_TEST_GUIDE.md)

---

## 🖥️ MuJoCo仿真测试

## 🎯 主要测试（推荐使用）

### 1. force_control_impedance.py ⭐⭐⭐
**测试内容**：笛卡尔阻抗控制 + 接触力控制

**场景描述**：
- 机器人末端接近桌面
- 施加-10N的Z向接触力
- 在保持接触力的同时进行XY平面运动

**运行方法**：
```bash
python3 force_control_impedance.py
```

**测试阶段**：
1. **Phase 1**: 接近阶段（3秒）
   - 从初始位置移动到桌面上方10mm
   - 启用阻抗控制（无力参考）
   
2. **Phase 2**: 接触阶段（2秒）
   - 下降2mm到桌面上方2mm
   - 检测接触（力传感器 < -1N）
   - 平滑施加目标力（1秒线性过渡）
   
3. **Phase 3**: XY运动阶段（8秒）
   - 沿45度方向移动50mm
   - 保持Z向-10N接触力
   - 姿态保持不变

**关键参数**：
```python
# 阻抗参数
position_stiffness: [500, 500, 50] N/m
position_damping: [50, 50, 15] N·s/m
orientation_stiffness: [50, 50, 50] N·m/rad

# 力控制
target_force_z: -10.0 N
contact_threshold: -1.0 N
transition_duration: 1.0 s
```

**预期结果**：
- 位置误差：< 5mm
- 力控制误差：± 2N
- 姿态偏差：< 2°

---

### 2. position_control_only.py ⭐⭐⭐
**测试内容**：纯位置控制（无阻抗/力控制）

**目的**：
- 验证基础运动学控制正常工作
- 作为力控制测试的基准对比
- 排查阻抗控制相关问题

**运行方法**：
```bash
python3 position_control_only.py
```

**测试阶段**：
1. **阶段1**: 移动到接近位置（3秒）
   - 目标高度：Z = 0.43m（桌面上方30mm）
   
2. **阶段2**: 下降到接近桌面（2秒）
   - 目标高度：Z = 0.41m（桌面上方10mm）
   
3. **阶段3**: XY平面运动（10秒）
   - 沿45度方向移动100mm
   - 保持Z = 0.41m

**控制方式**：
```python
# IK求解目标关节角
q_ref = controller.robot_kin.inverse_kinematics(ref_pose, q_curr)

# 重力补偿
tau_gravity = pinocchio.computeGeneralizedGravity(model, data, q_curr)
delta_q = tau_gravity / kp_joints

# 发送控制指令
data.ctrl[:6] = q_ref + delta_q
```

**预期结果**：
- 轨迹跟踪平滑无震荡
- 位置误差：< 2mm
- 姿态稳定

---

## 📚 参考测试

### 3. test_06_pure_position.py
**内容**：早期纯位置控制测试

**特点**：
- 简单的点对点运动
- 固定目标位姿
- 无轨迹规划

**适用场景**：快速验证IK和FK功能

---

### 4. test_07_smooth_trajectory.py
**内容**：平滑轨迹生成测试

**特点**：
- 五次多项式轨迹
- 圆形轨迹测试
- 轨迹可视化

**适用场景**：验证轨迹规划算法

---

### 5. test_08_torque_control.py
**内容**：力矩控制测试

**特点**：
- 计算力矩指令
- 测试动力学模型
- 雅可比转换

**适用场景**：验证动力学计算

---

## 🚀 使用建议

### 调试新功能的推荐流程

1. **Step 1**: 运行 `position_control_only.py`
   - 确认基础位置控制工作正常
   - 验证场景配置正确
   - 检查运动学求解没有问题

2. **Step 2**: 如果Step 1通过，运行 `force_control_impedance.py`
   - 在已验证的基础上添加阻抗和力控制
   - 观察接触行为
   - 调整阻抗参数

3. **Step 3**: 参数调优
   - 如果力控制震荡 → 降低刚度或增加阻尼
   - 如果响应慢 → 增加刚度
   - 如果姿态漂移 → 增加姿态刚度

### 常见问题诊断

| 问题现象 | 可能原因 | 诊断方法 |
|---------|---------|---------|
| 位置跟踪抖动 | IK参数不当 | 运行position_control_only检查 |
| 接触时震荡 | 阻抗参数过高 | 降低刚度，从50 N/m开始 |
| 力控制不稳定 | 接触检测不准 | 检查力传感器读数和阈值 |
| 姿态漂移 | 姿态刚度太低 | 增加orientation_stiffness到50+ |
| IK失败 | 目标超出范围 | 检查目标位置是否在工作空间内 |

---

## 🔧 测试环境要求

### 软件依赖
```bash
# 必需
mujoco >= 3.3.7
pinocchio >= 3.8.0
numpy >= 1.20

# 可选（用于可视化）
matplotlib
```

### 场景文件
测试需要以下MuJoCo场景文件：
```
/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/
├── scene_contact.xml  # 带桌面的接触场景（主要使用）
├── scene.xml          # 基础场景
└── ur5e.urdf          # UR5e机器人URDF
```

### 硬件要求
- 仿真测试：普通PC即可（主频2.0GHz+）
- 实时运行：建议使用实时内核（可选）

---

## 📊 测试数据记录

运行测试后，关注以下指标：

### 位置控制性能
```
- 轨迹跟踪误差（RMS）
- 最大偏差
- 稳态误差
```

### 力控制性能
```
- 接触力稳态值
- 力控制误差（RMS）
- 响应时间（到达目标力95%）
```

### 姿态保持
```
- 姿态偏差（度）
- 最大姿态变化
```

---

## 🎓 学习路径

### 新手入门
1. 阅读 `position_control_only.py` 理解基础控制流程
2. 理解IK求解和重力补偿原理
3. 运行测试观察机器人运动

### 进阶开发
1. 阅读 `force_control_impedance.py` 理解阻抗控制
2. 学习阻抗参数对系统的影响
3. 尝试修改参数观察效果

### 高级应用
1. 参考测试文件编写自定义控制策略
2. 添加更复杂的轨迹规划
3. 集成力/位混合控制

---

## 📝 代码结构说明

### 典型测试文件结构
```python
# 1. 导入模块
import mujoco
import pinocchio
from impedance_control import CartesianImpedanceController

# 2. 加载模型
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 3. 初始化控制器
controller = CartesianImpedanceController(...)

# 4. 定义轨迹
def smooth_trajectory(t, t_total, p_start, p_end):
    # 五次多项式插值
    ...

# 5. 控制循环
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 生成参考轨迹
        pos_ref = smooth_trajectory(...)
        
        # 计算控制量
        tau = controller.compute_control(...)
        
        # 执行控制
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 💡 提示和技巧

1. **调试时使用慢速**：在测试开始时可以调整`time.sleep()`增加延迟，方便观察
2. **打印关键信息**：位置误差、力读数、姿态偏差等
3. **可视化轨迹**：可以在MuJoCo viewer中添加轨迹线
4. **保存数据**：记录测试数据用于后续分析
5. **参数备份**：找到好用的参数后及时记录

---

## ⚠️ 安全注意事项

虽然这些是仿真测试，但开发完成后将用于真实机器人：

1. **始终从低刚度开始**：避免过大的力
2. **设置力限制**：防止意外碰撞
3. **监控姿态**：大幅度姿态变化可能导致奇异
4. **工作空间限制**：确保目标在安全范围内
5. **紧急停止**：Ctrl+C可随时中断测试

---

更多信息请参考主目录的 [README.md](../README.md)
