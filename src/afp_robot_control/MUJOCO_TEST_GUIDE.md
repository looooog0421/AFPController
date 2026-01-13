# MuJoCo阻抗控制测试指南

## 📋 测试文件说明

我为您创建了两个MuJoCo测试脚本：

### 1. `test_mujoco_simple.py` - 快速测试 ⭐ 推荐先用这个

**用途**: 快速验证阻抗控制器基本功能
**特点**: 
- 简单直接，易于调试
- 固定位置跟踪测试
- 实时可视化
- 详细的状态输出

**使用方法**:
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/scripts
python3 test_mujoco_simple.py
```

**测试流程**:
1. 加载MuJoCo模型和URDF
2. 初始化阻抗控制器
3. 获取当前末端位姿
4. 设置目标位姿（偏移2cm）
5. 运行5秒仿真，观察跟踪效果

### 2. `test_mujoco_impedance.py` - 完整测试

**用途**: 完整的轨迹跟踪和策略测试
**特点**:
- 多种测试场景
- 支持AFP策略
- 圆形轨迹跟踪
- 数据记录和可视化

**使用方法**:
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/scripts
python3 test_mujoco_impedance.py
```

**测试选项**:
1. 固定位置保持（标准策略）
2. 圆形轨迹跟踪（标准策略）
3. 圆形轨迹跟踪（AFP策略）

---

## 🚀 快速开始

### 步骤1: 确认依赖

```bash
# 检查MuJoCo
python3 -c "import mujoco; print('✓ MuJoCo:', mujoco.__version__)"

# 检查Pinocchio
python3 -c "import pinocchio; print('✓ Pinocchio:', pinocchio.__version__)"

# 如果缺少依赖
pip install mujoco
pip install pin
```

### 步骤2: 检查文件路径

测试脚本会自动查找以下文件：

```
/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/
├── scene.xml      # MuJoCo场景文件
└── ur5e.urdf      # 机器人URDF文件
```

如果文件路径不同，您可以编辑测试脚本中的路径：

```python
# 在脚本顶部修改
model_path = "/your/path/to/scene.xml"
urdf_path = "/your/path/to/ur5e.urdf"
```

### 步骤3: 运行快速测试

```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/scripts
python3 test_mujoco_simple.py
```

**预期输出**:
```
============================================================
MuJoCo阻抗控制快速测试
============================================================

1. 加载MuJoCo模型...
   ✓ 自由度: 6, 控制输入: 6

2. 初始化阻抗控制器...
   ✓ 控制频率: 500.0 Hz

3. 获取初始状态...
   初始关节角度: [0.0, -45.0, 90.0, 0.0, 90.0, 0.0]

4. 正运动学计算...
   末端位置: [0.577, -0.134, 0.045]
   末端姿态: [0.270, 0.656, -0.651, -0.271]

5. 生成测试轨迹...
   目标位置: [0.597, -0.134, 0.045]
   位置偏移: Δx=20mm

6. 执行阻抗控制计算...
   ✓ 控制计算成功
   目标关节角度: [...]
   位置误差: 0.020000

7. 启动MuJoCo可视化...
   提示: 按ESC退出

开始仿真 (按Ctrl+C提前退出)...
   t=0.0s | 位置误差=20.00mm
   t=1.0s | 位置误差=5.23mm
   t=2.0s | 位置误差=1.42mm
   ...
```

---

## 🎯 测试场景详解

### 场景1: 固定位置保持

**目的**: 验证阻抗控制器能否维持目标位置

**实现**:
```python
# 参考轨迹保持不变
reference_pose = initial_pose
```

**观察指标**:
- 位置误差逐渐收敛到0
- 无外力时，机器人稳定在目标位置
- 添加外力后，表现出柔顺性

### 场景2: 圆形轨迹跟踪（标准策略）

**目的**: 测试动态轨迹跟踪能力

**实现**:
```python
# 在x-z平面画圆
pos[0] = center[0] + radius * cos(omega * t)
pos[2] = center[2] + radius * sin(omega * t)
```

**参数**:
- 半径: 5cm
- 角速度: 0.5 rad/s
- 周期: ~12.6秒

**观察指标**:
- 轨迹跟踪误差
- 速度和加速度平滑性
- 关节运动连续性

### 场景3: AFP策略测试

**目的**: 测试恒定接触力控制

**实现**:
```python
strategy = AFPStrategy(target_contact_force=20.0)
```

**测试方法**:
1. 让机器人接触一个表面
2. 观察x-z平面合力是否保持恒定
3. 验证力的分配是否按比例

---

## 🔧 参数调整

### 阻抗参数

在测试脚本中修改：

```python
# 各向同性
impedance_params = ImpedanceParams.create_uniform(
    stiffness=500.0,  # 刚度 (N/m)
    damping=50.0      # 阻尼 (Ns/m)
)

# 各向异性（推荐用于AFP）
impedance_params = ImpedanceParams.create_anisotropic(
    trans_stiffness=np.array([300, 1000, 300]),  # x,y,z
    rot_stiffness=np.array([10, 10, 10]),
    trans_damping=np.array([30, 80, 30]),
    rot_damping=np.array([1, 1, 1])
)
```

**调优建议**:
- 刚度太大 → 跟踪精度高但硬，可能振荡
- 刚度太小 → 柔顺但跟踪误差大
- 阻尼太大 → 响应慢
- 阻尼太小 → 容易振荡

### AFP参数

```python
strategy = AFPStrategy(
    target_contact_force=20.0,    # 目标接触力 (N)
    y_target_force=0.0,           # y方向力 (N)
    target_torque=np.zeros(3)     # 目标力矩 (Nm)
)
```

---

## 📊 数据可视化

### 使用matplotlib绘图

完整测试脚本支持数据可视化：

```bash
python3 test_mujoco_impedance.py
# 选择测试场景
# 测试结束后选择 'y' 绘制结果
```

**图表内容**:
1. 位置误差随时间变化
2. 接触力随时间变化

### 手动分析数据

数据存储在测试对象中：

```python
test_env.time_history          # 时间序列
test_env.pos_error_history     # 位置误差
test_env.force_history         # 接触力
test_env.joint_cmd_history     # 关节指令
```

---

## 🐛 常见问题

### 问题1: 找不到场景文件

**症状**:
```
✗ 找不到MuJoCo场景: .../scene.xml
```

**解决**:
1. 检查文件是否存在
2. 修改脚本中的路径
3. 或创建符号链接

### 问题2: MuJoCo viewer无法启动

**症状**:
```
Error: Could not initialize GLFW
```

**解决**:
```bash
# 确保有图形界面支持
export DISPLAY=:0

# 或使用无头模式（修改脚本，不使用viewer）
```

### 问题3: IK不收敛

**症状**:
```
Warning: IK did not converge after 100 iterations
```

**原因**:
- 目标位姿超出工作空间
- 阻抗参数导致目标偏移过大
- IK求解器参数需要调整

**解决**:
```python
# 调整IK参数
controller.robot_kin.set_ik_parameters(
    eps=1e-3,        # 收敛阈值（放宽）
    max_iter=200,    # 最大迭代次数
    damping=1e-4     # 阻尼参数
)
```

### 问题4: 仿真不稳定/振荡

**排查**:
1. 降低刚度参数
2. 增大阻尼参数
3. 检查MuJoCo时间步是否合适
4. 检查控制频率是否匹配

---

## 🎓 扩展开发

### 添加自定义轨迹

```python
def my_trajectory(t):
    """自定义轨迹函数"""
    pos = initial_pose.position.copy()
    
    # 您的轨迹逻辑
    pos[0] += 0.1 * np.sin(t)
    pos[2] += 0.05 * np.cos(2*t)
    
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    return CartesianState(position=pos, orientation=quat)

# 使用
test_env.run_trajectory_test(my_trajectory, duration=10.0)
```

### 添加接触力模拟

在MuJoCo场景中添加物体，测试接触：

```xml
<!-- 在scene.xml中添加 -->
<body name="contact_surface" pos="0.6 0 0">
  <geom type="box" size="0.1 0.2 0.01" rgba="0.8 0.8 0.8 1"/>
</body>
```

### 集成到ROS

结合MuJoCo仿真和ROS阻抗控制节点：

1. MuJoCo发布 `/joint_states`
2. MuJoCo发布 `/netft_data`（模拟）
3. 阻抗控制节点订阅数据
4. MuJoCo接收 `/joint_position_command`

---

## 📈 性能基准

**预期性能**（UR5e机器人）:

| 指标 | 值 |
|------|------|
| 位置跟踪误差（稳态） | < 1mm |
| 姿态跟踪误差 | < 0.01 rad |
| 控制频率 | 200-500 Hz |
| IK收敛率 | > 95% |
| 响应时间（阶跃） | < 0.5s |

---

## 🎉 测试检查清单

- [ ] 运行 `test_mujoco_simple.py` 成功
- [ ] MuJoCo可视化正常显示
- [ ] 位置误差收敛到<1mm
- [ ] 无明显振荡
- [ ] IK收敛率高
- [ ] 运行 `test_mujoco_impedance.py` 成功
- [ ] 圆形轨迹跟踪平滑
- [ ] AFP策略功能正常
- [ ] 数据可视化正常

---

## 📞 下一步

测试通过后，您可以：

1. **集成到ROS**: 使用真实的ROS话题
2. **实机测试**: 连接真实机器人
3. **参数优化**: 根据实际任务调优
4. **添加安全监控**: 限位、碰撞检测等

祝测试顺利！🚀
