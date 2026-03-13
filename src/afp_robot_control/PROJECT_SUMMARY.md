# 笛卡尔空间阻抗控制器 - 项目交付文档

## 📋 项目概述

已成功为您设计并实现了一个**通用、模块化、可扩展**的笛卡尔空间阻抗控制器。该控制器专为AFP（自动纤维铺放）任务设计，同时支持多种其他应用场景。

---

## 🎯 已实现的功能

### 1. 核心架构（4层分层设计）

```
ROS接口层 → 任务策略层 → 阻抗控制核心 → 运动学层
```

- ✅ **完全模块化**：各层独立，便于复用和扩展
- ✅ **策略可插拔**：轻松切换不同任务策略
- ✅ **运动学解耦**：支持更换运动学库

### 2. 已实现的文件清单

#### 核心模块 (`src/impedance_control/`)
| 文件 | 功能 | 状态 |
|------|------|------|
| `impedance_types.py` | 数据结构定义 | ✅ 完成 |
| `task_strategy.py` | 任务策略（标准/AFP/混合） | ✅ 完成 |
| `coordinate_transform.py` | 坐标系转换 | ✅ 完成 |
| `robot_kinematics_wrapper.py` | Pinocchio运动学封装 | ✅ 完成 |
| `cartesian_impedance_controller.py` | 阻抗控制核心 | ✅ 完成 |

#### ROS节点与接口
| 文件 | 功能 | 状态 |
|------|------|------|
| `scripts/cartesian_impedance_controller_node.py` | ROS节点（200Hz） | ✅ 完成 |
| `msg/ImpedanceParams.msg` | 阻抗参数消息 | ✅ 完成 |
| `msg/JointPositionCommand.msg` | 关节指令消息 | ✅ 完成 |

#### 配置与启动
| 文件 | 功能 | 状态 |
|------|------|------|
| `config/impedance_controller_default.yaml` | 默认配置 | ✅ 完成 |
| `config/afp_task_config.yaml` | AFP专用配置 | ✅ 完成 |
| `launch/cartesian_impedance_controller.launch` | 通用启动 | ✅ 完成 |
| `launch/afp_impedance_controller.launch` | AFP启动 | ✅ 完成 |

#### 文档与测试
| 文件 | 功能 | 状态 |
|------|------|------|
| `IMPEDANCE_CONTROLLER_README.md` | 详细使用文档 | ✅ 完成 |
| `CMAKE_SETUP.md` | CMake配置说明 | ✅ 完成 |
| `scripts/test_basic_modules.py` | 基础模块测试 | ✅ 完成 |
| `scripts/test_impedance_controller.py` | 完整测试 | ✅ 完成 |
| `install_dependencies.sh` | 依赖安装脚本 | ✅ 完成 |

---

## 🔧 核心特性详解

### 特性1: 多种任务策略

#### 标准策略 (StandardStrategy)
```python
# 适用于常规阻抗控制
strategy = StandardStrategy(target_wrench=np.zeros(6))
```
- 直接位姿误差计算
- 四元数姿态控制
- 适用：拖动示教、装配任务

#### AFP策略 (AFPStrategy) ⭐
```python
# 您的AFP任务专用
strategy = AFPStrategy(target_contact_force=20.0)
```
- **核心逻辑**: `sqrt(fx^2 + fz^2) = F_target`
- **力分配**: `Δfx/Δfz = fx/fz`
- 末端x-z平面合力恒定控制
- 其他3自由度标准阻抗

#### 混合策略 (HybridStrategy)
```python
# 部分自由度力控，部分位置控
strategy = HybridStrategy(
    force_controlled_dofs=[2],  # Z轴力控
    target_wrench=[0, 0, 10, 0, 0, 0]
)
```

### 特性2: 坐标系转换

```python
# 传感器相对末端旋转90度
transformer = CoordinateTransformer(rotation_axis='z', angle_deg=90.0)
wrench_ee = transformer.transform_wrench_to_ee(wrench_sensor)
```

- 支持任意轴旋转配置
- 自动处理力/力矩变换
- 配置文件灵活设置

### 特性3: 动态参数调整

```python
# 实时发布新的阻抗参数
# 格式: [K1...K6, D1...D6]
rostopic pub /impedance_params_dynamic std_msgs/Float32MultiArray \
  "data: [500,500,500,10,10,10, 50,50,50,1,1,1]"
```

### 特性4: 位置型阻抗控制

控制流程：
1. 位姿误差: `e = x_ref - x_curr`
2. 阻抗控制: `F_d = K*e + D*ė`
3. 力误差: `ΔF = F_d - F_measured`
4. 位置修正: `Δx = (K + λI)^(-1) * ΔF`
5. 修正目标: `x_target = x_ref + Δx`
6. 逆运动学: `q_target = IK(x_target)`

---

## 📦 文件组织结构

```
src/afp_robot_control/
├── src/impedance_control/          # 核心模块（新增）
│   ├── __init__.py
│   ├── impedance_types.py
│   ├── task_strategy.py
│   ├── coordinate_transform.py
│   ├── robot_kinematics_wrapper.py
│   └── cartesian_impedance_controller.py
│
├── scripts/                         # 脚本
│   ├── cartesian_impedance_controller_node.py  # 新ROS节点
│   ├── test_basic_modules.py       # 基础测试
│   ├── test_impedance_controller.py # 完整测试
│   ├── impedance_feeder_node.py    # 您的原有节点（保留）
│   └── impedance_controller.py     # 您的原有代码（保留）
│
├── msg/                            # 消息定义（新增）
│   ├── ImpedanceParams.msg
│   └── JointPositionCommand.msg
│
├── config/                         # 配置文件（新增）
│   ├── impedance_controller_default.yaml
│   └── afp_task_config.yaml
│
├── launch/                         # 启动文件（新增）
│   ├── cartesian_impedance_controller.launch
│   └── afp_impedance_controller.launch
│
├── IMPEDANCE_CONTROLLER_README.md  # 详细文档
├── CMAKE_SETUP.md                  # CMake配置说明
└── install_dependencies.sh         # 依赖安装脚本
```

**重要**: 所有新文件都在独立目录中，**完全没有覆盖您的现有代码**！

---

## 🚀 快速开始指南

### 步骤1: 安装依赖

```bash
cd /home/lgx/Project/AFP/src/afp_robot_control
bash install_dependencies.sh
```

或手动安装：
```bash
pip3 install numpy scipy pin
```

### 步骤2: 配置CMakeLists.txt

参考 `CMAKE_SETUP.md`，在 `CMakeLists.txt` 中添加消息生成配置。

### 步骤3: 编译

```bash
cd ~/Project/AFP
catkin_make
source devel/setup.bash
```

### 步骤4: 配置参数

编辑 `config/afp_task_config.yaml`：

```yaml
# AFP任务配置
task_type: "afp"
control_frequency: 200.0

# URDF路径（根据您的实际路径修改）
urdf_path: "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"

# AFP阻抗参数
default_stiffness: [300.0, 1000.0, 300.0, 15.0, 5.0, 15.0]
default_damping: [30.0, 80.0, 30.0, 2.0, 0.5, 2.0]

# 传感器旋转配置
sensor:
  rotation_axis: "z"
  rotation_angle: 90.0  # 根据您的实际安装调整

# AFP参数
afp:
  target_contact_force: 25.0  # 根据您的工艺要求调整
```

### 步骤5: 启动控制器

```bash
roslaunch afp_robot_control afp_impedance_controller.launch
```

### 步骤6: 发布参考轨迹

```bash
# 示例：发布参考位姿
rostopic pub /reference_trajectory geometry_msgs/PoseStamped \
  "header:
    frame_id: 'base_link'
  pose:
    position: {x: 0.5, y: 0.0, z: 0.3}
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}"
```

---

## 🧪 测试验证

### 基础模块测试（不需要Pinocchio）

```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/scripts
python3 test_basic_modules.py
```

测试内容：
- ✅ 数据结构
- ✅ 任务策略
- ✅ 坐标转换
- ✅ 阻抗参数

### 完整测试（需要Pinocchio）

```bash
python3 test_impedance_controller.py
```

测试内容：
- ✅ 运动学计算
- ✅ 完整控制循环
- ✅ IK求解

---

## 📊 ROS话题接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（必需） |
| `/netft_data` | `geometry_msgs/WrenchStamped` | 力传感器数据（必需） |
| `/reference_trajectory` | `geometry_msgs/PoseStamped` | 参考轨迹（必需） |
| `/impedance_params_dynamic` | `std_msgs/Float32MultiArray` | 动态阻抗参数（可选） |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_position_command` | `std_msgs/Float32MultiArray` | 关节位置指令 |
| `/impedance_debug` | `std_msgs/Float32MultiArray` | 调试信息 |

---

## 🔍 AFP特殊逻辑说明

您的AFP任务的特殊需求已完美实现：

### 核心算法

```python
# 末端坐标系x-z平面合力控制
F_current = sqrt(fx^2 + fz^2)
delta_F = F_target - F_current

# 按当前力比例分配调整量
if F_current > threshold:
    ratio_x = fx / F_current
    ratio_z = fz / F_current
else:
    ratio_x = 0.0
    ratio_z = 1.0

delta_fx = delta_F * ratio_x
delta_fz = delta_F * ratio_z
```

### 设计优势

1. **策略层实现**: 不影响底层架构，易于复用
2. **参数可调**: `target_contact_force` 动态设置
3. **平滑过渡**: `min_force_threshold` 避免除零
4. **其他自由度标准控制**: y平移、x/z转动不受影响

---

## 🎓 使用场景示例

### 场景1: AFP纤维铺放（您的任务）

```yaml
# config/afp_task_config.yaml
task_type: "afp"
afp:
  target_contact_force: 25.0
default_stiffness: [300, 1000, 300, 15, 5, 15]
```

### 场景2: 表面打磨

```yaml
task_type: "hybrid"
hybrid:
  force_controlled_dofs: [2]  # Z轴力控
  target_wrench: [0, 0, 15, 0, 0, 0]
default_stiffness: [800, 800, 200, 20, 20, 20]
```

### 场景3: 装配任务

```yaml
task_type: "standard"
standard:
  target_wrench: [0, 0, -5, 0, 0, 0]  # 向下5N
default_stiffness: [1000, 1000, 500, 30, 30, 30]
```

---

## ⚙️ 参数调优建议

### AFP任务推荐参数

```yaml
# 刚度配置
# x,z方向柔顺（低刚度）→ 适应曲面
# y方向较硬（高刚度）→ 避免侧向偏移
default_stiffness: [300, 1000, 300, 15, 5, 15]

# 阻尼配置
# 平衡响应速度和稳定性
default_damping: [30, 80, 30, 2, 0.5, 2]

# AFP目标力
afp:
  target_contact_force: 20-30  # 根据材料特性调整
```

### 调试流程

1. **初始阶段**: 降低刚度（100-300），提高阻尼
2. **稳定后**: 逐步增加刚度，优化跟踪精度
3. **力控调试**: 调整 `target_contact_force`
4. **坐标系校准**: 验证传感器旋转配置

---

## 📝 后续开发建议

### 1. 自定义策略模板

```python
from impedance_control.task_strategy import TaskStrategy

class MyStrategy(TaskStrategy):
    def compute_error(self, current, reference, wrench):
        # 您的误差计算逻辑
        pass
    
    def compute_desired_wrench(self, current_wrench, ref_wrench, error):
        # 您的期望力计算逻辑
        pass
```

### 2. 参数自适应

可以添加一个独立节点，根据任务状态动态发布阻抗参数：

```python
# 伪代码
if contact_stable:
    K = high_stiffness
else:
    K = low_stiffness

pub.publish(new_impedance_params)
```

### 3. 轨迹规划器集成

当前控制器订阅 `/reference_trajectory`，您可以开发独立的轨迹规划节点。

---

## 🐛 常见问题排查

### 问题1: 模块导入失败

**症状**: `ModuleNotFoundError: No module named 'impedance_control'`

**解决**:
```bash
# 确保src目录在Python路径中
export PYTHONPATH=$PYTHONPATH:/home/lgx/Project/AFP/src/afp_robot_control/src
```

### 问题2: Pinocchio安装失败

**解决**:
```bash
# 方法1: pip
pip3 install pin

# 方法2: conda
conda install pinocchio -c conda-forge

# 方法3: 从源码编译
# 参考: https://stack-of-tasks.github.io/pinocchio/
```

### 问题3: IK不收敛

**症状**: 控制器输出 `IK convergence failed`

**排查**:
1. 检查参考轨迹是否在工作空间内
2. 降低 `correction_damping` (默认0.1)
3. 检查关节限位设置

### 问题4: 力控制振荡

**排查**:
1. 增大阻尼参数
2. 降低刚度参数
3. 检查传感器噪声和滤波
4. 调整 `correction_damping`

---

## 📞 技术支持

### 文档位置

- 详细使用手册: `IMPEDANCE_CONTROLLER_README.md`
- CMake配置: `CMAKE_SETUP.md`
- 代码注释: 所有模块都有详细注释

### 调试技巧

```bash
# 查看实时日志
rostopic echo /rosout | grep impedance

# 监控调试数据
rostopic echo /impedance_debug

# 可视化
rqt_plot /impedance_debug/data[0]:data[1]:data[2]
```

---

## ✅ 交付清单总结

| 项目 | 状态 | 备注 |
|------|------|------|
| 核心阻抗控制模块 | ✅ | 5个Python文件 |
| 任务策略（标准/AFP/混合） | ✅ | 可扩展 |
| ROS节点（200Hz） | ✅ | 完整实现 |
| 消息定义 | ✅ | 2个.msg文件 |
| 配置文件 | ✅ | 默认+AFP配置 |
| Launch文件 | ✅ | 2个启动文件 |
| 测试脚本 | ✅ | 基础+完整测试 |
| 文档 | ✅ | 3份文档 |
| 依赖安装脚本 | ✅ | install_dependencies.sh |
| 与现有代码兼容 | ✅ | 无覆盖 |

---

## 🎉 总结

我们成功设计并实现了一个：

✅ **通用性强**: 支持多种任务场景  
✅ **模块化好**: 各层独立，易于维护  
✅ **可扩展性**: 轻松添加新策略  
✅ **AFP优化**: 特殊逻辑完美实现  
✅ **文档完善**: 详细的使用说明  
✅ **测试充分**: 单元测试和集成测试  
✅ **零覆盖**: 完全不影响现有代码  

您现在可以：
1. 直接用于AFP纤维铺放任务
2. 轻松扩展到其他应用场景
3. 根据需求修改和开发新功能

祝您的AFP项目顺利！🚀
