# AFP机器人控制包 - 项目结构

## 📋 目录概览

```
afp_robot_control/
│
├── 📖 文档 (根目录)
│   ├── README.md                    # 主文档
│   ├── QUICKSTART.md               # ⭐ 快速开始
│   ├── IMPEDANCE_CONTROLLER_README.md  # 控制器详解
│   ├── MUJOCO_TEST_GUIDE.md        # （已过时，参考README）
│   ├── PROJECT_SUMMARY.md          # 项目总结
│   ├── CMAKE_SETUP.md              # CMake配置
│   └── PROJECT_STRUCTURE.md        # 本文档
│
├── 🧪 tests/ - 测试程序
│   ├── README.md                   # 测试文档
│   ├── force_control_impedance.py  # ⭐⭐⭐ 主要测试
│   ├── position_control_only.py    # ⭐⭐⭐ 基准测试
│   ├── test_06_pure_position.py   # 参考：位置控制
│   ├── test_07_smooth_trajectory.py  # 参考：轨迹规划
│   └── test_08_torque_control.py    # 参考：力矩控制
│
├── 💡 examples/ - 示例工具
│   ├── README.md                   # 示例说明
│   ├── check_kinematics.py        # 运动学验证
│   └── force_sensor_filter.py     # 力传感器滤波
│
├── 📦 archived/ - 历史文件
│   ├── README.md                   # 归档说明
│   └── test_*.py                   # 早期测试（已废弃）
│
├── 🎮 scripts/ - ROS节点
│   ├── cartesian_impedance_controller_node.py  # 笛卡尔阻抗控制
│   ├── impedance_controller.py                 # 关节阻抗控制
│   ├── impedance_feeder_node.py               # 轨迹输入节点
│   └── setup_path.py                          # 路径配置
│
├── 🔧 src/ - 核心源码
│   ├── impedance_control/         # 阻抗控制模块
│   │   ├── __init__.py
│   │   ├── controller.py          # 主控制器
│   │   ├── robot_kinematics_wrapper.py  # 运动学封装
│   │   ├── state.py              # 状态定义
│   │   └── strategy.py           # 控制策略
│   ├── core/                      # 核心功能
│   ├── ros/                       # ROS接口
│   └── utils/                     # 工具函数
│
├── ⚙️ config/ - 配置文件
│   ├── impedance_controller_default.yaml  # 默认参数
│   └── afp_task_config.yaml              # AFP任务配置
│
├── 🚀 launch/ - Launch文件
│   ├── cartesian_impedance_controller.launch
│   ├── hardware_drivers.launch
│   └── afp_impedance_controller.launch
│
├── 📨 msg/ - 消息定义
│   ├── ImpedanceParams.msg
│   └── JointPositionCommand.msg
│
└── 🛠️ 构建文件
    ├── CMakeLists.txt
    ├── package.xml
    └── install_dependencies.sh
```

---

## �� 使用场景快速导航

### 我想...

#### 🏃 快速开始
→ 阅读 [QUICKSTART.md](QUICKSTART.md)
→ 运行 `tests/position_control_only.py`

#### 🧪 运行测试
→ 阅读 [tests/README.md](tests/README.md)
→ 主要测试：`force_control_impedance.py`, `position_control_only.py`

#### 📖 理解原理
→ 阅读 [IMPEDANCE_CONTROLLER_README.md](IMPEDANCE_CONTROLLER_README.md)
→ 查看 `src/impedance_control/controller.py`

#### 🔧 开发新功能
→ 参考 `examples/check_kinematics.py`
→ 复制 `tests/position_control_only.py` 修改

#### 🚀 部署到机器人
→ 使用 `scripts/cartesian_impedance_controller_node.py`
→ 启动 `launch/cartesian_impedance_controller.launch`

#### 🐛 调试问题
→ 运行 `examples/check_kinematics.py` 检查运动学
→ 查看 [tests/README.md](tests/README.md) 常见问题章节

---

## 📊 文件重要性评级

### ⭐⭐⭐ 核心文件（必读）

**文档**：
- `README.md` - 主文档
- `QUICKSTART.md` - 快速开始
- `tests/README.md` - 测试说明

**测试**：
- `tests/force_control_impedance.py` - 力控制测试
- `tests/position_control_only.py` - 位置控制测试

**源码**：
- `src/impedance_control/controller.py` - 主控制器
- `src/impedance_control/robot_kinematics_wrapper.py` - 运动学

### ⭐⭐ 重要文件（推荐阅读）

**文档**：
- `IMPEDANCE_CONTROLLER_README.md` - 控制器原理

**示例**：
- `examples/check_kinematics.py` - 运动学工具

**ROS节点**：
- `scripts/cartesian_impedance_controller_node.py`

### ⭐ 参考文件（按需查看）

**测试**：
- `tests/test_06-08*.py` - 历史测试（参考）

**配置**：
- `config/*.yaml` - 配置文件

**历史**：
- `archived/*` - 历史代码

---

## 🗂️ 文件功能分类

### 按功能分类

#### 控制核心
```
src/impedance_control/
├── controller.py           # 阻抗控制算法
├── robot_kinematics_wrapper.py  # FK/IK/Jacobian
├── state.py               # 状态数据结构
└── strategy.py            # 控制策略接口
```

#### 测试验证
```
tests/
├── force_control_impedance.py   # 完整功能测试
├── position_control_only.py     # 基础功能测试
└── test_06-08*.py              # 单项功能测试
```

#### ROS接口
```
scripts/
├── cartesian_impedance_controller_node.py  # 笛卡尔空间
├── impedance_controller.py                 # 关节空间
└── impedance_feeder_node.py               # 轨迹输入
```

#### 工具示例
```
examples/
├── check_kinematics.py        # 运动学验证
└── force_sensor_filter.py     # 信号处理
```

---

## 📈 开发历史

### 时间线

```
2024-Q1: 基础功能
├── Pinocchio运动学集成
├── MuJoCo仿真环境搭建
└── 基础阻抗控制实现

2024-Q2: 功能完善
├── ROS节点开发
├── 多种控制策略
└── 配置文件系统

2024-Q3-Q4: 测试迭代
├── test_01-05: 模块测试
├── test_06-08: 集成测试
└── 参数调优

2025-Q4 - 2026-Q1: 力控制 ⬅️ 当前
├── 接触力控制实现
├── 弹性接触模型
├── 代码整理重构
└── 文档完善
```

### 代码演进

**Phase 1**: 分散的测试文件（test_01-05）
- 功能：单模块验证
- 问题：文件混乱、难以维护

**Phase 2**: 集成测试（test_mujoco_*）
- 功能：完整流程测试
- 问题：接口不统一

**Phase 3**: 规范化（当前）✅
- 清晰的目录结构
- 统一的接口设计
- 完整的文档系统

---

## 🔄 文件依赖关系

### 核心模块依赖

```
tests/*.py
    │
    ├─→ src/impedance_control/controller.py
    │       │
    │       ├─→ robot_kinematics_wrapper.py
    │       │       └─→ pinocchio
    │       │
    │       └─→ strategy.py
    │               └─→ state.py
    │
    └─→ mujoco (仿真环境)
```

### ROS节点依赖

```
scripts/*_node.py
    │
    ├─→ src/impedance_control/
    ├─→ src/ros/
    └─→ ROS (rospy, messages)
```

---

## 📝 维护指南

### 添加新测试
1. 在 `tests/` 创建文件
2. 参考 `position_control_only.py` 结构
3. 更新 `tests/README.md`

### 添加新功能
1. 在 `src/` 相应模块添加代码
2. 在 `tests/` 添加测试
3. 在 `examples/` 添加示例（可选）
4. 更新相关文档

### 清理历史文件
1. 确认功能已被新版本覆盖
2. 移动到 `archived/`
3. 更新 `archived/README.md`

---

## 📊 代码统计

```
核心源码 (src/):           ~2000行
测试代码 (tests/):         ~1500行
ROS节点 (scripts/):        ~800行
示例工具 (examples/):      ~300行
总计:                      ~4600行
```

---

## 🎓 学习路径建议

### 新手（第1-2天）
1. 阅读 [QUICKSTART.md](QUICKSTART.md)
2. 运行 `position_control_only.py`
3. 理解基础控制流程

### 进阶（第3-5天）
1. 阅读 [IMPEDANCE_CONTROLLER_README.md](IMPEDANCE_CONTROLLER_README.md)
2. 运行 `force_control_impedance.py`
3. 调整参数观察效果

### 高级（第6-10天）
1. 阅读源码 `src/impedance_control/`
2. 修改策略 `strategy.py`
3. 开发自定义控制器

---

更多信息请参考 [主README](README.md) 或 [快速开始](QUICKSTART.md)
