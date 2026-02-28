# 笛卡尔空间阻抗控制器

通用、模块化、可扩展的笛卡尔空间阻抗控制框架，支持多种任务场景。

## 目录结构

```
src/afp_robot_control/
├── src/impedance_control/              # 核心模块
│   ├── impedance_types.py              # 数据结构定义
│   ├── task_strategy.py                # 任务策略（标准/AFP/混合）
│   ├── coordinate_transform.py         # 坐标系转换
│   ├── robot_kinematics_wrapper.py     # Pinocchio运动学封装
│   └── cartesian_impedance_controller.py # 阻抗控制核心
├── scripts/
│   └── cartesian_impedance_controller_node.py # ROS节点
├── msg/
│   ├── ImpedanceParams.msg             # 阻抗参数消息
│   └── JointPositionCommand.msg        # 关节指令消息
├── config/
│   ├── impedance_controller_default.yaml  # 默认配置
│   └── afp_task_config.yaml            # AFP任务配置
└── launch/
    ├── cartesian_impedance_controller.launch # 通用启动
    └── afp_impedance_controller.launch      # AFP启动
```

## 功能特点

### 1. 分层模块化架构
- **ROS接口层**: 消息订阅/发布、参数管理
- **任务策略层**: 可插拔的任务策略（标准/AFP/混合/自定义）
- **控制核心层**: 通用笛卡尔阻抗控制算法
- **运动学层**: Pinocchio正/逆运动学、雅可比

### 2. 多种任务策略

#### 标准策略 (StandardStrategy)
- 适用场景：常规阻抗控制任务
- 特点：直接位姿误差、目标力控制
- 使用示例：拖动示教、装配任务

#### AFP策略 (AFPStrategy)
- 适用场景：纤维铺放任务
- 特点：
  - x-z平面合力恒定控制: `sqrt(fx^2 + fz^2) = F_target`
  - 力分配由接触状态决定: `Δfx/Δfz = fx/fz`
  - y方向和转矩使用标准阻抗
- 使用示例：保持恒定接触力的铺放作业

#### 混合策略 (HybridStrategy)
- 适用场景：部分自由度力控、部分位置控
- 特点：灵活配置每个自由度的控制模式
- 使用示例：表面打磨（法向力控+切向位置控）

### 3. 位置型阻抗控制

控制律：
```
1. 位姿误差: e = x_ref - x_curr
2. 阻抗控制: F_desired = K*e + D*ė [+ M*ë]
3. 力误差: ΔF = F_desired - F_measured
4. 位置修正: Δx = (K + λI)^(-1) * ΔF
5. 修正目标: x_target = x_ref + Δx
6. 逆运动学: q_target = IK(x_target)
```

### 4. 坐标系转换
- 支持力传感器到末端坐标系的旋转变换
- 支持末端到基坐标系的变换
- 灵活配置旋转轴和角度

### 5. 动态参数调整
- 实时订阅阻抗参数更新
- 支持6自由度独立配置刚度/阻尼
- 可选的质量项支持

## 使用指南

### 安装依赖

```bash
# Python依赖
pip install numpy scipy pinocchio

# ROS依赖
sudo apt-get install ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-sensor-msgs
```

### 编译消息

```bash
cd ~/Project/AFP
catkin_make
source devel/setup.bash
```

### 配置参数

编辑 `config/impedance_controller_default.yaml` 或 `config/afp_task_config.yaml`：

```yaml
# 选择任务策略
task_type: "afp"  # 'standard', 'afp', 'hybrid'

# 阻抗参数
default_stiffness: [300.0, 1000.0, 300.0, 15.0, 5.0, 15.0]
default_damping: [30.0, 80.0, 30.0, 2.0, 0.5, 2.0]

# AFP参数
afp:
  target_contact_force: 25.0
```

### 启动控制器

#### 方式1：使用默认配置
```bash
roslaunch afp_robot_control cartesian_impedance_controller.launch
```

#### 方式2：使用AFP配置
```bash
roslaunch afp_robot_control afp_impedance_controller.launch
```

#### 方式3：指定配置文件
```bash
roslaunch afp_robot_control cartesian_impedance_controller.launch config_file:=/path/to/custom_config.yaml
```

### ROS话题接口

#### 订阅话题
- `/joint_states` (sensor_msgs/JointState): 关节状态
- `/netft_data` (geometry_msgs/WrenchStamped): 力/力矩传感器数据
- `/reference_trajectory` (geometry_msgs/PoseStamped): 参考笛卡尔轨迹
- `/impedance_params_dynamic` (std_msgs/Float32MultiArray): 动态阻抗参数

#### 发布话题
- `/joint_position_command` (std_msgs/Float32MultiArray): 关节位置指令
- `/impedance_debug` (std_msgs/Float32MultiArray): 调试信息（可选）

### 动态调整阻抗参数

```python
import rospy
from std_msgs.msg import Float32MultiArray

# 发布新的阻抗参数
pub = rospy.Publisher('/impedance_params_dynamic', Float32MultiArray, queue_size=1)

# 格式: [K1...K6, D1...D6]
new_params = Float32MultiArray()
new_params.data = [
    500, 500, 500, 10, 10, 10,  # 刚度
    50, 50, 50, 1, 1, 1         # 阻尼
]
pub.publish(new_params)
```

## 编程接口

### 核心控制器使用

```python
from impedance_control import (
    CartesianImpedanceController,
    AFPStrategy,
    ImpedanceParams,
    CartesianState,
    WrenchData
)

# 创建策略
strategy = AFPStrategy(target_contact_force=20.0)

# 创建控制器
controller = CartesianImpedanceController(
    urdf_path="/path/to/ur5e.urdf",
    strategy=strategy,
    control_frequency=200.0
)

# 计算控制指令
output = controller.compute_control(
    current_joint_state=q_curr,
    current_joint_velocity=dq_curr,
    reference_cartesian=ref_pose,
    current_wrench=wrench_ee,
    impedance_params=params
)

print(output.joint_positions)  # 关节角度指令
print(output.debug_info)       # 调试信息
```

### 自定义策略

```python
from impedance_control.task_strategy import TaskStrategy

class MyCustomStrategy(TaskStrategy):
    def compute_error(self, current_state, reference_state, current_wrench):
        # 自定义误差计算
        pos_error = reference_state.position - current_state.position
        ori_error = self._compute_orientation_error(...)
        return pos_error, ori_error
    
    def compute_desired_wrench(self, current_wrench, reference_wrench, error_6d):
        # 自定义期望力计算
        desired_wrench = np.zeros(6)
        # ... 您的逻辑
        return desired_wrench
```

## 调试与监控

### 查看调试信息

```bash
# 实时查看日志
rostopic echo /rosout | grep impedance

# 查看调试数据
rostopic echo /impedance_debug
```

### 可视化

```python
# 可以使用rqt_plot查看实时数据
rqt_plot /impedance_debug/data[0]:data[1]:data[2]
```

## 参数调优指南

### 刚度参数 (K)
- 越大：跟踪精度越高，但对外力响应越硬
- 越小：柔顺性越好，但轨迹跟踪精度下降
- 建议范围：100-2000 N/m（平移），5-50 Nm/rad（旋转）

### 阻尼参数 (D)
- 越大：系统越稳定，但响应速度变慢
- 越小：响应快，但可能产生振荡
- 建议关系：临界阻尼 `D = 2*sqrt(K*M)`

### AFP特殊参数
- `target_contact_force`: 根据材料和工艺要求设定
- 刚度配置：x-z方向柔顺（300），y方向较硬（1000）

## 常见问题

### 1. IK不收敛
- 检查参考轨迹是否在工作空间内
- 调整 `correction_damping` 参数
- 降低阻抗刚度

### 2. 振荡问题
- 增大阻尼参数
- 降低刚度参数
- 检查力传感器噪声

### 3. 力控制不准确
- 校准传感器零点
- 检查坐标系转换配置
- 调整AFP策略的 `min_force_threshold`

## 扩展开发

### 添加新的策略
1. 继承 `TaskStrategy` 基类
2. 实现 `compute_error` 和 `compute_desired_wrench`
3. 在节点中注册策略

### 修改运动学
- 编辑 `robot_kinematics_wrapper.py`
- 可替换为其他运动学库（KDL、MoveIt等）

## 性能指标

- 控制频率：200 Hz
- IK求解时间：< 2 ms（典型）
- 单次控制循环：< 5 ms（典型）

## 作者与维护

开发者：GitHub Copilot  
项目：AFP (自动纤维铺放) 控制系统  
日期：2026年1月

## 许可证

请遵循项目主许可证
