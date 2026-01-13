# 示例代码

本目录包含AFP机器人控制系统的实用工具和示例代码。

## 📂 文件说明

### check_kinematics.py
**功能**：运动学验证工具

**用途**：
- 验证正运动学计算
- 验证逆运动学求解
- 测试雅可比矩阵计算
- 检查工作空间范围

**使用方法**：
```bash
python3 check_kinematics.py
```

**示例输出**：
```
正运动学测试:
  输入关节角: [0, -π/2, π/2, 0, π/2, 0]
  末端位置: [0.817, 0.191, 0.406]
  末端姿态 (四元数): [1.0, 0.0, 0.0, 0.0]

逆运动学测试:
  目标位置: [0.5, 0.3, 0.4]
  求解成功: True
  关节角解: [-0.785, -1.571, ...]
  位置误差: 0.23mm
```

**适用场景**：
- 调试运动学问题
- 验证IK参数设置
- 检查奇异位形
- 工作空间分析

---

### force_sensor_filter.py
**功能**：力传感器信号滤波示例

**特点**：
- 低通滤波器实现
- 滑动窗口平均
- 中值滤波
- 卡尔曼滤波示例

**使用方法**：
```python
from force_sensor_filter import LowPassFilter, MovingAverage

# 低通滤波器
lpf = LowPassFilter(cutoff_freq=10.0, sampling_freq=500.0)
filtered_force = lpf.update(raw_force)

# 滑动窗口平均
ma = MovingAverage(window_size=10)
filtered_force = ma.update(raw_force)
```

**示例代码**：
```python
import numpy as np
from force_sensor_filter import LowPassFilter

# 创建滤波器
filter = LowPassFilter(cutoff_freq=10.0, sampling_freq=500.0)

# 模拟力传感器数据（带噪声）
dt = 1.0 / 500.0
t = np.arange(0, 5, dt)
true_force = -10.0 * np.ones_like(t)
noise = np.random.normal(0, 0.5, len(t))
raw_force = true_force + noise

# 滤波
filtered = []
for f in raw_force:
    filtered.append(filter.update(f))

# 结果
print(f"原始力均值: {np.mean(raw_force):.2f}N")
print(f"滤波后均值: {np.mean(filtered):.2f}N")
print(f"噪声降低: {np.std(raw_force) - np.std(filtered):.2f}N")
```

**适用场景**：
- 力控制中减少传感器噪声
- 接触检测前的信号预处理
- 实时滤波需求
- 比较不同滤波算法性能

---

## 🚀 使用建议

### 开发工作流

1. **开始新功能前**：
   - 使用 `check_kinematics.py` 验证运动学正确
   - 确认目标位姿在工作空间内
   - 测试IK能否收敛

2. **集成力传感器**：
   - 参考 `force_sensor_filter.py` 实现滤波
   - 根据采样频率调整滤波器参数
   - 测试延迟是否可接受

3. **调试运动学问题**：
   - 运行 `check_kinematics.py`
   - 检查FK/IK一致性
   - 验证雅可比计算

---

## 📖 扩展示例

### 添加新的工具

如果你开发了通用工具函数，可以添加到本目录：

```python
# my_tool.py
"""
简短的功能描述
"""

def my_function():
    """详细的函数说明"""
    pass

if __name__ == '__main__':
    # 添加示例用法
    print("示例运行")
```

### 推荐的工具类型

- 数据可视化工具
- 参数调优脚本
- 性能测试工具
- 数据记录/回放
- 标定程序

---

## 🔗 相关文档

- [测试程序](../tests/README.md) - 完整功能测试
- [主README](../README.md) - 包的整体说明
- [阻抗控制器文档](../IMPEDANCE_CONTROLLER_README.md) - 控制器详细说明

---

## 💡 提示

- 这些是**示例代码**，根据需要修改使用
- 不需要保证与测试程序相同的接口
- 可以添加更多注释和说明
- 鼓励实验性功能

---

更多信息请参考 [主README](../README.md)
