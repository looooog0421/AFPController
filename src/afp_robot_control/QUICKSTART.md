# 快速开始指南

5分钟快速上手AFP机器人控制系统。

## ✅ 前置检查

### 1. 确认环境
```bash
# 激活环境
conda activate AFP

# 检查依赖
python3 -c "import mujoco; import pinocchio; print('✓ 环境正常')"
```

### 2. 确认场景文件存在
```bash
ls /home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene_contact.xml
```

## 🚀 运行第一个测试

### 步骤1: 进入测试目录
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/tests
```

### 步骤2: 运行纯位置控制测试
```bash
python3 position_control_only.py
```

**你应该看到**：
- MuJoCo viewer窗口打开
- 机器人从初始位置移动
- 末端下降接近桌面
- 沿对角线平滑移动

**预期输出**：
```
============================================================
测试9: 纯位置控制 - 接触测试
============================================================

1. 加载模型...
场景中的geom数量: 156

2. 初始化控制器...
重力补偿: 启用

初始末端位置: [0.625 -0.431  0.526]
初始末端Z高度: 0.5265m
接触平面高度: 0.40m

3. 移动到接近位置...
  时间: 0.0s, 位置误差: 0.00mm, 姿态偏差: 0.00°
  时间: 1.0s, 位置误差: 2.34mm, 姿态偏差: 0.12°
  ...
✓ 到达接近位置

最终结果:
  XY误差: 1.23mm
  Z误差: 0.45mm
```

### 步骤3: 运行力控制测试
```bash
python3 force_control_impedance.py
```

**你应该看到**：
- 机器人接近桌面
- 接触时施加-10N力
- 在保持力的同时XY移动

---

## 📊 判断测试是否成功

### ✅ 成功的标志
- [ ] 机器人运动平滑，无抖动
- [ ] 位置误差 < 5mm
- [ ] 姿态偏差 < 2°
- [ ] （力控制）接触力稳定在目标值附近

### ❌ 常见问题

**问题1: IK求解失败**
```
解决: 检查目标位姿是否在工作空间内
```

**问题2: 机器人剧烈震荡**
```
解决: 降低阻抗刚度参数
在force_control_impedance.py中:
position_stiffness = [100, 100, 20]  # 降低刚度
```

**问题3: 找不到场景文件**
```
解决: 修改测试文件中的xml_path
xml_path = "你的实际路径/scene_contact.xml"
```

---

## 📚 下一步

### 理解测试代码
```bash
# 阅读最简单的测试
less tests/position_control_only.py

# 重点关注:
# - 第24行: main()函数
# - 第67行: 初始化控制器
# - 第136行: IK求解
# - 第143行: 重力补偿
```

### 修改参数试验
1. 打开 `tests/position_control_only.py`
2. 找到第60行：`ENABLE_GRAVITY_COMPENSATION = True`
3. 改为 `False`，观察效果差异
4. 对比有无重力补偿的位置误差

### 自定义轨迹
```python
# 在position_control_only.py中找到第89-91行
end_position = start_position.copy()
end_position[0] += 0.10  # 改成你想要的X偏移
end_position[1] += 0.10  # 改成你想要的Y偏移
```

---

## 🎯 常用命令

### 快速测试
```bash
# 位置控制
cd /home/lgx/Project/AFP/src/afp_robot_control/tests
python3 position_control_only.py

# 力控制
python3 force_control_impedance.py
```

### 查看帮助
```bash
# 查看测试说明
cat tests/README.md

# 查看主文档
cat README.md
```

### 检查运动学
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/examples
python3 check_kinematics.py
```

---

## 📖 推荐阅读顺序

1. **本文档** - 快速运行第一个测试 ✓ 你在这里
2. [tests/README.md](tests/README.md) - 详细测试说明
3. [README.md](README.md) - 完整包文档
4. [IMPEDANCE_CONTROLLER_README.md](IMPEDANCE_CONTROLLER_README.md) - 控制器原理

---

## 💡 提示

- **Ctrl+C** 随时中断测试
- **慢速观察**：修改 `time.sleep(model.opt.timestep)` 增加延迟
- **打印调试**：添加 `print()` 输出关键变量
- **参数备份**：找到好参数后记录下来

---

## 🆘 获取帮助

如果遇到问题：

1. 检查错误信息
2. 查看 [tests/README.md](tests/README.md) 的常见问题章节
3. 运行 `examples/check_kinematics.py` 排查运动学问题
4. 确认环境和依赖版本正确

---

**准备好了吗？** 现在就运行你的第一个测试吧！

```bash
cd /home/lgx/Project/AFP/src/afp_robot_control/tests && python3 position_control_only.py
```
