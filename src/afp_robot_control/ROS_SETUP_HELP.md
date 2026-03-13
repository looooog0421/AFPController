# ⚠️ ROS环境设置重要提醒

## 问题：找不到launch文件

如果遇到错误：
```
RLException: [test_impedance_control.launch] is neither a launch file in package [afp_robot_control]
```

## 解决方法

### 方式1: 运行设置脚本（推荐）
```bash
cd /home/lgx/Project/AFP/src/afp_robot_control
./setup_ros_env.sh
```

### 方式2: 手动设置

**步骤1**: 重新编译包
```bash
cd /home/lgx/Project/AFP
catkin build afp_robot_control
```

**步骤2**: Source环境
```bash
source /home/lgx/Project/AFP/devel/setup.bash
```

**步骤3**: 验证
```bash
rospack find afp_robot_control
# 应该输出: /home/lgx/Project/AFP/src/afp_robot_control

roslaunch --files afp_robot_control test_impedance_control.launch
# 应该输出launch文件路径
```

**步骤4**: 运行测试
```bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position
```

---

## 永久配置

每次打开新终端都需要source环境。为避免重复操作，可以添加到bashrc：

```bash
echo "source /home/lgx/Project/AFP/devel/setup.bash" >> ~/.bashrc
```

**注意**: 如果有多个catkin工作空间，只添加主工作空间的setup.bash

---

## 快速测试流程

```bash
# Terminal 1: 确保环境已source
source /home/lgx/Project/AFP/devel/setup.bash

# 启动roscore（如果未运行）
roscore

# Terminal 2: 启动阻抗控制器
source /home/lgx/Project/AFP/devel/setup.bash
roslaunch afp_robot_control cartesian_impedance_controller.launch

# Terminal 3: 运行测试
source /home/lgx/Project/AFP/devel/setup.bash
roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position
```

---

## 常见问题

### Q: 每次都要source很麻烦？
**A**: 添加到~/.bashrc或创建alias：
```bash
alias afp_setup='source /home/lgx/Project/AFP/devel/setup.bash'
```

### Q: 修改了launch文件需要重新编译吗？
**A**: 
- 修改launch文件内容 → **不需要**重新编译
- 新增launch文件 → **需要**重新编译和source

### Q: 修改了Python脚本需要重新编译吗？
**A**: **不需要**，Python是解释型语言，修改后直接生效

---

## 验证清单

运行前检查：
- [ ] ROS环境已source
- [ ] 可以找到afp_robot_control包（`rospack find afp_robot_control`）
- [ ] 可以找到launch文件（`roslaunch --files afp_robot_control xxx.launch`）
- [ ] roscore正在运行
- [ ] 所有Python脚本有可执行权限

---

更多信息请查看：
- [ROS测试指南](tests/ROS_TEST_GUIDE.md)
- [ROS接口文档](ROS_INTERFACE.md)
- [主README](README.md)
