#!/bin/bash
# MuJoCo仿真集成测试快速启动脚本

echo "=================================================="
echo "MuJoCo仿真集成测试"
echo "=================================================="
echo ""

# 设置ROS环境
source /home/lgx/Project/AFP/devel/setup.bash

# 检查必要的包是否存在
echo "检查ROS包..."
if ! rospack find afp_robot_control > /dev/null 2>&1; then
    echo "错误: 找不到afp_robot_control包"
    exit 1
fi

if ! rospack find afp_mjc > /dev/null 2>&1; then
    echo "错误: 找不到afp_mjc包"
    exit 1
fi

echo "✓ 所有必要的包都已就绪"
echo ""

# 提示用户选择测试模式
echo "请选择测试模式:"
echo "  1) circular    - 圆形轨迹测试（默认）"
echo "  2) fixed_position - 固定位置测试"
echo "  3) xy_motion   - XY平面运动测试"
echo "  4) approach_contact - 接触测试"
echo ""
read -p "输入选项 (1-4，默认1): " choice

case $choice in
    2)
        TEST_MODE="fixed_position"
        echo "启动固定位置测试..."
        read -p "目标X坐标 (默认0.5): " target_x
        read -p "目标Y坐标 (默认0.0): " target_y
        read -p "目标Z坐标 (默认0.4): " target_z
        target_x=${target_x:-0.5}
        target_y=${target_y:-0.0}
        target_z=${target_z:-0.4}
        EXTRA_ARGS="target_x:=$target_x target_y:=$target_y target_z:=$target_z"
        ;;
    3)
        TEST_MODE="xy_motion"
        echo "启动XY平面运动测试..."
        ;;
    4)
        TEST_MODE="approach_contact"
        echo "启动接触测试..."
        ;;
    *)
        TEST_MODE="circular"
        echo "启动圆形轨迹测试..."
        ;;
esac

echo ""
echo "=================================================="
echo "正在启动测试..."
echo "测试模式: $TEST_MODE"
echo "=================================================="
echo ""
echo "提示: "
echo "  - MuJoCo窗口将会打开显示机器人运动"
echo "  - 按Ctrl+C停止测试"
echo "  - 查看话题: rostopic list"
echo "  - 监控状态: rostopic echo /joint_states"
echo ""

# 启动launch文件
roslaunch afp_robot_control test_with_mujoco_sim.launch test_mode:=$TEST_MODE $EXTRA_ARGS
