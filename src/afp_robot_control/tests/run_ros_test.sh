#!/bin/bash
# ROS阻抗控制器快速测试脚本

echo "=========================================="
echo "ROS阻抗控制器测试脚本"
echo "=========================================="
echo ""

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS环境未配置！"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo "✓ ROS环境: $ROS_DISTRO"
echo ""

# 检查roscore
if ! rostopic list &> /dev/null; then
    echo "❌ roscore未运行！"
    echo "请在另一个终端运行: roscore"
    exit 1
fi

echo "✓ roscore正在运行"
echo ""

# 选择测试模式
echo "选择测试模式:"
echo "  1) 固定位置保持 (fixed_position)"
echo "  2) 圆形轨迹 (circular)"
echo "  3) XY平面运动 (xy_motion)"
echo "  4) 接近接触 (approach_contact)"
echo ""
read -p "请输入选项 [1-4]: " choice

case $choice in
    1)
        MODE="fixed_position"
        ;;
    2)
        MODE="circular"
        ;;
    3)
        MODE="xy_motion"
        ;;
    4)
        MODE="approach_contact"
        ;;
    *)
        echo "无效选项，使用默认: fixed_position"
        MODE="fixed_position"
        ;;
esac

echo ""
echo "=========================================="
echo "测试模式: $MODE"
echo "=========================================="
echo ""

# 检查阻抗控制器是否运行
if ! rostopic list | grep -q "/reference_trajectory"; then
    echo "⚠️  警告: 阻抗控制器可能未运行"
    echo ""
    read -p "是否先启动阻抗控制器? [y/N]: " start_controller
    if [[ $start_controller == "y" || $start_controller == "Y" ]]; then
        echo "在新终端运行以下命令："
        echo "  roslaunch afp_robot_control cartesian_impedance_controller.launch"
        echo ""
        read -p "启动完成后按Enter继续..."
    fi
fi

echo ""
echo "启动测试节点..."
echo ""

# 运行测试
roslaunch afp_robot_control test_impedance_control.launch test_mode:=$MODE

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
