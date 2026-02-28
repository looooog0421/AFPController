#!/bin/bash
# AFP机器人控制包 - 快速设置脚本

echo "=========================================="
echo "AFP Robot Control - 快速设置"
echo "=========================================="
echo ""

WORKSPACE_PATH="/home/lgx/Project/AFP"

# 检查工作空间
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "❌ 错误: 工作空间不存在: $WORKSPACE_PATH"
    exit 1
fi

cd $WORKSPACE_PATH

# 检查是否需要编译
if [ ! -f "devel/setup.bash" ]; then
    echo "⚠️  工作空间未编译，开始编译..."
    catkin build
    if [ $? -ne 0 ]; then
        echo "❌ 编译失败！"
        exit 1
    fi
fi

# Source环境
echo "✓ 加载ROS环境..."
source devel/setup.bash

# 验证包
echo ""
echo "验证包安装..."
PKG_PATH=$(rospack find afp_robot_control 2>/dev/null)
if [ -z "$PKG_PATH" ]; then
    echo "❌ 找不到afp_robot_control包！"
    echo "   尝试重新编译: cd $WORKSPACE_PATH && catkin build afp_robot_control"
    exit 1
fi

echo "✓ afp_robot_control包路径: $PKG_PATH"

# 检查launch文件
LAUNCH_FILE="$PKG_PATH/launch/test_impedance_control.launch"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✓ 测试launch文件存在"
else
    echo "⚠️  警告: 测试launch文件不存在，重新编译..."
    catkin build afp_robot_control
    source devel/setup.bash
fi

echo ""
echo "=========================================="
echo "✅ 设置完成！"
echo "=========================================="
echo ""
echo "使用方法："
echo ""
echo "1. 在每个新终端运行："
echo "   source $WORKSPACE_PATH/devel/setup.bash"
echo ""
echo "2. 或添加到 ~/.bashrc："
echo "   echo 'source $WORKSPACE_PATH/devel/setup.bash' >> ~/.bashrc"
echo ""
echo "3. 运行测试："
echo "   roslaunch afp_robot_control test_impedance_control.launch test_mode:=fixed_position"
echo ""
echo "=========================================="
