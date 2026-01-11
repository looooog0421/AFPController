#!/bin/bash
# 这是一个自动化配置硬件接口的脚本
# 用法: ./setup_hardware.sh

echo "🚀 正在初始化硬件接口..."

# ================= 1. 配置 CAN 总线 (电机控制) =================
echo "--------------------------------"
echo "[1/3] 配置 CAN0 (波特率 1M)..."

# 先尝试关闭设备，防止之前状态错误导致无法重新设置
sudo ip link set can0 down 2>/dev/null

# 设置波特率 1000000 并启动接口
sudo ip link set can0 up type can bitrate 1000000

# 检查是否启动成功
if ifconfig can0 | grep -q "UP"; then
    echo "✅ CAN0 启动成功 (Motor Ready)"
else
    echo "❌ CAN0 启动失败! 请检查 USB-CAN 模块是否插好"
fi

# ================= 2. 配置 USB 串口 (张力传感器) =================
echo "--------------------------------"
echo "[2/3] 配置张力传感器 (/dev/ttyUSB*)..."

# 这里的逻辑是：给所有的 ttyUSB 设备权限，防止有时候它变成 ttyUSB1
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    sudo chmod 777 /dev/ttyUSB*
    echo "✅ /dev/ttyUSB* 权限已开启 (Tension Sensor Ready)"
else
    echo "⚠️ 警告: 未检测到 /dev/ttyUSB 设备，张力传感器可能未连接"
fi

# ================= 3. 配置 ACM 串口 (继电器) =================
echo "--------------------------------"
echo "[3/3] 配置继电器 (/dev/ttyACM0)..."

if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0
    echo "✅ /dev/ttyACM0 权限已开启 (Relay Ready)"
else
    # 有时候继电器可能会被识别成 ttyACM1，这里做一个备用检查
    if ls /dev/ttyACM* 1> /dev/null 2>&1; then
        sudo chmod 777 /dev/ttyACM*
        echo "✅ 检测到其他 ACM 设备，权限已开启"
    else
        echo "⚠️ 警告: 未检测到 /dev/ttyACM0，继电器可能未连接"
    fi
fi

echo "--------------------------------"
echo "🎉 所有硬件配置脚本执行完毕！可以运行 ROS 节点了。"
