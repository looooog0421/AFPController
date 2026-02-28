#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================
relay_control.py
作者: hzk
功能: 使用 RS485（Modbus RTU）控制继电器第二路开关
节点名: relay2_controller

说明:
    该节点通过订阅 "/relay2/cmd" 主题来控制继电器的开关。
    收到消息:
        1 → 打开继电器第二路
        0 → 关闭继电器第二路

    继电器通讯协议为 Modbus RTU，所有指令均包含 CRC16 校验。
    串口参数、端口号、波特率均可通过 ROS 参数传入。
========================================================
"""

import rospy
import serial
from std_msgs.msg import Int32


# ------------------------------------------------------------
#  CRC16(Modbus RTU) 计算函数
# ------------------------------------------------------------
def crc16(data: bytes):
    """
    功能：
        根据 Modbus RTU 协议计算 CRC16 校验码（低位在前，高位在后）。

    输入：
        data: 待校验的字节序列 (bytes)

    输出：
        crc: 16位 CRC 校验值 (int)
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


# ------------------------------------------------------------
#  继电器控制类
# ------------------------------------------------------------
class RelayController:
    """
    RelayController 类用于管理 RS485 串口与 ROS 订阅逻辑。
    """

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("relay2_controller", anonymous=True)
        rospy.loginfo("===== Relay2 Controller Node Started =====")

        # ------------------------------------------------------------------
        # 1. 读取 ROS 参数（可在 launch 中配置）
        # ------------------------------------------------------------------
        self.port = rospy.get_param("~port", "/dev/ttyACM0")     # 串口设备
        self.baudrate = rospy.get_param("~baudrate", 9600)       # 波特率
        self.timeout = rospy.get_param("~timeout", 0.02)         # 接收超时时间(s)

        # ------------------------------------------------------------------
        # 2. 打开串口
        # ------------------------------------------------------------------
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            rospy.loginfo(f"[OK] 串口打开成功: {self.port}, 波特率: {self.baudrate}")
        except Exception as e:
            rospy.logerr(f"[ERROR] 串口打开失败: {e}")
            rospy.signal_shutdown("Serial port error")
            return

        # ------------------------------------------------------------------
        # 3. 准备 Modbus 指令（来自设备说明书）
        # ------------------------------------------------------------------

        # FE（地址） 05（功能码：写单线圈） 00 01（第二路寄存器地址）
        # FF 00（写1）C9 F5（CRC）
        self.cmd_open = bytearray([0xFE, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xC9, 0xF5])

        # FE 05 00 01 00 00 88 05
        self.cmd_close = bytearray([0xFE, 0x05, 0x00, 0x01, 0x00, 0x00, 0x88, 0x05])

        rospy.loginfo("加载完成：继电器第二路开/关指令")

        # ------------------------------------------------------------------
        # 4. 订阅 ROS Topic
        # ------------------------------------------------------------------
        rospy.Subscriber("/relay2/cmd", Int32, self.command_callback)
        rospy.loginfo("订阅 Topic: /relay2/cmd (Int32)")

        # 清理动作
        rospy.on_shutdown(self.cleanup)

        # 进入循环（阻塞）
        rospy.spin()

    # ------------------------------------------------------------
    # 清理：关闭串口
    # ------------------------------------------------------------
    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("串口已关闭，节点退出。")

    # ------------------------------------------------------------
    # 处理收到的 ROS 消息
    # ------------------------------------------------------------
    def command_callback(self, msg: Int32):
        """
        当收到 /relay2/cmd 的消息时触发
        msg.data:
            1 → 打开第二路继电器
            0 → 关闭第二路继电器
        """
        if msg.data == 1:
            rospy.loginfo("收到指令：打开继电器第二路")
            self.send_cmd(self.cmd_open)

        elif msg.data == 0:
            rospy.loginfo("收到指令：关闭继电器第二路")
            self.send_cmd(self.cmd_close)

        else:
            rospy.logwarn("收到无效指令！请发送 0 或 1")

    # ------------------------------------------------------------
    # 串口发送 Modbus 指令
    # ------------------------------------------------------------
    def send_cmd(self, cmd: bytearray):
        """
        将生成好的 Modbus RTU 帧通过 RS485 发出去。
        """
        try:
            self.ser.write(cmd)
            rospy.loginfo(f"已发送指令: {cmd.hex(' ').upper()}")
        except Exception as e:
            rospy.logerr(f"[ERROR] 指令发送失败: {e}")


# ------------------------------------------------------------
#  主入口
# ------------------------------------------------------------
if __name__ == "__main__":
    try:
        RelayController()
    except rospy.ROSInterruptException:
        pass
