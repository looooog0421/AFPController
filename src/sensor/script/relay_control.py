#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================
relay_control.py
作者: hzk
功能: 使用 RS485（Modbus RTU）控制双路继电器开关
节点名: relay2_controller（兼容旧配置）

说明:
    该节点默认订阅 "/relay2/cmd" 主题来控制两个继电器输出。
    收到消息:
        11 → 打开继电器第一路（output1）
        10 → 关闭继电器第一路（output1）
        21 → 打开继电器第二路（output2）
        20 → 关闭继电器第二路（output2）

    编码规则：十位表示输出编号，个位表示状态（1=通，0=断）。
    继电器通讯协议为 Modbus RTU，所有指令均包含 CRC16 校验。
    串口参数、端口号、波特率、订阅话题均可通过 ROS 参数传入。
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
        rospy.loginfo("===== Dual Relay Controller Node Started =====")

        # ------------------------------------------------------------------
        # 1. 读取 ROS 参数（可在 launch 中配置）
        # ------------------------------------------------------------------
        self.port = rospy.get_param("~port", "/dev/ttyACM0")      # 串口设备
        self.baudrate = rospy.get_param("~baudrate", 9600)         # 波特率
        self.timeout = rospy.get_param("~timeout", 0.02)           # 接收超时时间(s)
        self.cmd_topic = rospy.get_param("~cmd_topic", "/relay2/cmd")
        self.relay_address = 0xFE

        # 常见双路继电器地址映射：第一路 0x0000，第二路 0x0001
        self.coil_map = {
            1: 0x0000,
            2: 0x0001,
        }

        self.ser = None

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

        rospy.loginfo("加载完成：双路继电器控制指令生成器")
        rospy.loginfo("支持指令：11/10 控制 output1，21/20 控制 output2")

        # ------------------------------------------------------------------
        # 3. 订阅 ROS Topic
        # ------------------------------------------------------------------
        rospy.Subscriber(self.cmd_topic, Int32, self.command_callback)
        rospy.loginfo(f"订阅 Topic: {self.cmd_topic} (Int32)")

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
    # 解析收到的 ROS 消息
    # ------------------------------------------------------------
    def parse_command(self, value: int):
        output_id = value // 10
        state = value % 10

        if output_id not in self.coil_map or state not in (0, 1):
            return None, None

        return output_id, state

    # ------------------------------------------------------------
    # 生成 Modbus RTU 写单线圈指令
    # ------------------------------------------------------------
    def build_cmd(self, output_id: int, state: int):
        coil_addr = self.coil_map[output_id]
        frame = bytearray([
            self.relay_address,
            0x05,
            (coil_addr >> 8) & 0xFF,
            coil_addr & 0xFF,
            0xFF if state == 1 else 0x00,
            0x00,
        ])

        crc = crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        return frame

    # ------------------------------------------------------------
    # 处理收到的 ROS 消息
    # ------------------------------------------------------------
    def command_callback(self, msg: Int32):
        """
        当收到命令时触发。
        msg.data:
            11 → 打开第一路继电器
            10 → 关闭第一路继电器
            21 → 打开第二路继电器
            20 → 关闭第二路继电器
        """
        output_id, state = self.parse_command(msg.data)

        if output_id is None:
            rospy.logwarn(f"收到无效指令: {msg.data}，请发送 11/10/21/20")
            return

        action = "打开" if state == 1 else "关闭"
        rospy.loginfo(f"收到指令：{action}继电器第{output_id}路")
        cmd = self.build_cmd(output_id, state)
        self.send_cmd(cmd)

    # ------------------------------------------------------------
    # 串口发送 Modbus 指令
    # ------------------------------------------------------------
    def send_cmd(self, cmd: bytearray):
        """
        将生成好的 Modbus RTU 帧通过 RS485 发出去。
        """
        try:
            self.ser.write(cmd)
            self.ser.flush()
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
