#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""========================================================
relay_control.py
作者: hzk

现状：
    旧版本通过 topic `/relay2/cmd` 的 Int32 魔法数（11/10/21/20）控制继电器，
    不利于别人接手与后续工程扩展。

当前版本（方案 B / 分层架构）：
    本文件作为“继电器驱动节点入口”，唯一占用串口，提供标准 service API：
        ~output1/set (std_srvs/SetBool)
        ~output2/set (std_srvs/SetBool)

    语义化控制（剪刀/压紧）由上层节点完成：
        /scissor/enable (std_msgs/Bool) -> 调用 /relay_driver/output1/set
        /clamp/enable   (std_msgs/Bool) -> 调用 /relay_driver/output2/set

可选兼容：
    若设置参数 `~enable_legacy_cmd:=true`，则仍可订阅 legacy topic（默认 /relay2/cmd）
    并解析 11/10/21/20。

串口参数（ROS private params）：
    ~port               默认 /dev/ttyACM0
    ~baudrate           默认 9600
    ~timeout            默认 0.02
    ~relay_address      默认 0xFE
    ~output1_coil_addr  默认 0x0000
    ~output2_coil_addr  默认 0x0001
========================================================"""

import rospy
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolResponse

from sensor.relay_control import RelaySerialDriver


class RelayDriverNode:
    def __init__(self):
        rospy.init_node("relay_driver", anonymous=False)
        rospy.loginfo("===== Relay Driver Node Started =====")

        # 串口与继电器参数
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baudrate = int(rospy.get_param("~baudrate", 9600))
        timeout = float(rospy.get_param("~timeout", 0.02))
        relay_address = int(rospy.get_param("~relay_address", 0xFE))

        output1_coil_addr = int(rospy.get_param("~output1_coil_addr", 0x0000))
        output2_coil_addr = int(rospy.get_param("~output2_coil_addr", 0x0001))

        coil_map = {
            1: output1_coil_addr,
            2: output2_coil_addr,
        }

        try:
            self.driver = RelaySerialDriver(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                relay_address=relay_address,
                coil_map=coil_map,
            )
            rospy.loginfo(f"[OK] 串口打开成功: {port}, 波特率: {baudrate}")
        except Exception as e:
            rospy.logerr(f"[ERROR] 串口打开失败: {e}")
            rospy.signal_shutdown("Serial port error")
            return

        rospy.on_shutdown(self.cleanup)

        # 驱动层 service API（使用 private name，保证在 /<node_name>/... 下）
        self.srv_output1 = rospy.Service("~output1/set", SetBool, self._handle_output1)
        self.srv_output2 = rospy.Service("~output2/set", SetBool, self._handle_output2)

        rospy.loginfo(
            "Services: %s, %s",
            f"{rospy.get_name()}/output1/set",
            f"{rospy.get_name()}/output2/set",
        )

        # legacy topic 兼容（可选）
        self.enable_legacy_cmd = bool(rospy.get_param("~enable_legacy_cmd", False))
        if self.enable_legacy_cmd:
            legacy_cmd_topic = rospy.get_param("~legacy_cmd_topic", "/relay2/cmd")
            rospy.Subscriber(legacy_cmd_topic, Int32, self._legacy_cmd_callback, queue_size=10)
            rospy.logwarn(
                "Legacy Int32 cmd enabled on %s (11/10/21/20). "
                "Deprecated: please migrate to /scissor/enable and /clamp/enable.",
                legacy_cmd_topic,
            )
        else:
            rospy.loginfo(
                "Legacy Int32 cmd disabled. Set ~enable_legacy_cmd:=true to enable /relay2/cmd parsing."
            )

        rospy.spin()

    def cleanup(self):
        try:
            self.driver.close()
        except Exception:
            pass
        rospy.loginfo("串口已关闭，节点退出。")

    def _set_output(self, output_id: int, state: bool):
        try:
            cmd = self.driver.set_output(output_id, state)
            action = "打开" if state else "关闭"
            rospy.loginfo(
                "继电器输出%d: %s, 已发送: %s",
                output_id,
                action,
                cmd.hex(" ").upper(),
            )
            return True, "OK"
        except Exception as e:
            rospy.logerr("设置继电器输出%d失败: %s", output_id, e)
            return False, str(e)

    def _handle_output1(self, req):
        ok, msg = self._set_output(1, bool(req.data))
        return SetBoolResponse(success=ok, message=msg)

    def _handle_output2(self, req):
        ok, msg = self._set_output(2, bool(req.data))
        return SetBoolResponse(success=ok, message=msg)

    @staticmethod
    def _parse_legacy_cmd(value: int):
        output_id = value // 10
        state = value % 10
        if output_id not in (1, 2) or state not in (0, 1):
            return None
        return output_id, bool(state)

    def _legacy_cmd_callback(self, msg: Int32):
        parsed = self._parse_legacy_cmd(msg.data)
        if parsed is None:
            rospy.logwarn("收到无效 legacy 指令: %s（请发送 11/10/21/20）", msg.data)
            return

        rospy.logwarn_throttle(
            5.0,
            "Legacy /relay2/cmd is deprecated. Use /scissor/enable and /clamp/enable.",
        )

        output_id, state = parsed
        self._set_output(output_id, state)


if __name__ == "__main__":
    try:
        RelayDriverNode()
    except rospy.ROSInterruptException:
        pass
