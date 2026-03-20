#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import can.logger
import rospy
import time
import threading
from std_msgs.msg import Float32
from motor_msgs.msg import Motor

# ... [辅助函数 GetS16, Dec2Hex16 保持不变] ...
def GetS16(val):
    return val if val < 0x8000 else val - 0x10000

def Dec2Hex16(val):
    OFFSET = 1 << 16
    MASK = OFFSET - 1
    hexs = '%04x' % (val + OFFSET & MASK)
    return hexs, '0x' + hexs[0:2], '0x' + hexs[2:4]

class PIDcontroller:
    def __init__(self, kp, ki, kd, output_limits=(-10, 10)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = 0
        self.pos = 0
        self.vel = 0
        self.pout = 0
        self.iout = 0
        self.dout = 0
        # 安全电流限制
        self.output_max = 3000.0
        self.output_min = -3000.0

    def pid_reset(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.iout = 0 # 重置时清空积分

    def pid_update(self, pos, vel):
        self.pos = pos
        self.vel = vel

    def pid_calculate(self, target):
        self.target = target
        err = self.target - self.pos

        # ===【新增安全保护】===
        # 如果误差超过 180 度，说明失控了，直接返回 0 并报警
        if abs(err) > 900.0:
            rospy.logerr_throttle(1, f"EMERGENCY: Error too large ({err:.1f}), shutting down output!")
            return 0.0
        # ======================

        self.pout = self.kp * err

        # 积分分离
        if abs(err) < 30.0:
            self.iout += self.ki * err * 0.005
        else:
            self.iout = 0

        # 积分限幅
        if self.iout > 1000:
            self.iout = 1000
        if self.iout < -1000:
            self.iout = -1000

        self.dout = self.kd * (0 - self.vel)

        output = self.pout + self.iout + self.dout

        # 总输出限幅
        if output > self.output_max:
            output = self.output_max
        if output < self.output_min:
            output = self.output_min

        return output

class CanCurrentMux0x200:
    def __init__(self, channel='can0', tx_rate_hz=200.0, arbitration_id=512):
        self.channel = channel
        self.tx_period = 1.0 / tx_rate_hz if tx_rate_hz > 0 else 0.005
        self.arbitration_id = arbitration_id
        self.lock = threading.Lock()
        self.currents = {513: 0, 514: 0}
        self.running = True

        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype='socketcan',
                bitrate=1000000,
            )
        except Exception as e:
            rospy.logerr(f"CAN TX Init Error: {e}")
            raise

        self.tx_thread = threading.Thread(target=self._tx_loop)
        self.tx_thread.daemon = True
        self.tx_thread.start()

    def register_motor(self, motor_id):
        with self.lock:
            self.currents.setdefault(motor_id, 0)

    def set_current(self, motor_id, curr):
        with self.lock:
            self.currents[motor_id] = int(curr)

    def _build_data(self):
        data = [0x00] * 8
        for motor_id, slot in ((513, 0), (514, 2)):
            curr = self.currents.get(motor_id, 0)
            _, hexh, hexl = Dec2Hex16(curr)
            data[slot] = int(hexh, 16)
            data[slot + 1] = int(hexl, 16)
        return data

    def _send_once(self):
        with self.lock:
            data = self._build_data()
        try:
            msg = can.Message(arbitration_id=self.arbitration_id, is_extended_id=False, data=data)
            self.bus.send(msg)
        except can.CanError:
            pass

    def _tx_loop(self):
        while self.running:
            loop_start_time = time.time()
            self._send_once()
            elapsed = time.time() - loop_start_time
            if self.tx_period > elapsed:
                time.sleep(self.tx_period - elapsed)

    def stop(self):
        self.running = False
        with self.lock:
            for motor_id in self.currents:
                self.currents[motor_id] = 0
        self._send_once()

class MotorControl:
    def __init__(self, id, mux, channel='can0', reduction_ratio=36.0):
        self.id = id
        self.mux = mux
        # ==========================================
        # 【关键修改 1】设置减速比
        # M2006 (C610) 通常是 36.0
        # M3508 (C620) 通常是 19.2
        # 你先填 36.0，如果动得太大了就改 19.2
        self.reduction_ratio = reduction_ratio
        # ==========================================
        mid, _, _ = Dec2Hex16(id)
        filters = [{"can_id": int(mid, 16), "can_mask": 0xFFFF, "extended": False}]

        try:
            self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=1000000, can_filters=filters)
        except Exception as e:
            rospy.logerr(f"CAN Init Error: {e}")
            raise

        self.mux.register_motor(self.id)
        self.pid_controller = PIDcontroller(0, 0, 0)
        self.lock = threading.Lock()

        self.raw_angle_last = -1
        self.circle_count = 0
        self.total_angle = 0.0
        self.last_speed = 0
        self.last_torque = 0

        self.debug_current_cmd = 0.0
        self.current_target = 0
        self.enable_control = False
        self.last_cmd_time = time.time()
        self.running = True

        self.recv_thread = threading.Thread(target=self._can_receiver_thread)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.ctrl_thread = threading.Thread(target=self._control_loop_thread)
        self.ctrl_thread.daemon = True
        self.ctrl_thread.start()

    def _check_multiturn(self, current_raw):
        if self.raw_angle_last == -1:
            self.raw_angle_last = current_raw
            return 0
        diff = current_raw - self.raw_angle_last

        if diff < -4000:
            self.circle_count += 1
        elif diff > 4000:
            self.circle_count -= 1

        self.raw_angle_last = current_raw
        total_rotor_angle = (self.circle_count * 8192.0 + current_raw) / 8192.0 * 360.0
        return total_rotor_angle / self.reduction_ratio

    def _can_receiver_thread(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg and len(msg.data) >= 6:
                    raw_angle = msg.data[0] * 256 + msg.data[1]
                    speed = GetS16(msg.data[2] * 256 + msg.data[3])
                    torque = GetS16(msg.data[4] * 256 + msg.data[5])

                    with self.lock:
                        self.total_angle = self._check_multiturn(raw_angle)
                        self.last_speed = speed
                        self.last_torque = torque
                        self.pid_controller.pid_update(self.total_angle, speed)
            except Exception:
                pass

    def _control_loop_thread(self):
        target_period = 0.005 # 200Hz
        last_print_time = time.time()

        while self.running:
            loop_start_time = time.time()

            if time.time() - self.last_cmd_time > 0.2:
                self.enable_control = False

            output = 0
            if self.enable_control:
                with self.lock:
                    output = self.pid_controller.pid_calculate(self.current_target)
                self.send_msg(output)
            else:
                self.send_msg(0)

            self.debug_current_cmd = output

            elapsed = time.time() - loop_start_time
            if target_period > elapsed:
                time.sleep(target_period - elapsed)

            if time.time() - last_print_time > 1.0:
                actual_loop_time = time.time() - loop_start_time
                if actual_loop_time > 0:
                    freq = 1.0 / actual_loop_time
                    if freq < 180:
                        rospy.logwarn(f"[Motor Thread {self.id}] LOW FREQ: {freq:.1f}Hz")
                    else:
                        rospy.loginfo(f"[Motor Thread {self.id}] Stable: {freq:.1f}Hz")
                last_print_time = time.time()

    def set_target(self, target, kp, ki, kd):
        with self.lock:
            self.current_target = target
            if kp != self.pid_controller.kp or ki != self.pid_controller.ki or kd != self.pid_controller.kd:
                self.pid_controller.pid_reset(kp, ki, kd)
            self.enable_control = True
            self.last_cmd_time = time.time()

    def send_msg(self, curr):
        self.mux.set_current(self.id, curr)

    def measure(self):
        with self.lock:
            return self.total_angle, self.last_speed, self.last_torque

    def get_debug_info(self):
        return self.debug_current_cmd, self.current_target, self.total_angle

    def stop(self):
        self.running = False
        self.send_msg(0)

class ServoMotorLowLevelControl:
    def __init__(self):
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        can_channel = rospy.get_param('~can_channel', 'can0')
        tx_rate_hz = rospy.get_param('~tx_rate_hz', 200.0)
        motor1_id = rospy.get_param('~motor1_id', 513)
        motor2_id = rospy.get_param('~motor2_id', 514)
        motor1_ratio = rospy.get_param('~motor1_reduction_ratio', 36.0)
        motor2_ratio = rospy.get_param('~motor2_reduction_ratio', 36.0)

        self.motor_status_pub1 = rospy.Publisher('motor1_status', Motor, queue_size=1)
        self.motor_cmd_sub1 = rospy.Subscriber('/motor1_cmd', Motor, self.motor_cmd_cb1)
        self.motor_status_pub2 = rospy.Publisher('motor2_status', Motor, queue_size=1)
        self.motor_cmd_sub2 = rospy.Subscriber('/motor2_cmd', Motor, self.motor_cmd_cb2)
        self.debug_pub = rospy.Publisher('/afp/debug/current_cmd', Float32, queue_size=1)

        self.mux = CanCurrentMux0x200(channel=can_channel, tx_rate_hz=tx_rate_hz)
        self.m1ctrl = MotorControl(motor1_id, self.mux, channel=can_channel, reduction_ratio=motor1_ratio)
        self.m2ctrl = MotorControl(motor2_id, self.mux, channel=can_channel, reduction_ratio=motor2_ratio)
        rospy.loginfo("底层驱动已启动 (双电机共享0x200发送版本)")

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        self.m1ctrl.stop()
        self.m2ctrl.stop()
        self.mux.stop()

    def motor_cmd_cb1(self, data):
        self.m1ctrl.set_target(data.position, data.kp, data.ki, data.kd)

    def motor_cmd_cb2(self, data):
        self.m2ctrl.set_target(data.position, data.kp, data.ki, data.kd)

    def publish_status(self):
        angle1, speed1, torque1 = self.m1ctrl.measure()
        calc_current, _, _ = self.m1ctrl.get_debug_info()

        motor_msg1 = Motor()
        motor_msg1.position = float(angle1)
        motor_msg1.velocity = float(speed1)
        motor_msg1.torque = float(torque1)
        self.motor_status_pub1.publish(motor_msg1)
        self.debug_pub.publish(Float32(data=calc_current))

        angle2, speed2, torque2 = self.m2ctrl.measure()
        motor_msg2 = Motor()
        motor_msg2.position = float(angle2)
        motor_msg2.velocity = float(speed2)
        motor_msg2.torque = float(torque2)
        self.motor_status_pub2.publish(motor_msg2)

if __name__ == "__main__":
    motor_interface = ServoMotorLowLevelControl()
    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            motor_interface.publish_status()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        motor_interface.shutdown()
