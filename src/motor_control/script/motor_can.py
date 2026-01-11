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
        if self.iout > 1000: self.iout = 1000
        if self.iout < -1000: self.iout = -1000
        
        self.dout = self.kd * (0 - self.vel)

        output = self.pout + self.iout + self.dout

        # 总输出限幅
        if output > self.output_max: output = self.output_max
        if output < self.output_min: output = self.output_min
        
        return output

class MotorControl:
    def __init__(self, id):
        self.id = id
        # ==========================================
        # 【关键修改 1】设置减速比
        # M2006 (C610) 通常是 36.0
        # M3508 (C620) 通常是 19.2
        # 你先填 36.0，如果动得太大了就改 19.2
        self.reduction_ratio = 36.0 
        # ==========================================
        mid, h, l = Dec2Hex16(id)
        filters = [{"can_id": int(mid, 16), "can_mask": 0xFFFF, "extended": False}]
        
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, can_filters=filters)
        except Exception as e:
            rospy.logerr(f"CAN Init Error: {e}")
            return

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
        
        # 删除滤波器相关变量

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
        
        # 多圈计数逻辑不变
        if diff < -4000: self.circle_count += 1
        elif diff > 4000: self.circle_count -= 1
        
        self.raw_angle_last = current_raw
        
        # ==========================================
        # 【关键修改 2】引入减速比计算
        # 原始公式: (circles * 8192 + raw) / 8192 * 360
        # 修正公式: 除以 self.reduction_ratio
        # ==========================================
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
        loop_counter = 0
        last_print_time = time.time()
        
        while self.running:
            loop_start_time = time.time()
            
            # 看门狗
            if time.time() - self.last_cmd_time > 0.2:
                self.enable_control = False
            
            output = 0
            if self.enable_control:
                with self.lock:
                    # ===【修正】直接计算，不滤波 output ===
                    output = self.pid_controller.pid_calculate(self.current_target)
                self.send_msg(output)
            else:
                self.send_msg(0)
                output = 0
            
            self.debug_current_cmd = output
            
            # 频率控制
            elapsed = time.time() - loop_start_time
            if target_period > elapsed:
                time.sleep(target_period - elapsed)
            
            # ===【修正】频率打印逻辑 ===
            # 每 1 秒强制打印一次，不依赖 loop count，防止因为循环卡顿而不打印
            if time.time() - last_print_time > 1.0:
                actual_loop_time = time.time() - loop_start_time
                if actual_loop_time > 0:
                    freq = 1.0 / actual_loop_time
                    # 只有频率异常才报警告，否则是 Info
                    if freq < 180:
                        rospy.logwarn(f"[Motor Thread] LOW FREQ: {freq:.1f}Hz")
                    else:
                        rospy.loginfo(f"[Motor Thread] Stable: {freq:.1f}Hz")
                last_print_time = time.time()

    def set_target(self, target, kp, ki, kd):
        with self.lock:
            self.current_target = target
            # 只有当参数变化很大时才 reset，防止频繁清空积分
            if kp != self.pid_controller.kp: 
                self.pid_controller.pid_reset(kp, ki, kd)
            self.enable_control = True
            self.last_cmd_time = time.time()

    def send_msg(self, curr):
        _, hexh, hexl = Dec2Hex16(int(curr))
        data = [0x00] * 8
        if self.id == 513:
            data[0] = int(hexh, 16); data[1] = int(hexl, 16)
        elif self.id == 514:
            data[2] = int(hexh, 16); data[3] = int(hexl, 16)
        try:
            msg = can.Message(arbitration_id=512, is_extended_id=False, data=data)
            self.bus.send(msg)
        except can.CanError:
            pass

    def measure(self):
        with self.lock:
            return self.total_angle, self.last_speed, self.last_torque
            
    def get_debug_info(self):
        return self.debug_current_cmd, self.current_target, self.total_angle

    def stop(self):
        self.running = False
        self.send_msg(0)

# ... [Class ServoMotorLowLevelControl 保持不变] ...
class ServoMotorLowLevelControl:
    def __init__(self):
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        self.motor_status_pub1 = rospy.Publisher('motor1_status', Motor, queue_size=1)
        self.motor_cmd_sub1 = rospy.Subscriber('/motor1_cmd', Motor, self.motor_cmd_cb1)
        self.debug_pub = rospy.Publisher('/afp/debug/current_cmd', Float32, queue_size=1)
        
        self.m1ctrl = MotorControl(513)
        rospy.loginfo("底层驱动已启动 (无滤波器版本)")
        
    def __del__(self):
        self.m1ctrl.stop()

    def motor_cmd_cb1(self, data):
        self.m1ctrl.set_target(data.position, data.kp, data.ki, data.kd)

    def publish_status(self):
        angle, speed, torque = self.m1ctrl.measure()
        calc_current, target, _ = self.m1ctrl.get_debug_info()
        
        motor_msg = Motor()
        motor_msg.position = float(angle)
        motor_msg.velocity = float(speed)
        motor_msg.torque = float(torque)
        self.motor_status_pub1.publish(motor_msg)
        self.debug_pub.publish(Float32(data=calc_current))

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
        motor_interface.m1ctrl.stop()