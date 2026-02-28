#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import rospy
import threading
from std_msgs.msg import String, Float32
import struct



def crc16(data: bytes):
    """
    Modbus RTU CRC16校验
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 0x0001):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


class TensionSensor:
    def __init__(self):
        rospy.init_node('tension_sensor_node', anonymous=True)

        # 参数
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 9600)
        timeout = rospy.get_param('~timeout', 0.02)
        self.read_interval = rospy.get_param('~read_interval', 0.02)


        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=timeout
            )
            rospy.loginfo(f"Opened serial port: {port} at {baudrate} baud.")
        except Exception as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            rospy.signal_shutdown("Serial port error")
            return
        
        # Modbus RTU 读取指令
        self.read_cmd = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A])  # 读取地址0x0000的1个寄存器

        # 发布&订阅
        self.pub = rospy.Publisher('/tension_sensor/data', Float32, queue_size=10)
        # self.sub = rospy.Subscriber('/tension_sensor/command', String, self.command_callback)

        self.lock = threading.Lock()
        self.running = True

        self.recv_thread = threading.Thread(target=self.receive_data)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.send_thread = threading.Thread(target=self.send_commands)
        self.send_thread.daemon = True
        self.send_thread.start()

        rospy.on_shutdown(self.cleanup)
        rospy.spin()

    def cleanup(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        rospy.loginfo("Serial port closed.")

    def send_commands(self):
        """
        周期发送读取命令
        """
        while self.running and not rospy.is_shutdown():
            with self.lock:
                self.ser.write(self.read_cmd)
                # rospy.loginfo(f"Sent: {self.read_cmd.hex()}")
            rospy.sleep(self.read_interval)


    def receive_data(self):
        buf = bytearray()
        while not rospy.is_shutdown() and self.running:
            if self.ser.in_waiting:
                buf = self.ser.read_all()
                # rospy.loginfo(f"Raw data: {buf.hex()}")
                
                # 检查是否收到完整的数据包
                if len(buf) >= 7:
                    # 假设数据包格式为: [地址][功能码][数据长度][数据][CRC低][CRC高]

                    # 帧头
                    if buf[0] != 0x01 or buf[1] != 0x03:
                        buf.pop(0)
                        continue

                    # 校验CRC
                    frame = buf[:7]
                    buf = buf[7:]
                    addr, func, length, data_h, data_l, crc_l, crc_h = struct.unpack('>BBBBBBB', frame)
                    crc_recv = (crc_h << 8) | crc_l
                    crc_calc = crc16(frame[:-2])

                    if crc_recv != crc_calc:
                        rospy.logwarn("CRC check failed")
                        continue


                    raw_value = (data_h << 8) | data_l
                    tension_value = raw_value * 0.01 # 两位小数
                        
                    self.pub.publish(tension_value)
                else:
                    rospy.loginfo(f"buf length less than 7, is {len(buf)}")
                
            
            rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        sensor = TensionSensor()
    except rospy.ROSInterruptException:
        pass