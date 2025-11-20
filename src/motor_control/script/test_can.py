
# sudo ip link set can0 up type can bitrate 1000000
# import can

# def receive_can_messages():
#     bus = can.interface.Bus(channel='can0', bustype='socketcan')
#     while True:
#         message = bus.recv()
#         if message is not None:
#             print(f"Received CAN message: ID={hex(message.arbitration_id)}, Data={message.data.hex()}")

# if __name__ == "__main__":
#     receive_can_messages()


import rospy
import can
import time
from motor_msgs.msg import Motor

def bytes_to_int(high, low):
    value = (high << 8) | low
    if value & 0x8000:
        value -= 0x10000
    return value

def main():
    rospy.init_node('motor_can_receiver', anonymous=True)
    pub = rospy.Publisher('/motor_feedback', Motor, queue_size=50)

    bus = can.interface.Bus(channel='can0', interface='socketcan')
    rospy.loginfo("启动 CAN 电机接收节点...")

    while not rospy.is_shutdown():
        # 非阻塞读取所有当前缓存帧
        msg = bus.recv(timeout=0)  # 立即返回
        while msg and not rospy.is_shutdown():
            if len(msg.data) >= 6:
                data = msg.data
                angle_deg = ((data[0]<<8)|data[1])*0.1
                speed_rpm = bytes_to_int(data[2], data[3])*0.1
                torque_nm = bytes_to_int(data[4], data[5])*0.01

                motor_msg = Motor()
                motor_msg.position = float(angle_deg)
                motor_msg.velocity = float(speed_rpm)
                motor_msg.torque = float(torque_nm)
                motor_msg.kp = 0.0  # 默认PID参数
                motor_msg.kd = 0.0
                motor_msg.ki = 0.0
                
                print("Received CAN ID:", hex(msg.arbitration_id), 
                      "Angle:", angle_deg, 
                      "Speed:", speed_rpm, 
                      "Torque:", torque_nm)
                pub.publish(motor_msg)

            msg = bus.recv(timeout=0)  # 继续读取缓存中下一帧
        
        # 添加小的延时，让CPU有机会处理其他任务和信号
        time.sleep(0.001)  # 1ms

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
