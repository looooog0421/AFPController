import rospy
import threading
import numpy as np
import time
from std_msgs.msg import Float32
import copy
###
# 张力控制器
# 调节放卷轮转速，以维持张力恒定，一般来说，放卷轮速度增加，张力降低，放卷轮速度降低，张力升高
# 放卷轮的参考转速是机械臂末端的移动速度,按道理来说如果放卷轮速度与机械臂末端速度相等，张力应该是恒定的
# 在此基础上进行PID控制调整
# 即：v_lun = v_end + delta_v
# 张力控制器就是计算这个delta_v
###



TENSION_DES = 1.0 # 目标张力值，单位：N


class TensionController:
    """
    张力控制器，读取张力传感器数据，调节放卷轮转速以维持张力恒定
    """
    def __init__(self, kp=0.6, kd=0.25, ki=0.0001, sub_freq=100.0):
        # PID参数
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.tension_sub = rospy.Subscriber('/tension_sensor/data', Float32, self.tension_callback)

        self.tension_cur = None
        self.tension_last = None
        self.lock = threading.Lock()
        self.tension_des = TENSION_DES

        self.sub_freq = sub_freq  # 控制频率
    
    def tension_callback(self, msg):
        """
        张力传感器回调函数, 更新当前张力值和上一时刻张力值
        """
        tension_err = self.tension_des - msg.data
        with self.lock:
            self.tension_cur = copy.deepcopy(msg.data)
            self.tension_last = self.tension_cur

    def delta_x(self):
        """
        根据张力误差计算放卷轮速度调整量delta_v，再转化为位置调整量delta_x
        """
        with self.lock:
            tension_err = self.tension_des - self.tension_cur
            tension_err_diff = (self.tension_cur - self.tension_last) * self.sub_freq
            delta_v = (self.kp * tension_err) + (self.kd * tension_err_diff)
        # 位置调整量 = 速度调整量 / 控制频率
        delta_x = delta_v / self.sub_freq
        return delta_x

if __name__ == '__main__':
    rospy.init_node('tension_controller_node')
    tension_controller = TensionController()
    rate = rospy.Rate(100)  # 100Hz
    while not rospy.is_shutdown():
        delta_x = tension_controller.delta_x()
        rospy.loginfo(f"Calculated delta_x for tension control: {delta_x}")
        rate.sleep()