import rospy
import threading
import numpy as np
import time
from std_msgs.msg import Float32
###
# 调节放卷轮转速，以维持张力恒定，一般来说，放卷论速度增加，张力降低，放卷轮速度降低，张力升高
# 选用pid控制器
###



TENSION_DES = 1.0 # 目标张力值，单位：N


class TensionController:
    """
    张力控制器，读取张力传感器数据，调节放卷轮转速以维持张力恒定
    """
    def __init__(self):
        # PID参数
        self.kp = 0.6
        self.kd = 0.25
        self.ki = 0.0001

        self.tension_sub = rospy.Subscriber('/tension_sensor/data', Float32, self.tension_callback)

        self.tension_cur = 0.0
        self.tension_err_last = 0.0
        self.lock = threading.Lock()
        self.tension_des = TENSION_DES

        self.sub_freq = 50.0  # 控制频率50Hz
    
    def tension_callback(self, msg):
        tension_err = self.tension_des - msg.data
        with self.lock:
            self.tension_cur = msg.data
            tension_err_diff = (tension_err - self.tension_err_last) * self.sub_freq
            self.tension_err_last = tension_err
        # PID计算
        control_signal = (self.kp * tension_err) + (self.kd * tension_err_diff)