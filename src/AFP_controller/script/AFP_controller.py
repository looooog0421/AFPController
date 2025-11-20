import rospy
import rospy
import threading
import numpy as np
import time
from std_msgs.msg import Float32
import copy

import ur5e_controller
from ur5e_controller import UR5eController
from tension_control import TensionController

"""
AFP控制器
应该包含路径规划
       |
       v
机械臂末端移动初始值
       |
       v
六维力传感器反馈
       |
       v
重力补偿计算实际压力
       |
       v
机械臂末端控制器调节下一步期望位置    ->   放卷轮电机跟着末端一起移动
                                                    |
                                                    v
                                        放卷轮电机跟着末端一起移动
                                                    |
                                                    v
                                        张力控制器调节放卷轮转动速度维持张力恒定


其中各子模块的基本功能由其他程序提供
该控制器只做集成和上层算法
"""

class AFPController:
    def __init__(self):
        self.ur5e_controller = UR5eController()
        