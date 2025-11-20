import mujoco
import numpy as np
import os
import glfw
import time
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from dataclasses import dataclass
import pinocchio as pin
from mujoco import viewer

"""
用于Mujoco仿真的辅助函数和类
接收ros节点里的控制器的指令，控制Mujoco中的机械臂运动
"""
class MujocoSim:
    def __init__(self, model_path):
        # 加载Mujoco模型
        full_path = os.path.abspath(model_path)
        self.model = mujoco.MjModel.from_xml_path(full_path)
        self.data = mujoco.MjData(self.model)

        self.ctrl_q = np.zeros(self.model.nu)  # 关节控制目标位置
        

        self.control_sub = rospy.Subscriber(
                            "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
                            FollowJointTrajectoryActionGoal,
                            self.control_callback
                            )
    
        with viewer.launch_passive(self.model, self.data) as sim:
            while sim.is_running():
                # 更新控制指令
                for i in range(self.model.nu):
                    self.data.ctrl[i] = self.ctrl_q[i]
                
                mujoco.mj_step(self.model, self.data)

                sim.sync()

                time.sleep(self.model.opt.timestep)
                
    def control_callback(self, msg):
        """
        接收控制器的目标关节位置指令
        """
        point: JointTrajectoryPoint = msg.goal.trajectory.points[-1]
        self.ctrl_q = np.array(point.positions)

    


if __name__ == "__main__":
    model_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/scene.xml"
    rospy.init_node("mujoco_sim_node")
    sim = MujocoSim(model_path)
    rospy.spin()
