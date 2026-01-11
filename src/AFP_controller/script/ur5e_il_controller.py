import rospy
import numpy as np
import pinocchio as pin
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R # <--- 必须引入这个数学神器

class AdmittanceController:
    def __init__(self):

        urdf_path = "/home/lgx/Project/AFP/src/afp_mjc/env/mujoco_ur5e/ur5e.urdf"

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_frame_name = "flange"
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)

        # 初始化阻抗控制参数
        self.admittance_gain = np.diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01])  # XYZ位置和RPY角度的阻抗增益

        if self.model.nq != 6:
            raise ValueError("当前仅支持6自由度机械臂的阻抗控制,当前自由度: {}".format(self.model.nq))
        
        self.num_joints = self.model.nq
        self.target_force = np.zeros(6)  # 目标力/力矩
        self.target_pos = np.zeros(3)  # 目标位置
        self.target_ori = np.zeros(4)  # 目标姿态（四元数）

        self.current_pos = np.zeros(3)
        self.current_ori = np.zeros(4)  # 四元数表示的当前姿态
        self.current_force = np.zeros(6)
        self.current_joints = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)

        # 订阅末端执行器位置和力传感器数据
        self.il_sub = rospy.Subscriber('/il/action', Float32MultiArray, self.il_callback)
        self.wrench_sub = rospy.Subscriber('/ee_wrench', WrenchStamped, self.wrench_callback)
        self.pos_sub = rospy.Subscriber('/ee_pose', PoseStamped, self.pos_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        self.joint_pub = rospy.Publisher(
            "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=1
        )

        self.rate = rospy.Rate(100)  # 控制频率100Hz
        
    def joint_callback(self, msg):
        # 确保接收到的关节顺序和 Pinocchio 模型一致
        # 简单起见，这里假设顺序是对的。严谨的做法是按 name 排序。
        if len(msg.position) == self.num_joints:
            self.current_joints = np.array(msg.position)