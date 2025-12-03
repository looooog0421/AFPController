import rospy
import threading
import numpy
from scipy.spatial.transform import Rotation
from linear import LinearCTTarer, LinearFTarer, LinearMFTarer, LinearTarer
import open3d as o3d

def tare_netft(force_list, torque_list, quat_list):
    """
    重力补偿标定
    force_list: list of force readings from netft sensor
    torque_list: list of torque readings from netft sensor
    quat_list: list of quaternions representing the orientation of the sensor, in (w, x, y, z) format
    """
    mft = LinearMFTarer()
    ft = LinearFTarer()
    ct = LinearCTTarer()

    # 将姿态和力信息加入补偿标定器
    for f_raw, t_raw, q_wxyz in zip(force_list, torque_list, quat_list):
        R_mat = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()

        mft.add_data(f_raw.reshape(3, 1), R_mat)

    # 求解质量和静态力偏置
    mf_result = mft.run()
    m = mf_result['m']
    f0 = mf_result['f0']
    # print("Mass:", m)

    ft.set_m(m)

    for f_raw, q_wxyz in zip(force_list, quat_list):
        R_mat = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()
        ft.add_data(f_raw.reshape(3, 1), R_mat)

    ft_result = ft.run()
    f0 = ft_result['f0']
    # print("Refined static force offset:", f0)

    ct.set_m(m)

    for t_raw, q_wxyz in zip(torque_list, quat_list):
        R_mat = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]).as_matrix()
        ct.add_data(t_raw.reshape(3, 1), R_mat)

    ct_result = ct.run()
    c = ct_result['c']
    t0 = ct_result['t0']
    # print("COM position:", c)
    # print("Static torque offset:", t0)
    
    return m, f0, c, t0

if __name__ == "__main__":
    # 示例数据
    force_list = [numpy.array([0.0, 0.0, 30.1]), numpy.array([0, 0, 30])]
    torque_list = [numpy.array([0.0, 0.0, 0.0]), numpy.array([0.0, 0.0, 0.0])]
    quat_list = [numpy.array([1.0, 0.0, 0.0, 0.0]), numpy.array([1.0, 0.0, 0.0, 0.0])]

    tare_netft(force_list, torque_list, quat_list)

    # visualize
    geometries = []
    base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    geometries.append(base_frame)
    for pyft_pose in quat_list:
        R_mat = Rotation.from_quat([pyft_pose[1], pyft_pose[2], pyft_pose[3], pyft_pose[0]]).as_matrix()
        ft_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
        ft_frame.rotate(R_mat, center=(0, 0, 0))
        geometries.append(ft_frame)
    o3d.visualization.draw_geometries(geometries)