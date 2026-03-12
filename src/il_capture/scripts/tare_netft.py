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
    quat_list: list of quaternions representing the orientation of the sensor, in (x, y, z, w) format
    """
    mft = LinearMFTarer()
    # ft = LinearFTarer()
    ct = LinearCTTarer()

    # 1 计算质量
    for f, q in zip(force_list, quat_list):
        R_mat = Rotation.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()
        mft.add_data(f, R_mat)
    
    res_mft = mft.run()
    m = res_mft['m']
    f0 = res_mft['f0']

    # 2 计算质心
    ct.set_m(m)
    for t, q in zip(torque_list, quat_list):
        R_mat = Rotation.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()
        ct.add_data(t, R_mat)
    res_ct = ct.run()
    c = res_ct['c']
    t0 = res_ct['t0']
    

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