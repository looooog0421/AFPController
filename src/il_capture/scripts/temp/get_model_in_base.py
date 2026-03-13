# 获取模具在基坐标系下的位姿

import numpy as np
from scipy.spatial.transform import Rotation as R

def get_relative_pose(T_mocap_base, T_mocap_obj):
    """
    输入:
    T_mocap_base: 4x4 numpy array (基座在动捕系下的矩阵)
    T_mocap_obj:  4x4 numpy array (零件在动捕系下的矩阵)
    
    输出:
    target_pos:  [x, y, z] (零件在基座系下的位置)
    target_quat: [x, y, z, w] (零件在基座系下的姿态四元数)
    """
    
    # 1. 计算基座矩阵的逆矩阵
    # T_base_mocap = inv(T_mocap_base)
    T_base_mocap = np.linalg.inv(T_mocap_base)
    
    # 2. 矩阵乘法得到结果矩阵
    # T_base_obj = T_base_mocap * T_mocap_obj
    T_base_obj = np.dot(T_base_mocap, T_mocap_obj)
    
    # 3. 从结果矩阵中提取位置 (前3行，第4列)
    target_pos = T_base_obj[:3, 3]
    
    # 4. 从结果矩阵中提取旋转矩阵并转为四元数 (前3行，前3列)
    rotation_matrix = T_base_obj[:3, :3]
    r = R.from_matrix(rotation_matrix)
    target_quat = r.as_quat() # 返回格式通常为 [x, y, z, w]
    
    return target_pos, target_quat, T_base_obj

# --- 使用示例 ---

# 假设你的输入矩阵如下 (示例数据)
# 这里的 T_base 和 T_obj 就是你从动捕软件里直接拿到的 4x4 矩阵
matrix_base = np.array([
    [ 0.999856,  0.016928, -0.000914,  0.112686],
    [-0.016925,  0.999848,  0.004154,  1.893302],
    [ 0.000984, -0.004138,  0.999991,  0.756350],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
])

matrix_obj = np.array([
    [ 0.999474, 0.032234, 0.003633, -0.488184],
    [-0.032276, 0.999408, 0.011897, 1.721358],
    [-0.003247, -0.012007, 0.999923, 0.744231],
    [0.000000, 0.000000, 0.000000, 1.000000]
])

pos, quat, result_matrix = get_relative_pose(matrix_base, matrix_obj)

print("计算结果 - 位置 (x, y, z):")
print(pos)
print("\n计算结果 - 四元数 (x, y, z, w):")
print(quat)
print("\n(可选) 完整结果矩阵 T_base_obj:")
print(result_matrix)