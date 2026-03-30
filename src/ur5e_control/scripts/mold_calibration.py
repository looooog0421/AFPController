#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模具标定脚本
1. 输入 SW 四点坐标 P_M（模具坐标系，mm）
2. 输入针尖触碰的 Base 坐标 P_B（mm）
3. SVD 求解 T_BM
4. 残差验证
5. 保存 T_BM 到文件
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


# ─────────────────────────────────────────────
# 1. 输入数据（单位：mm）
# ─────────────────────────────────────────────

# SW 模具坐标系下的四个角点
P_M_mm = np.array([
    [ 60.0, 120.0,  -8.0],   # P1
    [ 70.0,  10.0, -15.0],   # P2
    [ 80.0,  10.0, -20.0],   # P3
    [210.0, 120.0,  -4.0],   # P4
], dtype=float)

# 针尖触碰时 Base 坐标系下记录的坐标（mm）
P_B_mm = np.array([
    [-528.5,  -59.5,  -5.5],  # P1
    [-538.5, -169.0,   0.9],  # P2
    [-548.0, -168.5,   6.5],  # P3
    [-680.7,  -58.8,  -6.3],  # P4
], dtype=float)


# ─────────────────────────────────────────────
# 2. 单位统一转换为 m
# ─────────────────────────────────────────────
P_M = P_M_mm / 1000.0
P_B = P_B_mm / 1000.0


# ─────────────────────────────────────────────
# 3. SVD 刚体配准求 T_BM
# ─────────────────────────────────────────────
def solve_T_BM(P_M, P_B):
    """
    Kabsch / SVD 刚体配准
    P_M: (N,3) 模具坐标系下的点
    P_B: (N,3) Base 坐标系下对应点
    返回 T_BM (4x4)
    """
    assert P_M.shape == P_B.shape and P_M.shape[1] == 3

    c_M = P_M.mean(axis=0)
    c_B = P_B.mean(axis=0)

    A = P_M - c_M
    B = P_B - c_B

    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    R_BM = Vt.T @ U.T

    # 处理反射（行列式为 -1 时修正）
    if np.linalg.det(R_BM) < 0:
        Vt[-1, :] *= -1
        R_BM = Vt.T @ U.T

    t_BM = c_B - R_BM @ c_M

    T_BM = np.eye(4)
    T_BM[:3, :3] = R_BM
    T_BM[:3,  3] = t_BM

    return T_BM


T_BM = solve_T_BM(P_M, P_B)


# ─────────────────────────────────────────────
# 4. 残差验证
# ─────────────────────────────────────────────
R_BM = T_BM[:3, :3]
t_BM = T_BM[:3,  3]

P_B_pred = (R_BM @ P_M.T).T + t_BM
residuals = np.linalg.norm(P_B_pred - P_B, axis=1)

print("=" * 50)
print("T_BM (模具坐标系 → Base 坐标系):")
print(np.round(T_BM, 6))
print()
print("旋转矩阵 R_BM:")
print(np.round(R_BM, 6))
print()
print("平移向量 t_BM (m):")
print(np.round(t_BM, 6))
print()
print("各点配准残差 (mm):")
for i, r in enumerate(residuals):
    print(f"  P{i+1}: {r*1000:.3f} mm")
print(f"RMS 残差: {np.sqrt(np.mean(residuals**2))*1000:.3f} mm")
print("=" * 50)

# 旋转角度验证（欧拉角，方便直觉判断）
euler = R.from_matrix(R_BM).as_euler('xyz', degrees=True)
print(f"旋转欧拉角 (xyz, deg): roll={euler[0]:.2f}°  pitch={euler[1]:.2f}°  yaw={euler[2]:.2f}°")
print("=" * 50)


# ─────────────────────────────────────────────
# 5. 保存结果
# ─────────────────────────────────────────────
save_path = "/home/hzk/AFPController/src/ur5e_control/scripts/T_BM.npy"
np.save(save_path, T_BM)
print(f"T_BM 已保存到: {save_path}")
