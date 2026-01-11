import numpy as np
import cv2
import yaml
import os
from scipy.spatial.transform import Rotation as R

# ================= 1. 参数配置 =================
IMAGE_PATH = "/home/lgx/Project/hand_eye_calib/src/calibrate/raw_data/valid/frame_0000.png"
CAMERA_MATRIX_PATH = '/home/lgx/.ros/camera_info/rgb_camera.yaml'

# 棋盘格参数 (必须与你打印的标定板完全一致)
CHESSBOARD_SIZE = (11, 8)       # (列, 行) 内部角点数
CHESSBOARD_SQUARE_SIZE = 0.01   # 每个格子的物理边长 (米)

# 你提供的标定结果：相机到基坐标系的齐次变换矩阵 (Base_T_Cam)
BASE_T_CAM = np.array([
    [0.99784269, -0.01867954, 0.06293680, -0.48922722],
    [-0.05087262, -0.82596274, 0.56142456, -0.61241286],
    [0.04149630, -0.56341515, -0.82513116, 0.50541867],
    [0.00000000, 0.00000000, 0.00000000, 1.00000000]

])

# ================= 2. 数据加载与处理 =================

def calculate_base_frame_corners():
    # A. 加载内参
    with open(CAMERA_MATRIX_PATH, 'r') as f:
        cam_info = yaml.safe_load(f)
    K = np.array(cam_info['camera_matrix']['data']).reshape(3,3)
    D = np.array(cam_info['distortion_coefficients']['data'])

    # B. 读取并检测
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print(f"错误: 无法读取图片 {IMAGE_PATH}")
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 构造标定板局部 3D 点
    cols, rows = CHESSBOARD_SIZE
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * CHESSBOARD_SQUARE_SIZE

    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # 亚像素精修
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        corners_sub = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

        # 求解 PnP 得到标定板在相机系下的位姿
        success, rvec, tvec = cv2.solvePnP(objp, corners_sub, K, D)

        if success:
            rmat, _ = cv2.Rodrigues(rvec)

            # C. 定义四个角落的索引
            indices = {
                "左上 (TL[0])": 0,
                "右上 (TR)": cols - 1,
                "左下 (BL)": (rows - 1) * cols,
                "右下 (BR)": rows * cols - 1
            }

            print(f"--- 坐标转换结果 (单位: 米) ---")
            header = f"{'角点位置':<15} | {'基座系 X':>10} | {'基座系 Y':>10} | {'基座系 Z':>10}"
            print(header)
            print("-" * len(header))

            for name, idx in indices.items():
                # 1. 计算相机坐标系下的 3D 点: P_cam = R * P_obj + t
                p_obj = objp[idx].reshape(3, 1)
                p_cam = rmat @ p_obj + tvec
                
                # 2. 构造齐次坐标 [x, y, z, 1]
                p_cam_homo = np.array([p_cam[0,0], p_cam[1,0], p_cam[2,0], 1.0])
                
                # 3. 变换到基座坐标系: P_base = T_base_cam * P_cam_homo
                p_base = BASE_T_CAM @ p_cam_homo
                
                bx, by, bz = p_base[:3]
                print(f"{name:<15} | {bx:>10.4f} | {by:>10.4f} | {bz:>10.4f}")
            
            print("-" * len(header))
    else:
        print("识别失败，请检查棋盘格是否在视野内。")

if __name__ == "__main__":
    calculate_base_frame_corners()