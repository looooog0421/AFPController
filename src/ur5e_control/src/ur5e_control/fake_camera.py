#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def main():
    rospy.init_node("pcd_publisher_node", anonymous=True)
    
    # === 配置 ===
    # 请修改为你的 pcd 文件真实路径
    pcd_file_path = "/home/lgx/Project/AFP/src/il_capture/data/pointcloud/total_surface.pcd" 
    topic_name = "/camera/points"
    frame_id = "world" # 你的点云是在哪个坐标系下的？
    
    # 1. 创建发布者
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    
    # 2. 读取 PCD 文件
    print(f"正在读取点云文件: {pcd_file_path}")
    
    if not os.path.exists(pcd_file_path):
        rospy.logerr("文件不存在！")
        return

    pcd = o3d.io.read_point_cloud(pcd_file_path)
    if pcd.is_empty():
        rospy.logerr("点云为空或读取失败！")
        return

    # 转为 numpy
    points = np.asarray(pcd.points)
    print(f"读取成功！包含 {len(points)} 个点")

    # 如果 pcd 里有颜色
    colors = np.asarray(pcd.colors)
    has_color = False
    if len(colors) > 0:
        has_color = True
        # Open3D 颜色是 0-1，ROS 需要 0-255
        colors = (colors * 255).astype(np.uint8)

    rate = rospy.Rate(30) # 30Hz 发送频率

    while not rospy.is_shutdown():
        # 3. 构造 ROS 消息
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        if has_color:
            # 打包 XYZRGB
            # 注意：这步比较慢，如果是静态点云，其实可以放在 while 外面只生成一次
            # 但为了时间戳实时更新，我们在这里打包
            points_with_color = []
            for i in range(len(points)):
                x, y, z = points[i]
                r, g, b = colors[i]
                # 将 RGB 压缩成一个 float (ROS 标准黑魔法)
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                points_with_color.append([x, y, z, rgb])
            
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1),
            ]
            pc_msg = pc2.create_cloud(header, fields, points_with_color)
        else:
            # 只有 XYZ
            pc_msg = pc2.create_cloud_xyz32(header, points)

        # 4. 发布
        pub.publish(pc_msg)
        rate.sleep()

import os
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass