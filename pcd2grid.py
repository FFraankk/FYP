#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import open3d as o3d

def pcd_to_occupancy_grid():
    rospy.init_node('pcd_to_pgm_node')
    
    # --- 参数设置 ---
    pcd_path = "/home/nvidia/catkin_ws/src/hdl_localization/data/LS_2_3_raw_points.pcd" # 你的PCD路径
    map_res = 0.02     # 分辨率 2cm
    z_min = 4.5         # 截取高度下限（相对于该楼层地面）
    z_max = 5.1        # 截取高度上限
    # ----------------
    
    # 1. 读取PCD
    rospy.loginfo("Loading PCD...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    
    # 2. 高度滤波 (截取你想要的楼层范围)
    mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
    filtered_points = points[mask]
    
    # 3. 计算地图边界
    x_min, y_min = np.min(filtered_points[:, :2], axis=0)
    x_max, y_max = np.max(filtered_points[:, :2], axis=0)
    
    width = int((x_max - x_min) / map_res) + 1
    height = int((y_max - y_min) / map_res) + 1
    
    # 4. 初始化地图数据 (-1 代表未知, 0 代表空闲, 100 代表障碍物)
    grid_data = np.full((height, width), 0, dtype=np.int8)
    
    # 5. 投影点云到栅格
    for pt in filtered_points:
        ix = int((pt[0] - x_min) / map_res)
        iy = int((pt[1] - y_min) / map_res)
        if 0 <= ix < width and 0 <= iy < height:
            grid_data[iy, ix] = 100
            
    # 6. 构建并发布 OccupancyGrid 消息
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = map_res
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = x_min
    msg.info.origin.position.y = y_min
    msg.data = grid_data.flatten().tolist()
    
    rospy.loginfo("Map is ready! Use 'rosrun map_server map_saver -f my_floor' to save it.")
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        pcd_to_occupancy_grid()
    except rospy.ROSInterruptException:
        pass