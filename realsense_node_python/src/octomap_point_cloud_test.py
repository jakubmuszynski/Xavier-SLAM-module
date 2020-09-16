#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import ros_numpy

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration, get_point_cloud_from_topic



def callback(msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    points = point_cloud_filtration(xyz_array, 0.05)

    print("do it")

    pc2 = create_PointCloud2(points, 0.0, True)
    pub_pointcloud.publish(pc2)


# Node init and publisher definition
rospy.init_node('realsense_point_cloud', anonymous = True)
pub_pointcloud = rospy.Publisher("point_cloud2_v2", PointCloud2, queue_size=2)
rospy.Subscriber("point_cloud2_sync", PointCloud2, callback)

rate = rospy.Rate(30) # 30hz

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()
old_points = np.array([[0, 0, 0]]) 

#print("Start node")
rospy.loginfo("Realsense D435 is run")

while not rospy.is_shutdown():

    # create point_cloud
    #_, _, points = get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer, color_intrinsics.width, color_intrinsics.height, cx, cy, fx, fy)


    #points = point_cloud_filtration(points, 0.05)

    #pc2 = create_PointCloud2(points, timestamp_pc, True)
    #pub_pointcloud.publish(pc2)
    a = 1
    rate.sleep()

# Stop streaming
#pipeline_point_cloud.stop()

