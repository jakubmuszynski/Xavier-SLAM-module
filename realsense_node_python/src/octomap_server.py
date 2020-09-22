#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import message_filters
import math as m

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration, get_point_cloud_from_topic


def cameras_callback(odom_msg, point_cloud2_msg):
    timestamp_pc = point_cloud2_msg.header.stamp
    timestamp_tr = odom_msg.header.stamp
    print("PC: ", timestamp_pc, " TR: ", timestamp_tr, " difference: ", timestamp_tr - timestamp_pc)

    point_cloud2_msg.header.stamp = rospy.Time()   
    odom_msg.header.stamp = rospy.Time()  
 
    pub_odom.publish(odom_msg)
    pub_pointcloud.publish(point_cloud2_msg)

# Node init 
rospy.init_node('realsense_server', anonymous = True)

# Subscriber definition
odometry = message_filters.Subscriber('odom_t265', Odometry)
point_cloud = message_filters.Subscriber('point_cloud2', PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([odometry, point_cloud], 2, 0.00000005, allow_headerless=True)
ts.registerCallback(cameras_callback)

# Publisher definition
pub_odom = rospy.Publisher('odom_t265_sync', Odometry, queue_size=20)
pub_pointcloud = rospy.Publisher("point_cloud2_sync", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()

#print("Start node")
rospy.loginfo("Realsense server is run")


while not rospy.is_shutdown():
    
    # Get data from cameras
    b=1
    rate.sleep()

