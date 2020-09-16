#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import argparse
import message_filters
import math as m

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration, get_point_cloud_from_topic


def cameras_callback(odom_msg, point_cloud2_msg):
    timestamp_pc = point_cloud2_msg.header.stamp
    timestamp_tr = odom_msg.header.stamp
    print("PC: ", timestamp_pc, " TR: ", timestamp_tr, " difference: ", timestamp_tr - timestamp_pc)
    points = get_point_cloud_from_topic(point_cloud2_msg)
    position = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]

    w_r = odom_msg.pose.pose.orientation.w
    x_r = odom_msg.pose.pose.orientation.x
    y_r = odom_msg.pose.pose.orientation.y
    z_r = odom_msg.pose.pose.orientation.z

    pitch =  -m.asin(2.0 * (x_r*z_r - w_r*y_r)) * 180.0 / m.pi;
    roll  =  m.atan2(2.0 * (w_r*x_r + y_r*z_r), w_r*w_r - x_r*x_r - y_r*y_r + z_r*z_r) * 180.0 / m.pi
    yaw   =  m.atan2(2.0 * (w_r*z_r + x_r*y_r), w_r*w_r + x_r*x_r - y_r*y_r - z_r*z_r) * 180.0 / m.pi

    orientation = [pitch, roll, -yaw]

    points = transform_point_cloud(points, position, orientation)
    pc2 = create_PointCloud2(points, 0.0)
    pub_pointcloud.publish(pc2)

# Node init and publisher definition
rospy.init_node('realsense_point_cloud_trajectory', anonymous = True)

odometry = message_filters.Subscriber('odom_t265', Odometry)
point_cloud = message_filters.Subscriber('point_cloud2', PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([odometry, point_cloud], 2, 0.0005, allow_headerless=True)
ts.registerCallback(cameras_callback)

pub_pointcloud = rospy.Publisher("point_cloud2_tr", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()

# Arguments parser
parser = argparse.ArgumentParser()
parser.add_argument("--voxel_size", "-v", help="set voxel_size for filtration", type=float, default=0.01)

args = parser.parse_args()


print("Start node")


while not rospy.is_shutdown():
    
    # Get data from cameras


    # create point_cloud
    b = 1

    rate.sleep()

