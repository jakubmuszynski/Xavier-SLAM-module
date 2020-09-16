#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration


# D435 pipeline
pipeline_point_cloud = rs.pipeline()
config_point_cloud = rs.config()
config_point_cloud.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_point_cloud.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
prof_pc = pipeline_point_cloud.start(config_point_cloud)

# Align to depth 
align_to = rs.stream.color
align = rs.align(align_to)

# Node init and publisher definition
rospy.init_node('realsense_point_cloud', anonymous = True)
pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=10)
rate = rospy.Rate(30) # 30hz

# get color camera data
profile = pipeline_point_cloud.get_active_profile()
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()

camera_info = CameraInfo()
camera_info.width = color_intrinsics.width
camera_info.height = color_intrinsics.height
camera_info.distortion_model = 'plumb_bob'
cx = color_intrinsics.ppx
cy = color_intrinsics.ppy
fx = color_intrinsics.fx
fy = color_intrinsics.fy
camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
camera_info.D = [0, 0, 0, 0, 0]
camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()
old_points = np.array([[0, 0, 0]]) 

#print("Start node")
rospy.loginfo("Realsense D435 is run")

while not rospy.is_shutdown():
    
    # Get data from cameras
    frames = pipeline_point_cloud.wait_for_frames()
    timestamp_pc = frames.get_timestamp()
    #print("PC: ", timestamp_pc)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # create point_cloud
    _, _, points = get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer, color_intrinsics.width, color_intrinsics.height, cx, cy, fx, fy)
    points = point_cloud_filtration(points, 0.05)

    pc2 = create_PointCloud2(points, timestamp_pc, True)
    pub_pointcloud.publish(pc2)

    rate.sleep()

# Stop streaming
pipeline_point_cloud.stop()

