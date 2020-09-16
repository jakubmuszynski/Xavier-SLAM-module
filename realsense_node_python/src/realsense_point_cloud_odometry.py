#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import argparse

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from trajectory_fun import get_path_position_orientation

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration


# D435 pipeline
pipeline_point_cloud = rs.pipeline()
config_point_cloud = rs.config()
config_point_cloud.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_point_cloud.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# T265 pipeline
pipeline_trajectory = rs.pipeline()
config_trajectory = rs.config()
config_trajectory.enable_stream(rs.stream.pose)

# Start streaming
prof_pc = pipeline_point_cloud.start(config_point_cloud)
prof_tr = pipeline_trajectory.start(config_trajectory)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
#clipping_distance_in_meters = 1 #1 meter
#clipping_distance = clipping_distance_in_meters / depth_scale

# Start streaming with requested config
config_point_cloud.enable_record_to_file('test1.bag')
config_trajectory.enable_record_to_file('test2.bag')

# Align to depth 
align_to = rs.stream.color
align = rs.align(align_to)

# Node init and publisher definition
rospy.init_node('realsense_point_cloud_trajectory', anonymous = True)
pub_path = rospy.Publisher("path", Path, queue_size = 100)
pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
pub_odom = rospy.Publisher('odom_t265', Odometry, queue_size=1)
rate = rospy.Rate(30) # 30hz

# init trajectory variables
my_path = Path()
my_path.header.frame_id = 'camera'

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()
old_points = np.array([[0, 0, 0]]) 

# Arguments parser
parser = argparse.ArgumentParser()
parser.add_argument("--voxel_size", "-v", help="set voxel_size for filtration", type=float, default=0.01)

args = parser.parse_args()


print("Start node")


while not rospy.is_shutdown():
    
    # Get data from cameras
    frames = pipeline_point_cloud.wait_for_frames()
    #timestamp_pc = frames.get_timestamp()
    trajectory = pipeline_trajectory.wait_for_frames()
    timestamp_pc = frames.get_timestamp()
    timestamp_tr = trajectory.get_timestamp()
    print("PC: ", timestamp_pc, " TR: ", timestamp_tr, " difference: ", timestamp_tr - timestamp_pc)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    pose = trajectory.get_pose_frame()

    # create path, get position and orientation
    my_path, position, orientation, odom = get_path_position_orientation(pose, my_path, timestamp_tr, False)

    # publish odom
    pub_odom.publish(odom)

    #publish path
    pub_path.publish(my_path)

    # Remove background - Set pixels further than clipping_distance to grey
    #grey_color = 153
    #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    #bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # create point_cloud
    _, _, points = get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer)

    points = transform_point_cloud(points, position, orientation)
    points = point_cloud_filtration(points, args.voxel_size)
    pc2 = create_PointCloud2(points, timestamp_pc, False)
    pub_pointcloud.publish(pc2)

    rate.sleep()

# Stop streaming
pipeline_point_cloud.stop()
pipeline_trajectory.stop()

