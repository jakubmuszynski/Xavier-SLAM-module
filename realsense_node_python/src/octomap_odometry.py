#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from trajectory_fun import get_path_position_orientation

# T265 pipeline
pipeline_trajectory = rs.pipeline()
config_trajectory = rs.config()
config_trajectory.enable_stream(rs.stream.pose)

# Start streaming
prof_tr = pipeline_trajectory.start(config_trajectory)

# Start streaming with requested config
config_trajectory.enable_record_to_file('test2.bag')

# Align to depth 
align_to = rs.stream.color
align = rs.align(align_to)

# Node init and publisher definition
rospy.init_node('realsense_trajectory', anonymous = True)
pub_path = rospy.Publisher("path", Path, queue_size = 100)
pub_odom = rospy.Publisher('odom_t265', Odometry, queue_size=5)
rate = rospy.Rate(200) # 30hz

# init trajectory variables
my_path = Path()
my_path.header.frame_id = 'central'

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()
old_points = np.array([[0, 0, 0]]) 

#print("Start node")
rospy.loginfo("Realsense T265 is run")

while not rospy.is_shutdown():
    
    # Get data from cameras
    trajectory = pipeline_trajectory.wait_for_frames()
    timestamp_tr = trajectory.get_timestamp()
    #print("TR: ", timestamp_tr)

    pose = trajectory.get_pose_frame()

    # create path, get position and orientation
    my_path, position, orientation, odom = get_path_position_orientation(pose, my_path, timestamp_tr, True)

    # publish path
    pub_path.publish(my_path)

    # publish odom
    pub_odom.publish(odom)

    rate.sleep()

# Stop streaming
pipeline_trajectory.stop()

