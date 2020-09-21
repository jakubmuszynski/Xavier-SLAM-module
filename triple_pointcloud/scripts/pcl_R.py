#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError
import struct

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud

def create_PointCloud2_v2(new_points, cam_link):

    x = new_points[:, 0]
    y = new_points[:, 1]
    z = new_points[:, 2]

    points2 = list([x[i], y[i], z[i],
                    struct.unpack('I', struct.pack('BBBB',
                           200,
                           200,
                           200,
                           255))[0]] for i in range(len(x)))

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]

    header = Header()
    header.frame_id = cam_link
    pc2 = point_cloud2.create_cloud(header, fields, points2)
    pc2.header.stamp = rospy.Time.now()

    return pc2

# Arguments parser
parser = argparse.ArgumentParser()
parser.add_argument("--voxel_size", "-v", help="set voxel_size for filtration", type=float, default=0.01)
parser.add_argument("--cam_R_id", help="right camera serial number", type=int, default=948122072058)
args = parser.parse_args()

# D435 pipeline camera 2 - R
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device(str(args.cam_R_id))
config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming
pipeline_2.start(config_2)

# Align depth to color 2
align_to_2 = rs.stream.color
align_2 = rs.align(align_to_2)

# Processing blocks 2
pc_2 = rs.pointcloud()
decimate_2 = rs.decimation_filter()
decimate_2.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer_2 = rs.colorizer()

# Node init and publishers definitions
rospy.init_node('pointcloud_merge', anonymous = True)
pub_2 = rospy.Publisher("PointCloud_R", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

bridge = CvBridge()

print("Start node")

while not rospy.is_shutdown():
    # Get data from camera 2
    frames_2 = pipeline_2.wait_for_frames()
    depth_frame_2 = frames_2.get_depth_frame()
    color_frame_2 = frames_2.get_color_frame()

    # Publish align dpth to color image 2
    aligned_frames_2 = align_2.process(frames_2)
    aligned_depth_frame_2 = aligned_frames_2.get_depth_frame()
    align_depth_2 = np.asanyarray(aligned_depth_frame_2.get_data())

    _, _, points_2 = get_point_cloud(depth_frame_2, color_frame_2, pc_2, decimate_2, colorizer_2)
    #points_2 = point_cloud_filtration(points_2, args.voxel_size)
    pcv2_2 = create_PointCloud2_v2(points_2, 'cam_R_link')
    pub_2.publish(pcv2_2)

    rate.sleep()

# Stop streaming
pipeline_2.stop()
