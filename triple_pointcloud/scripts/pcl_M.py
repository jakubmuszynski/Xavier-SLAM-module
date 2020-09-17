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
parser.add_argument("--cam_M_id", help="middle camera serial number", type=int, default=947522072464)
args = parser.parse_args()

# D435 pipeline camera 1 - M
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device(str(args.cam_M_id))
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline_1.start(config_1)

# Align depth to color 1
align_to_1 = rs.stream.color
align_1 = rs.align(align_to_1)

# Processing blocks 1
pc_1 = rs.pointcloud()
decimate_1 = rs.decimation_filter()
decimate_1.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer_1 = rs.colorizer()

# Node init and publishers definitions
rospy.init_node('pointcloud_merge', anonymous = True)
pub_1 = rospy.Publisher("PointCloud_M", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

bridge = CvBridge()

print("Start node")

while not rospy.is_shutdown():
    # Get data from camera 1
    frames_1 = pipeline_1.wait_for_frames()
    depth_frame_1 = frames_1.get_depth_frame()
    color_frame_1 = frames_1.get_color_frame()

    # Publish align dpth to color image 1
    aligned_frames_1 = align_1.process(frames_1)
    aligned_depth_frame_1 = aligned_frames_1.get_depth_frame()
    align_depth_1 = np.asanyarray(aligned_depth_frame_1.get_data())

    _, _, points_1 = get_point_cloud(depth_frame_1, color_frame_1, pc_1, decimate_1, colorizer_1)
    #points_1 = point_cloud_filtration(points_1, args.voxel_size)
    pcv2_1 = create_PointCloud2_v2(points_1, 'cam_M_link')
    pub_1.publish(pcv2_1)

    rate.sleep()

# Stop streaming
pipeline_1.stop()
