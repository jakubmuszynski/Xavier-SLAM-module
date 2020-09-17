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
parser.add_argument("--cam_L_id", help="left camera serial number", type=int, default=938422071315)
args = parser.parse_args()

# D435 pipeline camera 3 - L
pipeline_3 = rs.pipeline()
config_3 = rs.config()
config_3.enable_device(str(args.cam_L_id))
config_3.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_3.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline_3.start(config_3)

# Align depth to color 3
align_to_3 = rs.stream.color
align_3 = rs.align(align_to_3)

# Processing blocks 3
pc_3 = rs.pointcloud()
decimate_3 = rs.decimation_filter()
decimate_3.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer_3 = rs.colorizer()

# Node init and publishers definitions
rospy.init_node('pointcloud_merge', anonymous = True)
pub_3 = rospy.Publisher("PointCloud_L", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

bridge = CvBridge()

print("Start node")

while not rospy.is_shutdown():
    # Get data from camera 3
    frames_3 = pipeline_3.wait_for_frames()
    depth_frame_3 = frames_3.get_depth_frame()
    color_frame_3 = frames_3.get_color_frame()

    # Publish align dpth to color image 3
    aligned_frames_3 = align_3.process(frames_3)
    aligned_depth_frame_3 = aligned_frames_3.get_depth_frame()
    align_depth_3 = np.asanyarray(aligned_depth_frame_3.get_data())

    _, _, points_3 = get_point_cloud(depth_frame_3, color_frame_3, pc_3, decimate_3, colorizer_3)
    #points_3 = point_cloud_filtration(points_3, args.voxel_size)
    pcv2_3 = create_PointCloud2_v2(points_3, 'cam_L_link')
    pub_3.publish(pcv2_3)

    rate.sleep()

# Stop streaming
pipeline_3.stop()
