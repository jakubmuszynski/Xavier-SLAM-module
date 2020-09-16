import time
import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
import struct
import open3d as o3d
import ros_numpy

from open3d.open3d.geometry import create_rgbd_image_from_color_and_depth, create_point_cloud_from_rgbd_image

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header


def get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer, width, height, cx, cy, fx, fy):
    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(
        depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())

    mapped_frame, color_source = color_frame, color_image

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    points = np.reshape(verts, (76800, 3))

    points[:, [1, 2]] = points[:, [2, 1]] 
    points[:, [2]] *= -1.0


    #color = np.ones((240, 320), np.uint8)

    #img_rgb = o3d.geometry.Image((color).astype(np.uint8))
    #img_depth = o3d.geometry.Image((depth_image).astype(np.uint8))

    #rgbd = create_rgbd_image_from_color_and_depth(img_rgb, img_depth)

    #pcd = create_point_cloud_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy), np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    #points2 = np.asarray(pcd.points)

    #points2[:, [0]] *= 10000.0 
    #points2[:, [1]] *= 10000.0 
    #points2[:, [2]] *= 5000.0 

    #print("new_points: ", points2.shape)
    #print('new_points: ', points2.max())
    #print('old_points: ', points.max()) 
 
    return verts, verts, points

def get_point_cloud_from_topic(msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    return xyz_array


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def point_cloud_filtration(points, voxel_size_arg):

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    a = np.array([[-2.0], [0.1], [-2.0]])
    b = np.array([[2.0], [2.0], [2.0]])
    pcd2 = o3d.geometry.crop_point_cloud(pcd, a, b)

    down_pcd = o3d.geometry.voxel_down_sample(pcd2, voxel_size = 0.02)

    statistic_pcd = o3d.geometry.statistical_outlier_removal(down_pcd, 20, 3.0)
    radious_pcd = o3d.geometry.radius_outlier_removal(statistic_pcd[0], 20, 0.15)
    
    new_points = np.asarray(radious_pcd[0].points) 

    return new_points


def transform_point_cloud(points, position, orientation):

    # po kalibracji, zależne od ułożenia sensorów względem siebie 
    
    # przekształcenie na podstawie modelu
    #rotation_calibr = np.array([[0.999968402, -0.006753626, -0.004188075],
    #                            [-0.006685408, -0.999848172, 0.016093893],
    #                            [-0.004296131, -0.016065384, -0.999861654]])

    #points_2 = np.dot(points, rotation_calibr.T)

    #points_2[:, [0]] += -0.015890727
    #points_2[:, [1]] += 0.028273059
    #points_2[:, [2]] += -0.009375589

    # przekształcenie na podstawie kalibracji z programu kalibr

    rotation_calibr = np.array([[0.9995814271318334, 0.027189379057314916, -0.009884745801685224],
                                [-0.02718441482412074, 0.9996302330252859, 0.0006362487578675182],
                                [0.009898389957786727, -0.00036727141109607316, 0.9999509422906472]])

    points_2 = np.dot(points, rotation_calibr.T)

    points_2[:, [0]] += 0.00427195870703538
    points_2[:, [1]] += -0.028419092165645855
    points_2[:, [2]] += 0.0006118486868745438



    c = np.cos(orientation[0] * 3.14/180)
    s = np.sin(orientation[0] * 3.14/180)

    rotation_x = np.array([[1, 0, 0],
                           [0, c,-s],
                           [0, s, c]])

    c = np.cos(orientation[1] * 3.14/180)
    s = np.sin(orientation[1] * 3.14/180)

    rotation_y = np.array([[c, 0,-s],
                           [0, 1, 0],
                           [s, 0, c]])

    c = np.cos((orientation[2]+90) * 3.14/180)
    s = np.sin((orientation[2]+90) * 3.14/180)

    rotation_z = np.array([[c,-s, 0],
                           [s, c, 0],
                           [0, 0, 1]])

    matrix = rotation_z @ rotation_y @ rotation_x

    new_points = np.dot(points_2, matrix.T)

    new_points[:, [0]] += position[0]
    new_points[:, [1]] += position[1]
    new_points[:, [2]] += position[2]   

    return new_points

def create_PointCloud2(new_points, timestamp, sync):

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
    header.frame_id = "camera"
    pc2 = point_cloud2.create_cloud(header, fields, points2)
    if sync:
        #print("point cloud timestamp: ",timestamp)
        t1 = (timestamp / 100000000)
        t2 = (t1 - int(t1)) * 100000

        #t1 = timestamp / 1000.0
        time = rospy.Time(secs=int(t2), nsecs = int((t2 - int(t2))*100))
        #print("point cloud time: ",time)
        pc2.header.stamp = time

    return pc2
