import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg

import termios
import tty
import os
import time
import math
import json


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    return

def publish_status(broadcaster, status, from_cam, to_cam):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = from_cam

    static_transformStamped.child_frame_id = to_cam
    static_transformStamped.transform.translation.x = status['x']['value']
    static_transformStamped.transform.translation.y = status['y']['value']
    static_transformStamped.transform.translation.z = status['z']['value']

    quat = tf.transformations.quaternion_from_euler(math.radians(status['roll']['value']),
                                                    math.radians(status['pitch']['value']),
                                                    math.radians(status['azimuth']['value']))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':

    file_MR = open('src/triple_pointcloud/scripts/transform_R-M.txt', 'r')
    file_MR_list = []
    print
    print 'R -> M transform:'
    for elem in file_MR:
        file_MR_list.append(elem)
        print float(elem)
    file_MR.close()
    x1, y1, z1, yaw1, pitch1, roll1 = [float(arg) for arg in file_MR_list]

    file_ML = open('src/triple_pointcloud/scripts/transform_L-M.txt', 'r')
    file_ML_list = []
    print
    print 'L -> M transform:'
    for elem in file_ML:
        file_ML_list.append(elem)
        print float(elem)
    file_ML.close()
    x2, y2, z2, yaw2, pitch2, roll2 = [float(arg) for arg in file_ML_list]

    file_Mmap = open('src/triple_pointcloud/scripts/transform_M-map.txt', 'r')
    file_Mmap_list = []
    print
    print 'M -> map transform:'
    for elem in file_Mmap:
        file_Mmap_list.append(elem)
        print float(elem)
    file_Mmap.close()
    x3, y3, z3, yaw3, pitch3, roll3 = [float(arg) for arg in file_Mmap_list]

    file_Mcam = open('src/triple_pointcloud/scripts/transform_camera-map.txt', 'r')
    file_Mcam_list = []
    print
    print 'M -> cam transform:'
    for elem in file_Mcam:
        file_Mcam_list.append(elem)
        print float(elem)
    file_Mcam.close()
    x4, y4, z4, yaw4, pitch4, roll4 = [float(arg) for arg in file_Mcam_list]

    status1 = {'mode': 'pitch',
              'x': {'value': x1, 'step': 0.1},
              'y': {'value': y1, 'step': 0.1},
              'z': {'value': z1, 'step': 0.1},
              'azimuth': {'value': yaw1, 'step': 1},
              'pitch': {'value': pitch1, 'step': 1},
              'roll': {'value': roll1, 'step': 1},
              'message': ''}
    status2 = {'mode': 'pitch',
              'x': {'value': x2, 'step': 0.1},
              'y': {'value': y2, 'step': 0.1},
              'z': {'value': z2, 'step': 0.1},
              'azimuth': {'value': yaw2, 'step': 1},
              'pitch': {'value': pitch2, 'step': 1},
              'roll': {'value': roll2, 'step': 1},
              'message': ''}
    status3 = {'mode': 'pitch',
              'x': {'value': x3, 'step': 0.1},
              'y': {'value': y3, 'step': 0.1},
              'z': {'value': z3, 'step': 0.1},
              'azimuth': {'value': yaw3, 'step': 1},
              'pitch': {'value': pitch3, 'step': 1},
              'roll': {'value': roll3, 'step': 1},
              'message': ''}
    status4 = {'mode': 'pitch',
              'x': {'value': x4, 'step': 0.1},
              'y': {'value': y4, 'step': 0.1},
              'z': {'value': z4, 'step': 0.1},
              'azimuth': {'value': yaw4, 'step': 1},
              'pitch': {'value': pitch4, 'step': 1},
              'roll': {'value': roll4, 'step': 1},
              'message': ''}

    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster1 = tf2_ros.StaticTransformBroadcaster()
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    broadcaster4 = tf2_ros.StaticTransformBroadcaster()

    status1_keys = [key[0] for key in status1.keys()]
    publish_status(broadcaster1, status1, 'cam_M_link', 'cam_R_link')

    time.sleep(1)

    status2_keys = [key[0] for key in status2.keys()]
    publish_status(broadcaster2, status2, 'cam_M_link', 'cam_L_link')

    time.sleep(1)

    status3_keys = [key[0] for key in status3.keys()]
    publish_status(broadcaster3, status3, 'map', 'cam_M_link')

    time.sleep(1)

    status4_keys = [key[0] for key in status4.keys()]
    publish_status(broadcaster4, status4, 'camera', 'map')

    print
    print 'Transforms published'
    print 'Press Q to quit'
    print

    while True:
        kk = getch()
        if kk.upper() == 'Q':
            sys.stdout.write('\n')
            exit(0)
