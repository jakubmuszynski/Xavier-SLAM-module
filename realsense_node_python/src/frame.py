#!/usr/bin/env python
import roslib
import rospy
import tf

from nav_msgs.msg import Odometry


def callback(msg):
    #br = tf.TransformBroadcaster()
    #br.sendTransform((-msg.pose.pose.position.x,
    #                  msg.pose.pose.position.y,
    #                  -msg.pose.pose.position.z),
    #                 (-msg.pose.pose.orientation.y,
    #                  msg.pose.pose.orientation.x,
    #                  msg.pose.pose.orientation.z,
    #                  msg.pose.pose.orientation.w),
    #                 rospy.Time(),
    #                 "central",
    #                 "map")

    br1 = tf.TransformBroadcaster()
    br1.sendTransform((-msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      - msg.pose.pose.position.z),
                     (0,
                      0,
                      0,
                      1),
                     rospy.Time.now(),
                     "central",
                     "map")

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0,
                       0,
                       0),
                      (-msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "map",
                      "camera")

    br_cam_M = tf.TransformBroadcaster()
    br_cam_M.sendTransform((0,
                            0,
                            0),
                           (0,
                            0,
                            0,
                            1),
                      rospy.Time.now(),
                      "camera",
                      "cam_M_link")

    br_ML = tf.TransformBroadcaster()
    br_ML.sendTransform((-0.075813,
                         -0.043789,
                         -0.0709),
                        (0,
                         0,
                         0.5,
                         0.8660254),
                      rospy.Time.now(),
                      "cam_M_link",
                      "cam_L_link")

    br_temp1 = tf.TransformBroadcaster()
    br_temp1.sendTransform((0,
                         0,
                         0),
                        (0,
                         0,
                         -0.5,
                         0.8660254),
                      rospy.Time.now(),
                      "cam_L_link",
                      "temp1")

    br_temp2 = tf.TransformBroadcaster()
    br_temp2.sendTransform((0.075813,
                         -0.043789,
                         0.0709),
                        (0,
                         0,
                         0,
                         1),
                      rospy.Time.now(),
                      "temp1",
                      "temp2")

    br_MR = tf.TransformBroadcaster()
    br_MR.sendTransform((0.075813,
                         -0.043789,
                         -0.0709),
                        (0,
                         0,
                         -0.5,
                         0.8660254),
                      rospy.Time.now(),
                      "temp2",
                      "cam_R_link")


if __name__ == '__main__':
    rospy.init_node('tf_camera')
    rospy.Subscriber('odom_t265',
                     Odometry,
                     callback)

    rospy.loginfo("TF publisher is run")

    rospy.spin()
