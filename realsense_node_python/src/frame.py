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
                     "map",
                     "central")

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0,
                       0,
                       0),
                      (-msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "camera",
                      "map")

    br_cam_M = tf.TransformBroadcaster()
    br_cam_M.sendTransform((0,
                            0,
                            0),
                           (0,
                            0,
                            0,
                            1),
                      rospy.Time.now(),
                      "cam_M_link",
                      "camera")

    br_ML = tf.TransformBroadcaster()
    br_ML.sendTransform((-0.02713135,
                         0.06760438,
                         -0.04535259),
                        (-0.0007557,
                         0.0075024,
                         0.4279183,
                         0.903786),
                      rospy.Time.now(),
                      "cam_L_link",
                      "cam_M_link")

    br_MR = tf.TransformBroadcaster()
    br_MR.sendTransform((-0.01463135,
                         -0.05510438,
                         0.00777241),
                        (-0.0001983,
                         0.0118488,
                         -0.4229403,
                         0.9060801),
                      rospy.Time.now(),
                      "cam_R_link",
                      "cam_M_link")


if __name__ == '__main__':
    rospy.init_node('tf_camera')
    rospy.Subscriber('odom_t265',
                     Odometry,
                     callback)

    rospy.loginfo("TF publisher is run")

    rospy.spin()
