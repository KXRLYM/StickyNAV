#!/usr/bin/python3

import rospy
import tf
from nav_msgs.msg import Odometry

br = tf.TransformBroadcaster()

br_map_to_odom = tf.TransformBroadcaster()

def callback(odom_msg):
    x = odom_msg.pose.pose.orientation.x
    y = odom_msg.pose.pose.orientation.y
    z = odom_msg.pose.pose.orientation.z
    w = odom_msg.pose.pose.orientation.w

    i = odom_msg.pose.pose.position.x
    j = odom_msg.pose.pose.position.y
    k = odom_msg.pose.pose.position.z

    #br.sendTransform(
    #    (i, j, k),
    #    (x,y,z,w),
    #    odom_msg.header.stamp,
    #    odom_msg.child_frame_id,
    #    odom_msg.header.frame_id,
    #)

    br_map_to_odom.sendTransform(
        (0, 0, 0),
        (0,0,0,1),
        odom_msg.header.stamp,
        "odom",
        "map",
    )

if __name__ == "__main__":
    rospy.init_node("odom_broadcaster")
    sub = rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()