#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def handle_odom_pose(ros_odom_msg, tf_frame_id):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t2 = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = tf_frame_id
    t.child_frame_id = "base_link"
    t.transform.translation.x = ros_odom_msg.pose.pose.position.x
    t.transform.translation.y = ros_odom_msg.pose.pose.position.y
    t.transform.translation.z = 0
    t.transform.rotation.x = ros_odom_msg.pose.pose.orientation.x
    t.transform.rotation.y = ros_odom_msg.pose.pose.orientation.y
    t.transform.rotation.z = ros_odom_msg.pose.pose.orientation.z
    t.transform.rotation.w = ros_odom_msg.pose.pose.orientation.w
    t2.child_frame_id = "odom"
    t2.header.frame_id="map"
    t2.transform.rotation.w = 1
    t2.header.stamp = rospy.Time.now()


    br.sendTransform(t)
    br.sendTransform(t2)
def callback(data):
    handle_odom_pose(data,"odom")


if __name__ == "__main__":
    rospy.init_node('odom_to_tf')
    rospy.Subscriber("odom",Odometry,callback)
    rospy.spin()
