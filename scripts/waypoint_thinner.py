#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msgs import Pose
import math
from tf.transformations import euler_from_quaternion



def pathCallback(path_msg):
    thinned_msg = Path()
    thinned = []
    last_pose = Pose()
    count = 0
    for pose_stamped in path_msg.poses:
        pose = pose_stamped.pose

        distance_to_last_pose = math.sqrt((pose.position.x - last_pose.position.x)^2 + (pose.position.y - last_pose.position.y)^2)



        angle_to_last_pose = math.abs(tf.transformations.euler_from_quaternion(last_pose.orientation)[2] - tf.transformations.euler_from_quaternion(pose.orientation)[2])
        if (angle_to_last_pose > 30 or distance_to_last_pose > 5 or count = len(path_msg.poses)):
            thinned.append(pose_stamped)
            last_pose = pose

        count = count+1

    thinned_msg.poses = thinned
    thinned_msg.header.stamp = rospy.Time.now()
    pub.publish(thinned_msg)




if __name__ == "__main__":
    rospy.init_node('paththinner')

    rospy.Subscriber("global_planner/planner/plan",Path,callback)
    pub = rospy.Publisher("waypoints")
