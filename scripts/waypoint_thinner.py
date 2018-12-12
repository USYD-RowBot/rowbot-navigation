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




        if count%2 == 0:
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
