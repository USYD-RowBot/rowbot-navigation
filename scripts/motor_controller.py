#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import Twist
from robotx_gazebo.msg import UsvDrive
from nav_msgs.msg import Odometry



class Node():
    def __init__(self):
        self.pub = None
        self.driveMsg =UsvDrive()
        self.cmd_vel = Twist()
        self.lin_kp = 1
        self.lin_ki = 1
        self.lin_kd = 0
        self.ang_kp = 1
        self.ang_ki = 1
        self.ang_kd = 0
        self.error_lin = 0
        self.error_ang = 0
        self.error_prior_lin = 0
        self.error_prior_ang = 0
        self.integral_ang = 0
        self.integral_lin = 0
        self.lin_vel = 0
        self.lin_vel_y = 0
        self.ang_vel = 0

    def callback(self,data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))

        #self.driveMsg.left = data.linear.x
        #self.driveMsg.right = data.linear.x

        #self.driveMsg.left-=data.angular.z
        #self.driveMsg.right+=data.angular.z
        self.cmd_vel = data


        #print("left: %f, right: %f"%(self.driveMsg.left,self.driveMsg.right))

    def callback_odom(self,data):
        twist_odom = data.twist.twist
        self.ang_vel = twist_odom.angular.z
        self.lin_vel = math.sqrt(twist_odom.linear.x**2 + twist_odom.linear.y**2) * twist_odom.linear.x/abs(twist_odom.linear.x)
        self.lin_vel_y = twist_odom.linear.y
        rospy.loginfo("Current speed is: x"+ str(twist_odom.linear.x)+"\t z:" +str(twist_odom.angular.z))

    def PID_loop(self,dt):
        k = 0.5
        self.error_prior_ang = self.error_ang
        self.error_prior = self.error_ang
        #ang = self.cmd_vel.angular.z -k * self.lin_vel_y*self.cmd_vel.linear.x

        self.error_lin =  self.cmd_vel.linear.x - self.lin_vel
        self.error_ang =  self.cmd_vel.angular.z - self.ang_vel
        self.integral_lin = self.integral_lin+(self.error_lin*dt)
        self.integral_ang = self.integral_ang+(self.error_ang*dt)
        derivative_lin = (self.error_lin - self.error_prior_lin)/dt
        derivative_ang = (self.error_ang - self.error_prior_ang)/dt


        self.driveMsg.left = self.lin_kp * self.error_lin + self.lin_ki * self.integral_lin + self.lin_kd * derivative_lin - self.lin_vel_y *self.cmd_vel.linear.x* k
        self.driveMsg.right = self.lin_kp * self.error_lin + self.lin_ki * self.integral_lin + self.lin_kd * derivative_lin + self.lin_vel_y *self.cmd_vel.linear.x* k

        self.driveMsg.left = self.driveMsg.left - (self.ang_kp * self.error_ang + self.ang_ki * self.integral_ang + self.ang_kd * derivative_ang)
        self.driveMsg.right = self.driveMsg.right + (self.ang_kp * self.error_ang + self.ang_ki * self.integral_ang + self.ang_kd * derivative_ang)
        #Publish the data

        self.pub.publish(self.driveMsg)
if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','cmd_vel')
    out_topic = rospy.get_param('~output_topic','cmd_drive')
    odom_topic = rospy.get_param('~odom_topic','odom')

    rospy.loginfo("Subscribing to <%s>, Publishing to <%s>"%(in_topic,out_topic))
    node=Node()

    # Publisher
    node.pub = rospy.Publisher(out_topic,UsvDrive,queue_size=10)
    node.driveMsg = UsvDrive()

    # Subscriber
    rospy.Subscriber(in_topic,Twist,node.callback)
    rospy.loginfo("Subscribing to <%s>"%(odom_topic))

    rospy.Subscriber("rowbot/odometry/filtered",Odometry,node.callback_odom)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            node.PID_loop(0.05)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
