import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0
last_time = rospy.rostime.get_time()

def get_twist(twist_data):
    global vx
    global vth
    vx = twist_data.linear.x
    vth = twist_data.angular.z

def calculate():
    global vx
    global vth
    global y
    global x
    global y
    global th
    global last_time
    current_time = rospy.rostime.get_time()


def init():
    rospy.init_node('create_odom', anonymous=True)
    rospy.Subscriber("cmd_vel")
    rospy.Publisher("odom")
    rate_hz = 100
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        rospy.spinOnce()
        calculate()
        last_time = rospy.rostime.get_time()
        rate.sleep()
