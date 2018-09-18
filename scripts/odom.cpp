#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
class Odom{
public:


  double vx;
  double vy;
  double vth;
  Odom(){
    double vx = 0;
    double vy = 0;
    double vth = 0;
  }

  void callback(const geometry_msgs::Twist::ConstPtr& msg){
    double k1 = 2;
    double k2 = 1.5;
    vx = msg->linear.x *k1;
    vy = 0;
    vth = msg->angular.z *k2;
  }

};



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("rowbot/odom/motor", 50);
  tf::TransformBroadcaster odom_broadcaster;

  Odom a;

  double x = 0;
  double y = 0;
  double th = 0;

  double vx = a.vx;
  double vy = a.vy;
  double vth = a.vth;

  double ax = a.vx;
  double ay = a.vy;
  double ath = a.vth;

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &Odom::callback, &a);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(200);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    double k = 1.5;
    if (ax>0){ax = a.vx*k;}
    else{ax = a.vx*0.9;}

    ay = a.vy;
    ath = a.vth*1;
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double k1 = 0.005;
    double k2 = 0.005;

    double delta_vx = (ax * cos(th) - ay * sin(th)) * dt -vx*k1;
    double delta_vy = (ax * sin(th) + ay * cos(th)) * dt -vy*k1;
    double delta_vth = ath * dt -vth*k2;
    vx += delta_vx;
    vy += delta_vy;
    vth += delta_vth;

    double delta_x = (vx  * dt);
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = 0.01;
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
