#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "./motor_drive/can_drive.h"


double	RobotV = 0.0;
double	YawRate = 0.0;
boost::array<double, 36> odom_pose_covariance_static = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance_static = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_pose_covariance_move = {
    {1e-3, 0, 0, 0, 0, 0, 
    0, 1e-3, 0, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e3}};
boost::array<double, 36> odom_twist_covariance_move = {
    {1e-3, 0, 0, 0, 0, 0, 
    0, 1e-3, 0, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e3}};
void Sleep(int ms)
{
  struct timeval delay;
  delay.tv_sec = 0;
  delay.tv_usec = ms*1000;
  select(0,NULL,NULL,NULL,&delay);
}

void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV = msg.linear.x;
	YawRate = msg.angular.z;
}
int main(int argc, char** argv){
  info_robot z_bot;
  float cur_pos_left=0.0,cur_pos_right=0.0,last_pos_left=0.0,last_pos_right=0.0;
  ros::init(argc, argv, "info_robot_bringup");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 30);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double distance = 0.0;
  double vx = 0.1;
  double vy = 0;
  double vth = 0.1;
  double vr = 0.0;
  double vl = 0.0;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Subscriber sub = n.subscribe("cmd_vel", 30, cmdCallback);
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
  
    // Sleep(10);
    cur_pos_left = z_bot.get_motor_pos(left_wheel);
    cur_pos_right = z_bot.get_motor_pos(right_wheel);
    //  printf("pos_left:%f---pos_right:%f\n",cur_pos_left,cur_pos_right);
    // check for incoming messages
    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    vr = (cur_pos_right - last_pos_right)/dt;
    vl = (cur_pos_left - last_pos_left)/dt;
    vx = (vr + vl)/2;
    vth = (vr - vl)/z_bot.wheel_length;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;       //机器人运动学

  

    x += delta_x;
    y += delta_y;
    th += delta_th;
    //里程计标定
    // distance = sqrt(x*x + y*y);
    // if (fabs(th) < 2*pi)
    // {
    //   z_bot.writespeed(0.0,2*pi/20.0);
    //   printf("theta:%lf\n",th);
    // }
    // else
    // {
    //     z_bot.writespeed(0.0,0.0);
    // }
    
  //   //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //   //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

  //   //send the transform
    odom_broadcaster.sendTransform(odom_trans);

  //   //next, we'll publish the odometry message over ROS
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
    if (fabs(RobotV) < 1e-5 && fabs(YawRate)<1e-5)
    {
    odom.pose.covariance = odom_pose_covariance_static;
    odom.twist.covariance = odom_twist_covariance_static;
    }
    else
    {
    odom.pose.covariance = odom_pose_covariance_move;
    odom.twist.covariance = odom_twist_covariance_move;
    }
    
    // printf("t%lf \n",z_bot.odom_pose_covariance[14]);
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    last_pos_left = cur_pos_left;
    last_pos_right = cur_pos_right;
    
    z_bot.writespeed(RobotV,YawRate);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  return 0;
}