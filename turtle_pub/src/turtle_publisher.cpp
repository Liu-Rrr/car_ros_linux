#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    //ros init
    ros::init(argc, argv, "turtle_publisher");

    //create handle
    ros::NodeHandle n;

    //create Publisher, topic:/turtle1/cmd_vel, msg:geometry_msgs::Twist
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    //init loop
    ros::Rate loop_rate(5);

    //int count = 0;
    while (ros::ok())
    {
        /* code */
        //init msg
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 1.0;
        vel_msg.angular.z = 0.5;

        //send msg
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Publish turtle velocity command[%.2f m/s, %.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);

        //send rate
        loop_rate.sleep();
    }

    return 0;
}
