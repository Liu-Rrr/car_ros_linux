#include<ros/ros.h>
#include<turtlesim/Pose.h>

//using namespace ros;

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    //print msg
    ROS_INFO("turtle pose: x : %.6f, y : %.6f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
    //ros init
    ros::init(argc, argv, "turtle_subscribe");

    //create handle
    ros::NodeHandle n;

    //create subscribe
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    //loop wait
    ros::spin();

    return 0;
}