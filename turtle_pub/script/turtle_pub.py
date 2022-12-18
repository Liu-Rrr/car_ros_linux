#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    #ros init
    rospy.init_node("turtle_pub", anonymous=True)

    #create publisher
    turtle_vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    #loop
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 2.0
        vel_msg.angular.z = 0.8

        #send msg
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle velocity command[%.2f m/s, %.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

        rate.sleep()

if __name__ == "__main__":
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
