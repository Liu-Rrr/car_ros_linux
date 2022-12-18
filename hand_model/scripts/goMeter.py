#! /usr/bin/python 
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

def out():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	rospy.init_node('keyboard')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    