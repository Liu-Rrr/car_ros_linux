#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

def postCallback(msg):
    rospy.loginfo("Turtle post: x: %.6f, y: %.6f", msg.x, msg.y)

def turtle_sub():
    #ros init
    rospy.init_node("turtle_subscribe", anonymous=True)

    #create subscribe
    rospy.Subscriber("/turtle1/pose", Pose, queue_size=10, callback=postCallback)

    #loop wait
    rospy.spin()

if __name__ == "__main__":
    try:
        turtle_sub()
    except rospy.ROSInterruptException:
        pass