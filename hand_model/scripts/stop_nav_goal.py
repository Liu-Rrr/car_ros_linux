#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import move_base_msgs
import actionlib_msgs.msg 
from std_msgs.msg import String
# from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped

def cancel_nav():
    goal = actionlib_msgs.msg.GoalID()
    for i in range(100000):
        cancel_pub.publish(goal)
    rospy.loginfo("cancel successs")

def deal_message(msg):
    msg = str(msg.data)
    msg = eval(msg)
    print(msg)
    try:
        if(msg["stop_nav_goal"] != None):
            point = msg["stop_nav_goal"]
            rospy.loginfo(point)
            cancel_nav()
    except:
        pass


if __name__ == "__main__":
    rospy.init_node("cancel_nav_goal")
    cancel_pub = rospy.Publisher("/move_base/cancel",actionlib_msgs.msg.GoalID,queue_size=20)
    # cancel_nav()
    rospy.Subscriber("/main_control",String,deal_message)
    rospy.spin()
    

    
