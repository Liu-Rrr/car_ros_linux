#!/usr/bin/python
# -*- coding: utf-8 -*-
from os import stat
from threading import Thread
from std_msgs.msg import String
from time import time
import rospy
from geometry_msgs.msg import Twist,Pose2D
import time
import json
def stop_move():
    global status,pub
    while(True):
        if(status):
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
        
def update_status():
    global status
    if(status == False):
        rospy.loginfo("stop stop stop!")
        status = True
        stop_nav_going()
    else:
        rospy.loginfo("receive status")
        status = False

def deal_message(msg):
    msg = str(msg.data)
    msg = eval(msg)
    try:
        if(msg['stop_move'] != None):
            status = msg["stop_move"]
            rospy.loginfo(status)
            update_status()
    except:
        pass

def stop_nav_going():
    mainPub = rospy.Publisher("/main_control",String,queue_size=25)
    nav_stop = "{\"stop_nav_goal\" : \"stop\"}"
    print(nav_stop)
    mainPub.publish(nav_stop)

def thread_stop():
    rospy.Subscriber("/main_control",String,deal_message)

if __name__ == "__main__":
    status = False
    rospy.init_node("stop_move")
    #比导航频率高
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=25)
    stop_move_thread = Thread(target=thread_stop)
    stop_move_thread.start()
    stop_move()
