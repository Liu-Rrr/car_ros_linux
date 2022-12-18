#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from symbol import import_from
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
def point_goal(x,y,z,w):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #point xyz
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    #point oth
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    return goal

def send_goal():
    #订阅move_base的活动server
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    rospy.loginfo("wait for move_base")
    #等待回应
    ac.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base")
    
    # ar_front_goal = point_goal(-0.064472466702,1.19911181927,0.978050418273,-0.208368374079)
    #原点
    ar_front_goal = point_goal(0.9977760314941406,-0.2532465159893036,0.8923829427859313,0.4512789419247493)

    goal = ar_front_goal
    
    rospy.loginfo("send goal success")
    ac.send_goal(goal)
    ac.wait_for_result()

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("move success")
        return 1
    else:
        rospy.loginfo("failed")

if __name__ == "__main__":
    rospy.init_node("test3")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    autoDockPub = rospy.Publisher('chatter',String,queue_size=5)

    s = input()
    rate = rospy.Rate(50)
    speed = 0.2
    move = 1
    moveTime = float(move / speed)
    tick = int(moveTime * 50)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    for i in range(tick):
        twist.linear.x = speed
        pub.publish(twist)
        rate.sleep()
    send_goal()




    

