#!/usr/bin/python
# -*- coding: utf-8 -*-
from symbol import import_from
import rospy
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#目标点
# header: 
#   seq: 2
#   stamp: 
#     secs: 1664874770
#     nsecs: 598334074
#   frame_id: ''
# goal_id: 
#   stamp: 
#     secs: 0
#     nsecs:         0
#   id: ''
# goal: 
#   target_pose: 
#     header: 
#       seq: 2
#       stamp: 
#         secs: 1664874770
#         nsecs: 596789327
#       frame_id: "map"
#     pose: 
#       position: 
#         x: -0.6847551465034485
#         y: 1.1441624164581299
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.9990359807751071
#         w: -0.04389885097266156
# ---

def sendOrigin():
    rate = rospy.Rate(50)
    speed = 0.2
    move = 0.75
    moveTime = float(move / speed)
    tick = int(moveTime * 50)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    for i in range(tick):
        twist.linear.x = speed
        cmd_pub.publish(twist)
        rate.sleep()
    #订阅move_base的活动server
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    rospy.loginfo("wait for move_base")
    #等待回应
    ac.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base")
    
    # ar_front_goal = point_goal(-0.064472466702,1.19911181927,0.978050418273,-0.208368374079)
    #原点
    # ar_front_goal = point_goal(0.9977760314941406,-0.2532465159893036,0.8923829427859313,0.4512789419247493)

    #安全点
    # ar_front_goal = point_goal(-1.101654291152954,0.8675270080566406,0.09223595067167618,0.995737178879895)

    # 正式初始点
    ar_front_goal = point_goal(2.036663055419922,-0.1254250407218933,-0.9999534913615998,0.006408842240622132)

    goal = ar_front_goal
    
    rospy.loginfo("send goal success")
    ac.send_goal(goal)
    ac.wait_for_result()

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("move success")
        return 1
    else:
        rospy.loginfo("failed")


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
    rate = rospy.Rate(50)
    speed = 0.2
    move = 0.01
    moveTime = float(move / speed)
    tick = int(moveTime * 50)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    for i in range(tick):
        twist.linear.x = speed
        cmd_pub.publish(twist)
        rate.sleep()
    #订阅move_base的活动server
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    rospy.loginfo("wait for move_base")
    #等待回应
    ac.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base")
    
    # 架构变前的点
    # ar_front_goal = point_goal(-0.064472466702,1.19911181927,0.978050418273,-0.208368374079)
    ar_front_goal = point_goal(-0.19256210327148438,1.02800452709198,-0.9839910192216099,0.17821805209129984)

    goal = ar_front_goal
    
    rospy.loginfo("send goal success")
    ac.send_goal(goal)
    ac.wait_for_result()

    if(ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("move success")
        return 1
    else:
        rospy.loginfo("failed")

def send_goal_select(msg):
    send_goal()

def send_goal_all():
    global autoDockPub
    status = send_goal()
    print(status)
    dicts = "{\"autoDock\" : \"true\"}"
    if(status):
        print("ss")
        autoDockPub.publish(String(dicts))

def deal_message(msg):
    global point
    msg = str(msg.data)
    msg = eval(msg)
    print(msg)
    try:
        if(msg["send_nav_goal"] == "all_project"):
            # point = msg["send_nav_goal"]
            send_goal_all()
            # send_goal()
        elif(msg["send_nav_goal"] == "car_back"):
            sendOrigin()
            print("start origin")
    except:
        pass

if __name__ == "__main__":
    point = ""
    rospy.init_node("send_nav_goal")
    rospy.Subscriber("/main_control",String,deal_message)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    autoDockPub = rospy.Publisher('chatter',String,queue_size=5)

    rospy.spin()
    
