#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
# header: 
#   seq: 7
#   stamp: 
#     secs: 1669212677
#     nsecs: 496240194
#   frame_id: "map"
# pose: 
#   pose: 
#     position: 
#       x: -0.6147876381874084
#       y: 0.7928148508071899
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -0.9997998057459481
#       w: 0.020008708863003658
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
# ---

def setPose(x,y,oriZ,oriW,covariance):
    for i in range(5):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.z = oriZ
        pose_msg.pose.pose.orientation.w = oriW
        pose_msg.pose.covariance = covariance
        initPosePub.publish(pose_msg)
        print("init success")

if __name__ == "__main__":
    rospy.init_node("initPose")
    initPosePub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=10)
    time.sleep(10)
    setPose(-0.6147876381874084,0.7928148508071899,-0.9997998057459481,0.020008708863003658,[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942])
