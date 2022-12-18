# -*- coding: utf-8 -*-
import rospy
from tf_conversions import transformations
from math import pi
import tf
import time
from std_msgs.msg import String
rospy.init_node('getRobotPoseMap')
pub = rospy.Publisher('/ar_car_pose',String)
# class Robot:
#     def __init__(self):
#         self.lis
tfRobotListener = tf.TransformListener()
tfRobotListener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
tfARtListener = tf.TransformListener()
try:
    tfARtListener.waitForTransform('/map', '/ar_marker_1', rospy.Time(), rospy.Duration(1.0))
except:
    pass


trans=[]
transAR=[]
r = rospy.Rate(100)
r.sleep()
while(1):
    try:
        now = rospy.Time.now()
        tfRobotListener.waitForTransform('/map', '/base_link', now, rospy.Duration(1.0))
        try:
            tfARtListener.waitForTransform('/map', '/ar_marker_1', now, rospy.Duration(1.0))
        except:
            pass

        (trans, rot) = tfRobotListener.lookupTransform('/map', '/base_link', now)
        (transAR, rotAR) = tfARtListener.lookupTransform('/map', '/ar_marker_1', now)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    if(transAR.__len__()<=0):
        pub.publish(String("none"))
    elif(trans[1] >= transAR[1]):
        print("小车在右")
        pub.publish(String("right"))
    elif(trans[1] <= transAR[1]):
        print("小车在左")
        pub.publish(String("left"))

    print("car:",trans)
    # print(rot)
    try:
        print("AR:" , transAR)
    except:
        pass
    r.sleep()
