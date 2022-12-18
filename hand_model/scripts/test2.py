#!/usr/bin/python
import rospy
from std_msgs.msg import Int32
from threading import Thread
status = False

def listen():
    sub = rospy.Subscriber("/test2",Int32,subTest)
    rospy.spin()

def subTest(msg):
    global status
    status = True
    print("sss")

def test():
    global status,rate
    while(1):
        print("msg")
        rate.sleep()
        if(status):
            return

if __name__ == "__main__":
    rospy.init_node("test2")
    rate = rospy.Rate(10)
    Thread(target=listen).start()
    Thread(target=test).start()
    rospy.spin()