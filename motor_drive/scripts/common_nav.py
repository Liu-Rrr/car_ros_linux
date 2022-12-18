#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy
import math
from ctypes import *
import time
from math import sin,cos,pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose,Quaternion,Twist,Vector3
VCI_USBCAN2 = 4
STATUS_OK = 1
wheel_radius = 0.07 #the radius of the drive wheel is 70mm
pulse_per_circle = 5000  #return 5000 pulse per circle
wheel_length = 0.446   
th = 0.0
x = 0.0
y = 0.0
class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]
class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte * 8),
                ("Reserved", c_ubyte * 3)
                ]
canDLL = cdll.LoadLibrary('/home/sigma/Desktop/catkin_ws/src/motor_drive/scripts/libcontrolcan.so')
ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_OpenDevice成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_OpenDevice出错\r\n')

# 初始0通道
vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0,0, 0x01, 0x1C, 0)  # 波特率250k，正常模式
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN1出错\r\n')

ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN1出错\r\n')

# 初始1通道
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN2 出错\r\n')
ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN2 出错\r\n')
def speed(add, speed_input):
    # 通道1发送数据
    # 速度控制
    # print(speed_input) 
    # canDLL = cdll.LoadLibrary('/home/info/catkin_ws/src/py_pkg_1/src/py_pkg_1/libcontrolcan.so')
    speed_input = speed_input * 1000  # 速度单位0.001rpm,乘以1000还原为rpm
    speed_1 = speed_input >> 24
    speed_2 = speed_input >> 16 & 0x00FF
    speed_3 = speed_input >> 8 & 0x00FF
    speed_4 = speed_input & 0x00FF
    speed = c_ubyte * 8
    speed_date = speed(3, 8, 0, speed_4, speed_3, speed_2, speed_1, 0)
    ubyte_3array = c_ubyte * 3
    b = ubyte_3array(0, 0, 0)
    vci_can_obj = VCI_CAN_OBJ(add, 0, 0, 1, 0, 0, 8, speed_date, b)  # 单次发送
    ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
    # if ret == STATUS_OK:
    #     print('CAN1通道发送成功\r\n')
    if ret != STATUS_OK:
        print('CAN1通道发送失败\r\n')


def place(add):
    # 通道1发送数据
    canDLL = cdll.LoadLibrary('/home/sigma/Desktop/catkin_ws/src/motor_drive/scripts/libcontrolcan.so')
    ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
    #if ret == STATUS_OK:
       # print('调用 VCI_StartCAN1成功\r\n')
    if ret != STATUS_OK:
        print('调用 VCI_StartCAN1出错\r\n')
    ubyte_array = c_ubyte * 8
    a = ubyte_array(1, 1, 0, 0, 0, 0, 0, 0)
    ubyte_3array = c_ubyte * 3
    b = ubyte_3array(0, 0, 0)
    vci_can_obj = VCI_CAN_OBJ(add, 0, 0, 1, 0, 0, 8, a, b)  # 单次发送
    ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
    #if ret == STATUS_OK:
        #print('CAN1通道发送成功\r\n')
    if ret != STATUS_OK:
        print('CAN1通道发送失败\r\n')
    # time.sleep(2)
    # 通道2接收数据
    a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
    ubyte_3array = c_ubyte * 3
    b = ubyte_3array(0, 0, 0)
    vci_can_obj = VCI_CAN_OBJ(add, 0, 0, 0, 0, 0, 0, a, b)  # 复位接收缓存
    ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 100, 0)
    

    # print(ret)
    while ret <= 0:  # 如果没有接收到数据，一直循环查询接收。
        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 100, 0)
    if ret > 0:  # 接收到一帧数据
        data = int(
            (list(vci_can_obj.Data)[6] << 24) + (list(vci_can_obj.Data)[5] << 16) + (list(vci_can_obj.Data)[4] << 8) +
            list(vci_can_obj.Data)[3])
        #print("data=", data)
    return data

pulse_per_circle = 5000  #return 5000 pulse per circle
def get_motor_pos(motor_flag): #get the motor position m
    pos_now = place(motor_flag)
    if pos_now > 4e9: 
        pos_now = -(2**32 - pos_now)*0.14*pi/pulse_per_circle
    else:
        pos_now = pos_now*0.14*pi/pulse_per_circle
    return pos_now
def writespeed(RobotV_,YawRate_):
    YawRate_ = -YawRate_
    right_v = RobotV_ + wheel_length*YawRate_/2
    left_v = RobotV_ - wheel_length*YawRate_/2
    # print(right_v,left_v)
    right_w = int((right_v/wheel_radius)/(pi/30)) #from rad/s  to rpm
    left_w = int((left_v/wheel_radius)/(pi/30))
    speed(1,right_w*5)
    speed(2,-left_w*5)

RobotV = 0
YawRate = 0
def callback(msg):
    global RobotV, YawRate
    RobotV = msg.linear.x
    YawRate = msg.angular.z
    # print(RobotV,YawRate)
rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
rospy.Subscriber("cmd_vel",Twist,callback)
odom_broadcaster = tf.TransformBroadcaster()

last_time = rospy.Time.now()
last_pos_right = 0
last_pos_left = 0  
r=rospy.Rate(10)
# covariance
odom_pose_covariance = (1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0,0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9)
odom_twist_covariance =(1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9)
while not rospy.is_shutdown():
    writespeed(RobotV,YawRate)
    time.sleep(0.05)
    cur_pos_right = get_motor_pos(1)
    cur_pos_left = -get_motor_pos(2)
    # print("cur_x,cur_y",current_posx,current_posy)
    cur_time = rospy.Time.now()
    dt = (cur_time - last_time).to_sec()
    vr  = (cur_pos_right - last_pos_right)/dt
    vl = (cur_pos_left - last_pos_left)/dt
    vx = (vr+vl)/2
    vth = (vr-vl)/wheel_length
    
    delta_x = vx*cos(th)*dt
    delta_y = vx*sin(th)*dt
    delta_th = vth*dt
    x += delta_x
    y -= delta_y  
   # print(x,y)
    th += delta_th
    # angle = th*180 /3.1415926
    # print("theta",angle,th)
    odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
    odom_broadcaster.sendTransform((x,y,0.),odom_quat,cur_time,"base_footprint","odom")
    # odom_broadcaster.sendTransform((x,y,0.),odom_quat,0.0,"base_footprint","odom")
    odom = Odometry()
    odom.header.stamp = cur_time
    odom.header.frame_id="odom"
    odom.pose.pose = Pose(Point(x,y,0.0),Quaternion(*odom_quat))
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx,0,0),Vector3(0,0,-vth))
    odom.pose.covariance = odom_pose_covariance
    odom.twist.covariance = odom_twist_covariance
    odom_pub.publish(odom)
    last_time = cur_time
    last_pos_right = cur_pos_right
    last_pos_left = cur_pos_left
    r.sleep()

canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
    

     
