# -*- coding: utf-8 -*-
from concurrent.futures import thread
from hashlib import new
from pydoc import doc
import rospy
import requests
from geometry_msgs.msg import Twist
# import sub
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time
from ar_track_alvar_msgs.msg import AlvarMarkers

import paho.mqtt.client as mqtt
from threading import Thread
import sys
import signal
from geometry_msgs.msg import Pose2D,Twist
import tf
import inspect
import ctypes
import json
from multiprocessing.dummy import Pool
from logger import logger
import math

current_pose = Pose2D()

PI = 3.14159265358979323846


# broker = '106.52.162.156'
broker = '114.132.163.111'
port = 1883

subTopic = "/control_car"
# subTopic = "/mai
# n_control2"

pubTopic = "/carControl"
client_id = 'main_control2'
user = 'cwl'
pwd = '19260817'
KeyOnline=''
keyKey=''
maxStatus = False


ar_pose = Pose2D()
ar_pose_origin = AlvarMarkers()

client = mqtt.Client(client_id)

# def on_success(r: Response):
#     if r.status_code == 200:
#         print(f'Post succeed: {r}')
#     else:
#         print(f'Post failed: {r}')

# def on_error(ex: Exception):
#     print(f'Post requests failed: {ex}')
pool = Pool(10) 

#对接状态量
moveRadiusState = False
moveCircleTanState = False
moveLineStatus = False
moveArStauts = False
moveCircleRetanRunStatus = False
def on_connect(client, userdata, flag, rc):
    if rc == 0:
        # 连接成功
        print("Connection successful")
    elif rc == 1:
        # 协议版本错误
        print("Protocol version error")
    elif rc == 2:
        # 无效的客户端标识
        print("Invalid client identity")
    elif rc == 3:
        # 服务器无法使用
        print("server unavailable")
    elif rc == 4:
        # 错误的用户名或密码
        print("Wrong user name or password")
    elif rc == 5:
        # 未经授权
        print("unaccredited")
    print("Connect with the result code " + str(rc))


# 当与代理断开连接时调用
def on_disconnect(client, userdata, rc):
    #  rc == 0回调被调用以响应disconnect（）调用

    # 如果以任何其他值断开连接是意外的，例如可能出现网络错误。

    if rc != 0:
        print("Unexpected disconnection %s" % rc)


# JSON
# # 当收到关于客户订阅的主题的消息时调用。

# def on_message(client, userdata, msg):

#     print(msg.topic + " " + str(msg.payload))

#     json_msg = json.loads(msg.payload.decode('utf-8'))


#     pass

# 当收到关于客户订阅的主题的消息时调用。
def on_message(client, userdata, msg):
    global keyKey,maxStatus
    # json_msg = json.loads(msg.payload.decode('utf-8'))
    # 加入个人逻辑
    print(client)
    print(userdata)
    temp = msg.payload.decode()
    temp = json.loads(temp)
    print("mqtt send msg")
    main_control(temp)


# 当使用使用publish()发送的消息已经传输到代理时被调用。

def on_publish(client, obj, mid):
    print("on_Publish, mid: " + str(mid))


# 当代理响应订阅请求时被调用
def on_subscribe(client, userdata, mid, granted_qos):
    print("on_Subscribed: " + str(mid) + " " + str(granted_qos))



# 当代理响应取消订阅请求时调用。
def on_unsubscribe(client, userdata, mid):
    print("on_unsubscribe, mid: " + str(mid))


# 当客户端有日志信息时调用
def on_log(client, obj, level, string):
    print("on_Log:" + string)


# mqtt订阅启动函数

def mqtt_subscribe():
    global client
    client.loop_forever()


# mqtt发布启动函数
def mqtt_publish(sensor_data, pubTopic='/carControl', qos=2):
    global client
    try:
        client.publish(topic=pubTopic, payload=sensor_data, qos=qos)
    except KeyboardInterrupt:
        print("EXIT")
        # 这是网络循环的阻塞形式，直到客户端调用disconnect（）时才会返回。它会自动处理重新连接。
        client.disconnect()
        sys.exit(0)

def maxControl():
    global maxStatus,pub
    while(True):
        if(maxStatus):
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)
        # e

def normalControl(msg):
    mapper={
        "left_up" : 'u',
        "up" : 'i',
        "right_up" : 'o',
        "left" : 'j',
        "stop" : 'k',
        "right" : 'l',
        "left_down" : 'm',
        "down" : ',' ,
        "riht_down" : '.'
    }

    try:
        mqtt_publish(mapper[msg])
        print(mapper[msg])
    except:
        print("expect")


# 启动函数
def mqtt_run():
    # 账号密码验证放到最前面
    # client.username_pw_set(user, pwd)
    # client = mqtt.Client()
    # 建立mqtt连接
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    # 绑定 MQTT 服务器地址
    broker_ip = broker
    print(broker_ip)
    # MQTT服务器的端口号
    # client.connect(host=broker_ip, port=1883, keepalive=6000)
    client.connect(host=broker_ip, port=port,keepalive=600)
    client.subscribe(subTopic, qos=0)
    client.subscribe('/check', qos=0)
    client.message_callback_add('/check',checkTopic)

def checkTopic(client, userdata, msg):
    msg = "{ \"situate\": \"ready\",\"stepname\": \"/main_control2\" }"
    mqtt_publish(msg,'/situate')
    print("check success")

#jieshou ros
def callbackRos(rosInfor):
    global maxStatus
    if(rosInfor.data == "whileStop"):
        maxStatus = True
    elif(rosInfor.data == "continue"):
        maxStatus = False
    elif(rosInfor.data == "circleReTan"):
        # print("右转90度")
        # # moveCircle(rosInfor.data)
        # MoveRadiustag(rosInfor.data)
        # print("odom转动")
        # odom_ar_car_radius()
        print("line")
        # moveAr()
        # moveLine(0.5)
        dock()
        # moveCircleTanRun
        # current_th = current_pose.theta
        # th = cal_angles_th(current_th, -1.5707963267948966)
        # moveCircleRetanRun(th)

    elif(rosInfor.data == "circleTan"):
        print("投料")
        moveTouliao()
        # moveCircle(rosInfor.data)

    else:
        normalControl(rosInfor.data)
    print(rosInfor.data)
        

def deal_esp_msg(msg):
    msg = msg.data
    print(msg)
    try:
        msg = json.loads(msg)
        main_control(msg)        
    except:
        print('analysis error')
        
def main_control(msg):
    global mainPub
    
    if(msg.get('stop_move') != None ):
        mqtt_stop()
        rospy.loginfo("receive stop_move")
    elif(msg.get('autoDock') != None):
        dock()
        rospy.loginfo("dock")
    else:
        print("recive error")


def loggerInto(array):
    logger.info(array)
    # print("jilu")
    print("记录")

def ros_listen():
     rospy.Subscriber('chatter', String, deal_esp_msg)
     rospy.spin()

#单纯计算x,y角度
def cal_ar_randius():
    global ar_pose
    x = ar_pose.x
    y = ar_pose.y
    radius = math.atan2(y,x)
    print(radius)
    return radius

#计算预先转动的角度，弧度制
def cal_angles_xy(current_th,x,y):
    #x与y是在ros坐标轴下的
    angles = math.atan(y/x)
    calMove = current_th + angles
    calMoveCar = cal_Target_angles(calMove)
    print("旋转",calMoveCar)

#计算预先转动的角度，弧度制
def cal_angles_th(current_th,angles):
    #x与y是在ros坐标轴下的
    # angles = math.atan(y/x)
    print("当前偏转角：",current_th)
    print("需要转动角度:",angles)
    calMove = current_th + angles
    print("初步累加",calMove)
    calMoveCar = cal_Target_angles(calMove)
    print("旋转",calMoveCar)
    return calMoveCar


def cal_Target_angles(angles):
    # target_th = 
    if( angles > PI):
        angles = -PI - (- (angles - PI))
    elif( -PI > angles):
        angles =  PI - (-angles - PI)
    
    print("最终转到",angles)
    return angles

def get_odom_pose(msg):

    global current_pose

    currentx=msg.pose.pose.position.x
    currenty=msg.pose.pose.position.y
    currentz=msg.pose.pose.position.z

    #四元素与欧拉角进行转换
    odom_or= tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    current_th=odom_or[2]
    # tf.transformations.
    #将信息进行封装
    current_pose.x = currentx
    current_pose.y = currenty
    current_pose.theta = current_th
    
    # print("偏航角",current_pose.theta)
    # print("欧拉角",odom_or)

def odom_listen():
    rospy.Subscriber("/odom", Odometry, get_odom_pose)
    rospy.spin()

def moveCircle(msg):
    global current_pose
    current_th = current_pose.theta
    tan = 1.542726251608995
    reTan = -1.5707963267948966
    if(msg == "circleReTan"):
        moveCircleRetanRun(cal_angles_th(current_th, reTan))
        print("计算后的角度",cal_angles_th(current_th, reTan))
    else:
        moveCircleTanRun(cal_angles_th(current_th, tan))
        print(cal_angles_th(current_th, tan))

def odom_ar_car_radius():
    global current_pose,ar_pose
    current_th = current_pose.theta
    carX = current_pose.x
    carY = current_pose.y
    arX = ar_pose.x
    arY = ar_pose.y

    x = abs(carX - arX)
    y = abs(carY - arY)

    degrees = math.atan2(x,y)
    print(degrees)
    angles = math.degrees(degrees)
    print("odom中两个相对角度",angles)
    # moveCircleTanRun(cal_angles_th(current_th, degrees))


def MoveRadiustag():
    global current_pose,moveRadiusState,moveCircleTanState
    current_th = current_pose.theta
    print(current_th)
    tan = cal_ar_randius()
    reTan = cal_ar_randius()
    if(reTan < 0):
        moveCircleRetanRun(cal_angles_th(current_th, reTan))
        print(cal_angles_th(current_th, reTan))
    elif(tan > 0):
        moveCircleTanRun(cal_angles_th(current_th, tan))
        print(cal_angles_th(current_th, tan))
    moveRadiusState = True
    moveCircleTanState = False
    moveCircleRetanRunStatus = False

def moveCircleTanRun(angles):
    print("左转")
    global pub,current_pose,moveCircleTanState
    twist = Twist()
    #精度太高有估算不出来
    if(abs(angles - current_pose.theta) > 1.5707963267948):
        print("越界")
        print("里程计的角度",math.degrees(current_pose.theta))
        print("正在旋转,绝对值越界")
        while(current_pose.theta > angles ):
            print(angles)
            twist.linear.x = 0
            twist.angular.z = 0.3
            pub.publish(twist)
        while(current_pose.theta < (angles - 0.065)):
            twist.linear.x = 0
            twist.angular.z = 0.3
            pub.publish(twist)
    else:
        print("里程计的角度",math.degrees(current_pose.theta))
        print("正在旋转")
        while(current_pose.theta < (angles - 0.065)):
            twist.linear.x = 0
            twist.angular.z = 0.3
            pub.publish(twist)

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    moveCircleTanState = True

def moveCircleRetanRun(angles):
    print("右转")
    global pub,current_pose,moveCircleRetanRunStatus
    twist = Twist()
     #精度太高有估算不出来
    if(abs(angles - current_pose.theta) > 1.5707963267948966):
        print("越界")
        print(abs(angles - current_pose.theta))
        #抵消信号延迟带来的角度偏转
        while(current_pose.theta < (angles - 0.040) ):
            # print("正在旋转,绝对值越界")
            twist.linear.x = 0  
            twist.angular.z = -0.3
            pub.publish(twist)
    else:
        while(current_pose.theta > (angles + 0.040)):
            # print("正在旋转")
            twist.linear.x = 0
            twist.angular.z = -0.3
            pub.publish(twist)

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    moveCircleRetanRunStatus = True

def ar_listen():
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, get_ar_pose)
    rospy.spin()

def get_ar_pose(msg):
    global ar_pose,ar_pose_origin
    if(len(msg.markers)>0):
        # ar_track
        ar_pose.x=msg.markers[0].pose.pose.position.x
        ar_pose.y=msg.markers[0].pose.pose.position.y
        # ar_pose.z=msg.markers[0].pose.pose.position.z

        ar_pose_origin = msg.markers[0].pose.pose
        # target_x = x
        # target_y = y
        # print("x轴",(msg.markers[0].pose.pose.position.x))
        # print("y轴",(msg.markers[0].pose.pose.position.y))
        # print("z轴",(msg.markers[0].pose.pose.position.z))

        # print("id号：",msg.markers[0].id)
        # print("数量：", len(msg.markers))
#自动对接接口
def dock():
    #定义全局变量，以及下面各流程状态，一般不需要添加，每个流程执行完后即可运行，但是会出意外，没执行一个动作就开始下一步
    global ar_pose_origin,moveRadiusState,moveCircleTanState,moveLineStatus,current_pose,moveArStauts,moveCircleRetanRunStatus
    print(ar_pose_origin)
    # url = "http://114.132.163.111:8991/yang/planone"
    # data = {"key":"value"}
    # pool.apply_async(requests.get, args=[url])
    # {"situate":"finish","stepname":"/casting_publish"}
    msg = "{ \"situate\": \"busy\",\"stepname\": \"/main_control2\" }"
    mqtt_publish(msg,'/situate')

    # Thread(target=http_send, args=["http://114.132.163.111:8992/yang/mixing_publish"]).start()
    #朝向二维码
    time.sleep(0.5)

    MoveRadiustag()
    #延迟是为了小车一卡一卡
    time.sleep(0.5)
    if(moveRadiusState):
        #朝向完成后就向前移动
        moveAr()
    # time.sleep(0.5)
    #移动完成后
    if(moveArStauts):
        #二次矫正
        MoveRadiustag()
        time.sleep(1)
    # MoveRadiustag()

    #moveRadiusState 
    if(moveRadiusState):
        # ar_track
        # 计算平均值，解决抖动问题，也可用卡尔曼滤波来进行
        moveTheraArr = []
        for i in range(7500):
            odom_or= tf.transformations.euler_from_quaternion([0,0, ar_pose_origin.orientation.y, ar_pose_origin.orientation.z, ar_pose_origin.orientation.w])
            current_th=odom_or[2]
            # print(math.degrees(current_th))
            # print(abs(math.degrees(current_th))-90)
            moveTheraArr.append(abs(math.radians(abs(math.degrees(current_th))-90)))
        # print(moveTheraArr)
        averge = sum(moveTheraArr) / len(moveTheraArr)
        print(averge)
        moveRadiusState = False
        # print(moveTheraArr)
        print("平均值角度",math.degrees(averge))
        print("平均值",averge)
        if(averge < 0.1):
            MoveRadiustag()
            time.sleep(3)
            moveTheraArr = []
            for i in range(15000):
                odom_or= tf.transformations.euler_from_quaternion([0,0, ar_pose_origin.orientation.y, ar_pose_origin.orientation.z, ar_pose_origin.orientation.w])
                current_th=odom_or[2]
                # print(math.degrees(current_th))
                # print(abs(math.degrees(current_th))-90)
                moveTheraArr.append(abs(math.radians(abs(math.degrees(current_th))-90)))
            # print(moveTheraArr)
            # Thread(target=loggerInto, args=[moveTheraArr]).start()

            averge = sum(moveTheraArr) / len(moveTheraArr)
        print("开始记录")
        loggerInto(moveTheraArr)

        current_th = current_pose.theta
        
    
        #用于计算小车旋转后要移动的距离,相对的x，这里的x是与二维码平行的距离
        # moveX = math.cos(ar_pose_origin.position.x)
        # 相机到小车的距离camera_car_distance = 0.364
        camera_car_distance_delay = 0.358
        moveX = (ar_pose_origin.position.x + camera_car_distance_delay)* math.cos(averge)
        print("距离二维码",ar_pose_origin.position.x)
        print("moveX: " + str(moveX))
        moveY = math.sin(ar_pose_origin.position.y)
        th = cal_angles_th(current_th, averge)
        print("th：",th)
        #当摄像头
        if(th < 0 or th > 0.2):
            #旋转角度，与坐标水平
            moveCircleTanRun(th)
            print("平行")
            time.sleep(0.5)
        
        else:
            #继续矫正
            MoveRadiustag()
            time.sleep(0.5)
        
        #旋转完成后
        if(moveCircleTanState or moveCircleRetanRunStatus):
            #-0.364相机距底盘中心位置
            # moveX = moveX - 0.23
            # moveX
            print("旋转后移动",moveX)
            # print("旋转前要移动y",moveY)
            #移动距离至与二维码垂直
            moveLine(-moveX)
            moveRadiusState = False
            if(moveLineStatus):
                print("开始朝向二维码")
                current_th = current_pose.theta
                th = cal_angles_th(current_th, -1.5707963267948966)
                moveCircleRetanRun(th)
                #矫正
                time.sleep(0.7)
                MoveRadiustag()
                time.sleep(0.5)
                if(moveCircleRetanRunStatus):
                    #再次矫正姿态
                    MoveRadiustag()
                    moveCircleRetanRunStatus = False
                time.sleep(0.3)
                #安全距离
                distance_ar = ar_pose_origin.position.x
                car_line = 0.67
                distance = distance_ar + 0.54
                # moveLine(distance)
                #距离ar一个安全距离停止
                moveTouliao()
                time.sleep(0.5)
                MoveRadiustag()
                # #矫正
                # MoveRadiustag()
                # time.sleep(0.3)
                # MoveRadiustag()
                rospy.loginfo("dock end")



    moveRadiusState = False
    moveCircleTanState = False  
    moveLineStatus = False
    moveArStauts = False
    moveCircleRetanRunStatus = False
    # msg = "{ \"situate\": \"busy\",\"stepname\": \"/car_finish\" }"
    # mqtt_publish(msg,'/situate')
    # msg = "{ \"windswitch\": \"1\" }"
    # mqtt_publish(msg,'/mixing_publish')
    # msg = "{ \"situate\": \"busy\",\"stepname\": \"/winding_publish\" }"
    
    msg = "{ \"situate\": \"finish\",\"stepname\": \"/main_control2\" }"
    mqtt_publish(msg,'/situate')
    # Thread(target=http_send, args=["http://114.132.163.111:8992/yang/planone"]).start()
    # time.sleep(1)
    # mqtt_publish(msg,'/situate')

#相机距离车体前身大概67cm
def moveAr():
    global current_pose,ar_pose,pub,moveArStauts
    twist = Twist()
    targetX = ar_pose.x
    targetY = ar_pose.y
    currentX = current_pose.x
    currentY = current_pose.y

    targetAll = currentX + targetX

    while(0.7999771031815268 < ar_pose.x):
        # print("前进")
        twist.linear.x = -0.2
        pub.publish(twist)
    

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    moveArStauts = True

    #0.7261553300074949

def moveTouliao():
    global current_pose,ar_pose,pub,moveArStauts
    twist = Twist()
    targetX = ar_pose.x
    targetY = ar_pose.y
    currentX = current_pose.x
    currentY = current_pose.y

    targetAll = currentX + targetX

    #+0.16为调试
    #0.30为未搭建
    while(0.30 + 0.10 < ar_pose.x):
        # print("前进")
        twist.linear.x = -0.2
        pub.publish(twist)
    

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    moveArStauts = True

def moveLine(distance):
    global current_pose,pub,moveLineStatus
    twist = Twist()

    currentX = current_pose.x
    currentY = current_pose.y

    targetAll = currentY + distance

    print("需要直行",distance)
    print("当前位置",currentY)
    print("移动到",targetAll)
    while(current_pose.y > targetAll):
        # print("前进")
        twist.linear.x = -0.2
        pub.publish(twist)
    

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    moveLineStatus = True


def quit(signum, frame):
    print('exit')
    global pub,subscribeRos_thread,subscribeRos_thread4
    print(subscribeRos_thread)
    stop_thread(subscribeRos_thread)
    stop_thread(subscribeRos_thread4)

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    for i in range(1000):
        pub.publish(twist)


    sys.exit()


 
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")
 
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

def mqtt_stop():
    global subscribe_thread5
    print("mqtt")
    stop_thread(subscribe_thread5)
    time.sleep(5)
    Thread(target=mqtt_subscribe).start()

def http_send(url):
    res = requests.get(url=url)
    print(res.text)


if __name__ == "__main__":
    rospy.init_node('carControl')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    global subscribeRos_thread
    try:
        signal.signal(signal.SIGINT, quit)
        signal.signal(signal.SIGTERM, quit)

        subscribeRos_thread = Thread(target=ros_listen)
        subscribeRos_thread.start()

        subscribeRos_thread2 = Thread(target=odom_listen)
        subscribeRos_thread2.start()

        subscribeRos_thread3 = Thread(target=ar_listen)
        subscribeRos_thread3.start()
        # listen()
        mqtt_run()
        # 创建线程去持续接收订阅信息
        subscribe_thread5 = Thread(target=mqtt_subscribe)
        subscribe_thread5.start()
    except Exception as exc:
        print("esc")