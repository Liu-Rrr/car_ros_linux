#!/usr/bin/python
# -*- coding: utf-8 -*-
from cmath import exp
from concurrent.futures import thread
from unittest.util import strclass
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from threading import Thread
import sys
import json


# 建立mqtt连接 test
# broker = '106.52.162.156'
# port = 1882

broker = '114.132.163.111'
port = 1883

# test
subTopic = "/main_control2"
# subTopic = "/control_car"
# pubTopic = "/carControl"
client_id = 'mq_car_control'
user = 'cwl'
pwd = '19260817'
# KeyOnline=''
# keyKey=''
maxStatus = False




client = mqtt.Client(client_id)

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

# 当收到关于客户订阅的主题的消息时调用。
def on_message(client, userdata, msg):
    global keyKey,maxStatus
    # json_msg = json.loads(msg.payload.decode('utf-8'))
    # 加入个人逻辑
    temp = msg.payload.decode()
    print(temp)
    temp = json.loads(temp)
    print("mqtt send msg")
    main_control(temp)
    # print(temp)
    # print(temp['stop_move'])

    # main_control(temp)
    

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

# 启动函数
def mqtt_run():
    # 账号密码验证放到最前面
    client.username_pw_set(user, pwd)
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
    client.connect(host=broker_ip, port=port)
    client.subscribe(subTopic, qos=0)
    # 创建线程去持续接收订阅信息
    subscribe_thread = Thread(target=mqtt_subscribe)
    subscribe_thread.start()

"""
    遥控按键信息
    静音:ffe21d
    1:ff30cf
    2:ff18e7
    3:ff7a85
"""




def ros_listen_esp():
    rospy.Subscriber("/chatter",String,deal_esp_msg)
    rospy.spin()

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
        mainPub.publish(str(msg))
        rospy.loginfo("receive stop_move")

    elif(msg.get('stop_nav_goal') != None):
        mainPub.publish(str(msg))
        rospy.loginfo("receive stop_nav_goal")

    elif(msg.get('send_nav_goal') != None or msg.get('car_back') != None):
        if(msg.get('send_nav_goal') == 'send_nav_goal'):
            msg = "{ \"situate\": \"busy\",\"stepname\": \"/main_control2\" }"
            mqtt_publish(msg,'/situate')
            print("car busy")
        # mainPub.publish(msg,'/situate')
        mainPub.publish(str(msg))
        rospy.loginfo("receive send_nav_goal")

    elif(msg.get('all_project') != None):
        mainPub.publish(str(msg))
        rospy.loginfo("receive all_projec")

    else:
        print("recive error")


if __name__ == "__main__":
    rospy.init_node("main_control")
    mainPub = rospy.Publisher("/main_control",String,queue_size=25)
    ros_listen_thread = Thread(target=ros_listen_esp)
    ros_listen_thread.start()
    mqtt_run()
