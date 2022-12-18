#! /usr/bin/python 
# -*- coding: UTF-8 -*-

import rospy

from geometry_msgs.msg import Twist
import sys, select, termios, tty
import struct
# mqtt
import json
import sys
# 引入mqtt包
import paho.mqtt.client as mqtt
# 使用独立线程运行
from threading import Thread
import time

TYPE_JSON = 0x01

# 建立mqtt连接
broker = '106.14.145.57'
port = 1883
topic = "/control_car"
client_id = 'mq_car'
user = 'cwl'
pwd = '19260817'
KeyOnline = ''
keyKey = ''

msg = """
Control Your mrobot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

##JSON Send
body = {
    "datastreams": [
        {
            "id": "wendu",  # 对应OneNet的数据流名称
            "datapoints": [
                {
                    "at": "2016-08-15T14:47:00",  # 数据提交时间，这里可通过函数来获取实时时间
                    "value": 55  # 数据值
                }
            ]
        }
    ]
}


def build_payload(type, payload):
    datatype = type
    packet = bytearray()
    packet.extend(struct.pack("!B", datatype))
    if isinstance(payload, str):
        udata = payload.encode('utf-8')
        length = len(udata)
        packet.extend(struct.pack("!H" + str(length) + "s", length, udata))
    return packet


# JSON Send Solu
# json_body = json.dumps(body)
# packet = build_payload(TYPE_JSON, json_body) #bytes
# client.publish("$dp", packet, qos=1)  #qos代表服务质量


speed = .2
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


# MQTT
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
    global keyKey
    # json_msg = json.loads(msg.payload.decode('utf-8'))
    # 加入个人逻辑
    temp = msg.payload.decode()
    temp = str(temp)
    keyKey = temp.replace("'", '')
    print(keyKey)

    pass


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
def mqtt_publish(sensor_data, topic='xxxxxxxx', qos=2):
    global client
    try:
        client.publish(topic=topic, payload=sensor_data, qos=qos)
    except KeyboardInterrupt:
        print("EXIT")
        # 这是网络循环的阻塞形式，直到客户端调用disconnect（）时才会返回。它会自动处理重新连接。
        client.disconnect()
        sys.exit(0)


client = mqtt.Client()


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
    # MQTT服务器的端口号
    # client.connect(host=broker_ip, port=1883, keepalive=6000)
    client.connect(host=broker_ip, port=port)
    client.subscribe(topic, qos=0)
    # 创建线程去持续接收订阅信息
    subscribe_thread = Thread(target=mqtt_subscribe)
    subscribe_thread.start()


def get_key():
    key = ''
    return key


if __name__ == "__main__":
    # mqtt
    mqtt_run()
    print("test")
    # ros
    rospy.init_node('keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed, turn))
        while (1):
            # mqtt part
            keyKey = get_key()
            time.sleep(0.095)
            if keyKey != '':
                print("mqtt_control", keyKey)

            # ros part
            key = keyKey
            if key in moveBindings.keys():
                print("keyKey")
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            twist = Twist()
            # print(control_speed)
            twist.linear.x = control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn
            pub.publish(twist)

        # print("loop: {0}".format(count))
        # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
        # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)