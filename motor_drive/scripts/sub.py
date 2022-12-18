from paho.mqtt import client as mqtt_client

broker = '47.113.192.157'
port = 1883
topic = "/python/mqtt"
client_id = 'mq_car'
user = 'mq_car'
pwd = '123456'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(user, pwd)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client):
    def on_message(client, userdata, msg):
        print("Received {} from {} topic".format(msg.payload.decode(), topic))

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()