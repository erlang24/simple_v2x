#!/usr/bin/env python3


import sys
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

import time

# HOST = "127.0.0.1"
HOST= "192.168.2.100"
PORT = 1883
TOPIC= "mqtt_api_report"

class EmqPub(object):

    def __init__(self, host = HOST ,port = PORT,topic=TOPIC,user="admin",passwd = "public"):
        self.__host = host 
        self.__port = port
        self._topic = topic
        self._user = user
        self._passwd = passwd

        self.client_id = time.strftime('%Y%m%d%H%M%S',time.localtime(time.time()))
        self.client_id  = "pub-" + self.client_id

        # self.client = mqtt.Client(self.client_id)    # ClientId不能重复，所以使用当前时间
        # self.client.username_pw_set("jxm", "public")  # 必须设置，否则会返回「Connected with result code 4」
        # self.client.on_connect = self.on_connect
        # self.client.on_message = self.on_message
        # self.client.connect(self.__host, self.__port, 60)

    # def on_connect(self,client, userdata, flags, rc):
    #     print("Connected with result code "+str(rc))
    #     client.subscribe("mqtt_test_api")

    # def on_message(self,client, userdata, msg):
    #     print(msg.topic+" "+msg.payload.decode("utf-8"))

    def publish(self,info): 
        # print("send -> ", info )
        publish.single(self._topic , info, qos = 1,hostname=self.__host,port=self.__port,
                    client_id=self.client_id,auth = { 'username': self._user,
                                                      'password': self._passwd})
        # self.client.publish("test", info, qos=0, retain=False)  # 发布消息

if __name__ == '__main__':

        __info={}
        __info["steer_cmd"] = {}
        __info["steer_cmd"]["steering_angle"] =  12.3

        __info["brake_cmd"] = {}
        __info['brake_cmd']['pedal_command'] = 45 

        __info["accel_cmd"] = {}
        __info['accel_cmd']['speed_command'] = 0  
        __info['accel_cmd']['pedal_command'] = 5 

        __info["gear_cmd"]  = {}
        __info['gear_cmd']['gear'] = 4  
        #connver_to_jetson
        # emq_send
        import json
        __info_str = json.dumps(__info)

        # print(__info)
        # self.trigger.emit(self.__info.copy())
        emp_pub = EmqPub(   host="192.168.2.100",
                            port=1883,
                            topic = "mqtt_api_cmd",
                            user='v2x',
                            passwd="654321")
        emp_pub.publish(__info_str)

        # import time
        # import datetime
        # t = time.time()
        # # print (t)                       #原始时间数据
        # # print (int(t))                  #秒级时间戳
        # print (int(round(t * 1000)))    #毫秒级时间戳

