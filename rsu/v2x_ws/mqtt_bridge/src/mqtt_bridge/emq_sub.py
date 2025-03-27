#!/usr/bin/env python3
# sub.py

import paho.mqtt.client as mqtt
import time

HOST= "192.168.2.100"
PORT = 1883
TOPIC= "mqtt_api_cmd"

from threading import Thread

class  EmqSub(Thread):
    def __init__(self,host = HOST,port = PORT,topic=TOPIC,user="admin",passwd = "public",callback=None):
        super(EmqSub, self).__init__()
        self._host = host
        self._port = port
        self._topic = topic
        self._user = user
        self._passwd = passwd

        self.__callback = callback

    def run(self):
        # super(EmqSub, self).run()
        self.client_loop()

    def client_loop(self):
        client_id = time.strftime('%Y%m%d%H%M%S',time.localtime(time.time()))
        client_id = "sub-" + client_id
        self.client = mqtt.Client(client_id)    # ClientId不能重复，所以使用当前时间
        self.client.username_pw_set(self._user, self._passwd)  # 必须设置，否则会返回「Connected with result code 4」
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self._host, self._port, 60)
        self.client.loop_forever()

    def disconnect(self):
        if self.client != None:
            self.client.disconnect()
        print("Emq sub disconnect .")

    def on_connect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe(self._topic)

    def on_message(self,client, userdata, msg):
        # print(msg.topic+" "+msg.payload.decode("utf-8"))
        if self.__callback != None :
            self.__callback(msg.payload)
        else:
            print(msg.topic+" "+msg.payload.decode("utf-8"))

if __name__ == '__main__':

    import json
    def test(msg):
        print(msg.hex())
        # print(json.loads(msg))
        # t = time.time()
        # print (t)                       #原始时间数据
        # print (int(t))                  #秒级时间戳
        # print (int(round(t * 1000)))    #毫秒级时间戳
        
    # client_loop()
    emq_sub = EmqSub( host="192.168.2.100",
                      port=1883,
                      topic = "mqtt_api_report",
                      user='v2x',
                      passwd="654321",
                      callback = test)

    emq_sub.start()
    while True:
        import time
        time.sleep(1)

    emq_sub.disconnect()

    
