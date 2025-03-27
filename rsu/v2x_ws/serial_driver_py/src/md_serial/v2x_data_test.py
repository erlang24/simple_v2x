import serial
import libscrc
from threading import Thread
from  v2x_serial import PmV2xSerial

import json
import time

class myThread(Thread):

    def __init__(self, callback ) -> None:
        super().__init__() 
        self.flag = False
        self.callback = callback
        pass
    def run(self) -> None:
        self.flag = True
        while self.flag:
            self.callback()
        return  super().run()

class V2xText():
    def __init__(self,serial_port="/dev/ttyUSB0") -> None:
        self.pm_v2x_serial =   PmV2xSerial()
        self.pm_v2x_serial.open(serial_port,serial_baud=115200)

        self.recive_thread = myThread(self.recive_func)
        self.recive_thread.start()

        self.recive_count = 0
    def recive_func(self):
        while True:
            ret = self.pm_v2x_serial.get_header_from_serial_port()
            # print(ret)
            if  ret > 0:
                info  = self.pm_v2x_serial.get_payload_from_serial_port(ret)
                if info == None:
                    print("[ Error]  : get_payload_from_serial_port return None")
                    continue

                if "traffic_light" in info:
                    #print(info["traffic_light"])
                    for tl_info in info["traffic_light"]:
                        for phase in tl_info["phase"]:
                            print("( id: %d - %s - %s  %.1f ) " %  
                                (tl_info["id"],tl_info["mode"],phase["light_text"],  phase["current_time_remaining"]/10) , end="- ")
                    print("")
                    #formatted_str = json.dumps(info["traffic_light"], indent=4)
                    #print(formatted_str)
                    pass

                # print(info)
                if "v2x_data" in info:
                    try:
                        self.recive_count += 1
                        # print(bytearray(info["v2x_data"]).hex())
                        my_dict = json.loads(info["v2x_data"].decode("utf-8"))
                        # print("[RECIVE] ", my_dict)
                        print("--------------[RECIVE] -------------- ")
                        print("[ recive_count : %d  -  %.2f %% ]" % 
                                (self.recive_count, (self.recive_count/my_dict["index"])*100))
                        import time
                        import datetime
                        t = time.time()
                        # print (t)                       #原始时间数据
                        # print (int(t))                  #秒级时间戳
                        # print (int(round(t * 1000)))      #毫秒级时间戳
                        my_dict["recv_sec"] = int(round(t * 1000))  
                        my_dict["diff_sec"] = my_dict["recv_sec"] -   my_dict["send_sec"]
                        formatted_str = json.dumps(my_dict, indent=4)
                        print(formatted_str)
                        print("------------------------------------ ")
                    except Exception as e:
                        #exit(-5)
                        print(e)
            # time.sleep(0.1)

    def test_send(self):
        info = {}  
        info["total"] = 10000  
        info["index"] = 0  

        while True:
            key = input("请输入 ‘y’ 并回车，开始发送数据:")
            if key == 'y':
                break
        while True:
            if  info["index"] < info["total"]:
                info["index"] +=1   
            else :
                break
            import time
            import datetime
            t = time.time()
            # print (t)                       #原始时间数据
            # print (int(t))                  #秒级时间戳
            # print (int(round(t * 1000)))      #毫秒级时间戳
            info["send_sec"] = int(round(t * 1000))  
            self.pm_v2x_serial.set_v2x_data(list(json.dumps(info).encode("utf-8")))

            _data = self.pm_v2x_serial.get_from_serial_send_queue()
            if _data.__len__() > 0:
                self.pm_v2x_serial.v2x_send(_data)
                # print("-----------------SEND---------------")
                # print("send count:  %d" % info["index"])
                # print("-----------------------------------")

            time.sleep(0.1)

if __name__ ==  "__main__":
    import sys

    serial_port = "/dev/ttyUSB0"
    if(len(sys.argv) == 1):
        input("默认使用： /dev/ttyUSB0")
    else:
        serial_port = sys.argv[1]
        input("使用： %s "%serial_port)
    v2x_test  = V2xText(serial_port)
    v2x_test.test_send()
