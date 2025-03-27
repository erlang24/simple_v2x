import serial
import libscrc
from threading import Thread
from  v2x_serial import PmV2xSerial
import sys

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

                if "traffic_light" in info:
                    #print(info["traffic_light"])
                    # for tl_info in info["traffic_light"]:
                    #     for phase in tl_info["phase"]:
                    #         print("( id: %d - %s - %s  %.1f ) " %  
                    #             (tl_info["id"],tl_info["mode"],phase["light_text"],  phase["current_time_remaining"]/10) , end="- ")
                    # print("")
                    #formatted_str = json.dumps(info["traffic_light"], indent=4)
                    #print(formatted_str)
                    pass

                # print(info)
                if "v2x_data" in info:
                    try:
                        # print(bytearray(info["v2x_data"]).hex())
                        # my_dict = json.loads(info["v2x_data"].decode("utf-8"))
                        # print("[RECIVE] ", my_dict)
                        print(" ==================== [ RECIVE ] ===================== ")
                        import datetime
                        now = datetime.datetime.now()
                        print(now.strftime("%Y-%m-%d %H:%M:%S"))
                        print(" ----------------------------------------------------- ")
                        # print(info["v2x_data"])
                        my_new_bytes = bytes(info["v2x_data"])
                        my_new_string = str(my_new_bytes, "utf-8")
                        print(my_new_string)
                        print(" ===================================================== ")
                    except Exception as e:
                        exit(-5)
                        print(e)
            # time.sleep(0.1)

    def test_send(self):
        while True:
            input_data = input("请输入 :")
            if input_data == "" or  input_data == None:
                continue
            # if input_data == "q":
            #     sys.exit(1)

            my_string = input_data
            my_bytes = bytes(my_string, "utf-8")
            my_list = list(my_bytes)

            self.pm_v2x_serial.set_v2x_data(my_list)
            _data = self.pm_v2x_serial.get_from_serial_send_queue()
            if _data.__len__() > 0:
                self.pm_v2x_serial.v2x_send(_data)
                # print("-----------------SEND---------------")
                # print("send count:  %d" % info["index"])
                # print("-----------------------------------")
                print(" ===================== [ SEND ] ===================== ")
                import datetime
                now = datetime.datetime.now()
                print(" ", now.strftime("%Y-%m-%d %H:%M:%S"))
                print(" ----------------------------------------------------- ")
                print(" ", input_data)
                print(" ===================================================== ")

            time.sleep(0.1)

if __name__ ==  "__main__":

    serial_port = "/dev/ttyUSB0"
    if(len(sys.argv) == 1):
        input("默认使用： /dev/ttyUSB0")
    else:
        serial_port = sys.argv[1]
        input("使用： %s "%serial_port)
    v2x_test  = V2xText(serial_port)
    v2x_test.test_send()
