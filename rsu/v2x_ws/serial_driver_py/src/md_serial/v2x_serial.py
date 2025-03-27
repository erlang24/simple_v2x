import serial
import libscrc 
from queue import Queue, Empty

LIGHT_STATE_UNAVAILABLE      = 0
LIGHT_STATE_DARK	         = 1
LIGHT_STATE_FLASHING_RED     = 2
LIGHT_STATE_RED	             = 3
LIGHT_STATE_FLASHING_GREEN   = 4
LIGHT_STATE_PERMISSIVE_GREEN = 5
LIGHT_STATE_PROTECTED_GREEN  = 6
LIGHT_STATE_YELLOW	         = 7
LIGHT_STATE_FLASHING_YELLOW  = 8

class PmV2xSerial(object):
    def __init__(self) -> None:
        self.serial = None
        self.traffic_keys={}  ## id - type
        self.serial_data_queue = Queue( maxsize = 5000)        

    def open(self,serial_port="/dev/ttyUSB0",serial_baud=115200):
        try:
            self.serial = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=0.2)
            if self.serial == None:
                print("open  %s  failed" % serial_port)
                return False
            return True
        except  Exception as e :
            print(e)
            print("open  %s  failed" % serial_port)
            return False

    def close(self):
        self.serial.close()
        self.serial = None

    def set_traffic_light_state(self,state):
        if self.traffic_keys.get(state['key']) is not None:
            pass

    def header_gen(self, len):
        if not hasattr(self, "header_seq"):
            self.header_seq = 0

        header_array=[0xAB, 0x00, 0x00, 0x00, 0x00,0x00]
        header_array[1] = len%256
        header_array[2] = int(len/256)

        header_array[3] = self.header_seq%256
        header_array[4] = int(self.header_seq/256)
        self.header_seq +=1
        if self.header_seq > 160:
            self.header_seq = 0

        checksum =  0
        for byte in header_array[:-1]:
            checksum ^= byte
        header_array[5] =checksum 

        return header_array
    def request_data_gen(self):


        slaveAddress = 0x01
        startAddress =  0x000
        count =  0x0e

        request_data = bytearray()  
        request_data.append(slaveAddress)  
        request_data.append(0x04)  
        request_data.append(startAddress >> 8)
        request_data.append(startAddress & 0xFF)
        request_data.append(count >> 8)
        request_data.append(count & 0xFF)

        code_crc16 = libscrc.modbus(bytearray(request_data))
        data = request_data
        data.append(code_crc16 % 256)
        data.append(int(code_crc16/256))
        
        # print(data.hex())
        return data



    def holdingRegisters_data_gen(self,slaveAddress,startAddress,data2set):
        request_data = bytearray()
        request_data.append(slaveAddress)#(0x01)
        request_data.append(0x10)
        request_data.append(startAddress >> 8)
        request_data.append(startAddress & 0xFF)

        half_length = int(len(data2set) / 2)  
        high_byte = (half_length >> 8) & 0xFF  
        low_byte = half_length & 0xFF  

        request_data.append(high_byte)
        request_data.append(low_byte)


        request_data.append(int(len(data2set)))

        request_data += data2set

        code_crc16 = libscrc.modbus(bytearray(request_data))

        data = request_data
        data.append(code_crc16 % 256)
        data.append(int(code_crc16/256))
        print(data.hex())
        return data

    def traffic_light_playload_gen(self,header_data,light_data):
        CMD = 0x02
        checksum = 0 ^ CMD

        lightLen_Ctrl =  light_data.__len__()
        checksum ^= lightLen_Ctrl
        for byte in light_data:
            checksum ^= byte

        playload_xor = checksum

        header_data.append(playload_xor)
        header_data.append(CMD)
        header_data.append(lightLen_Ctrl)
        header_data.extend(light_data)
        return header_data
    
    def v2x_playload_gen(self,data):

        ## Add CRC16
        code_crc16 = libscrc.modbus(bytearray(data))
        # print(bytearray(data[:-2]).hex(), "%02x, %02x" % (code_crc16 % 256,int(code_crc16/256) ))
        data.append(code_crc16 % 256)
        data.append(int(code_crc16/256))

        header_data = self.header_gen(data.__len__() +1)

        CMD = 0x01
        checksum = 0 ^ CMD

        #########
        for byte in data:
            checksum ^= byte

        playload_xor = checksum

        header_data.append(playload_xor)
        header_data.append(CMD)
        header_data.extend(data)
        return header_data
 

    def set_v2x_data(self,data):
        self.push_to_serial_send_queue(self.v2x_playload_gen(data))

    def set_slave_mode(self,info):
        '''设置从机模式'''
        # 设置命令 ---  0x1: 设置丛机模式,  0x2: 设置红黄绿时间, 0x4:  强制从机器状态

        # 4）设置从机模式： 
        #               01   19    00 02     00    1F FC
        #             地址码 功能码 寄存器地址 运行模式 CRC校验
        #             模式0X00为 自动模式，0X01为调试模式

        cmd_buff=[0x01, 0x19, 0x00, 0x02, 0x00, 0x1F, 0xFC]
        cmd_buff[0] = info["id"]
        if info["slave_mode"] == "debug":
            cmd_buff[4] = 1
        code_crc16 = libscrc.modbus(bytearray(cmd_buff[:-2]))
        cmd_buff[-2] = code_crc16 % 256
        cmd_buff[-1] = int(code_crc16/256)
        # print(bytearray(cmd_buff).hex())
        header = self.header_gen(cmd_buff.__len__()+2)
        data = self.traffic_light_playload_gen(header,cmd_buff)
        # self.v2x_send(data)
        print(bytearray(data).hex())
        self.push_to_serial_send_queue(data)

    def set_traffic_light_cycle_time(self,info):
        '''预置红黄绿灯时间'''

        ##################
        ## 双面红绿灯
        ##################

        # 预置红黄绿灯时间

        # if info["phase"].__len__() == 1:

            # cmd_buff = [0x01,0x1B,0x00,0x03,0x10,0x03,0x05,0x03,0x10,0x05,0xD7,0xAD]
            # cmd_buff[0] = info["id"]

            # cmd_buff[4] = info["phase"][0]["red_times"]
            # cmd_buff[5] = info["phase"][0]["flashing_red_times"]
            # cmd_buff[6] = info["phase"][0]["yellow_times"]
            # cmd_buff[7] = info["phase"][0]["flashing_yellow_times"]
            # cmd_buff[8] = info["phase"][0]["green_times"]
            # cmd_buff[9] = info["phase"][0]["flashing_green_times"]
            # code_crc16 = libscrc.modbus(bytearray(cmd_buff[:-2]))
            # cmd_buff[-2] = code_crc16 % 256
            # cmd_buff[-1] = int(code_crc16/256)


        data2set = bytearray()

        presetTime_red_s = info["phase"][0]["red_times"]
        data2set.append((presetTime_red_s>>8) & 0xFF)
        data2set.append(presetTime_red_s & 0xFF)
        data2set.append((presetTime_red_s>>24) & 0xFF)
        data2set.append((presetTime_red_s>>16) & 0xFF)

        presetTime_green_s = info["phase"][0]["green_times"]
        data2set.append((presetTime_green_s>>8) & 0xFF)
        data2set.append(presetTime_green_s & 0xFF)
        data2set.append((presetTime_green_s>>24) & 0xFF)
        data2set.append((presetTime_green_s>>16) & 0xFF)

        presetTime_yellow_s = info["phase"][0]["yellow_times"]
        data2set.append((presetTime_yellow_s>>8) & 0xFF)
        data2set.append(presetTime_yellow_s & 0xFF)
        data2set.append((presetTime_yellow_s>>24) & 0xFF)
        data2set.append((presetTime_yellow_s>>16) & 0xFF)
        print("data2set")
        print(data2set)
        # header = self.header_gen(cmd_buff.__len__()+2)
        data = self.holdingRegisters_data_gen(0x01,0x0002,data2set)
        self.push_to_serial_send_queue(data)

        ##################
        ## 四面红绿灯
        ##################

        # 预置东西南北红黄绿寄存器时间
        # '''
        # 01     2B     00 06    10 03 05 03 10 05    10 03 05  03 10 05   F8 71
        # 地址码 功能码 寄存器地址  东西红黄绿 Time Flash   南北红黄绿 Time Flash CRC校验                

        # 说明： 红灯时间一个字节，红闪时间一个字节，黄灯时间一个字节，黄闪时间一个字节，    
        #       绿灯时间一个字节，绿闪时间一个字节。时间数据等均为十六进制表示。
        # '''
        if info["phase"].__len__() == 2:
            data2set = bytearray()

            presetTime_red_s = info["phase"][1]["red_times"]
            data2set.append((presetTime_red_s>>8) & 0xFF)
            data2set.append(presetTime_red_s & 0xFF)
            data2set.append((presetTime_red_s>>24) & 0xFF)
            data2set.append((presetTime_red_s>>16) & 0xFF)

            presetTime_green_s = info["phase"][1]["green_times"]
            data2set.append((presetTime_green_s>>8) & 0xFF)
            data2set.append(presetTime_green_s & 0xFF)
            data2set.append((presetTime_green_s>>24) & 0xFF)
            data2set.append((presetTime_green_s>>16) & 0xFF)

            presetTime_yellow_s = info["phase"][1]["yellow_times"]
            data2set.append((presetTime_yellow_s>>8) & 0xFF)
            data2set.append(presetTime_yellow_s & 0xFF)
            data2set.append((presetTime_yellow_s>>24) & 0xFF)
            data2set.append((presetTime_yellow_s>>16) & 0xFF)


            # header = self.header_gen(cmd_buff.__len__()+2)
            data = self.holdingRegisters_data_gen(0x01,0x0009,data2set)
            # data = self.traffic_light_playload_gen(header,cmd_buff)
            # self.v2x_send(data)

            # print(bytearray(data).hex())
            self.push_to_serial_send_queue(data)

    def force_set_light_one_phase(self,info):
        ##################
        ## 双面红绿灯
        ##################
        # 强置从机状态

 
        mode_data = bytearray()

        mode_data.append(0x00)
        print(info["slave_mode"])
        if info["slave_mode"] == "auto":
            mode_data.append(0x00)
        else :
            mode_data.append(0x01)
        mode_data.append(0x00)
        

        if info["phase"][0]["light"] == LIGHT_STATE_UNAVAILABLE:
            mode_data.append(0x00)
        elif info["phase"][0]["light"]  == LIGHT_STATE_DARK:
            mode_data.append(0x01)
        elif info["phase"][0]["light"] == LIGHT_STATE_FLASHING_RED:
            mode_data.append(0x02)
        elif info["phase"][0]["light"] == LIGHT_STATE_RED:
            mode_data.append(0x03)
        elif info["phase"][0]["light"] == LIGHT_STATE_FLASHING_GREEN:
            mode_data.append(0x04)
        elif info["phase"][0]["light"] == LIGHT_STATE_PERMISSIVE_GREEN:
            mode_data.append(0x05)
        elif info["phase"][0]["light"] == LIGHT_STATE_PROTECTED_GREEN:
            mode_data.append(0x06)
        elif info["phase"][0]["light"] ==  LIGHT_STATE_YELLOW:
            mode_data.append(0x07)
        elif info["phase"][0]["light"] ==  LIGHT_STATE_FLASHING_YELLOW:
            mode_data.append(0x08)
        else:
            pass
        print(mode_data)
        data = self.holdingRegisters_data_gen(0x01,0x0000,mode_data)
        self.push_to_serial_send_queue(data)

    def force_set_light_tow_phase(self,info):
        ##################
        ## 四面红绿灯
        ##################
        '''
        01    2C    01      00        04     00     10  71 A1
        地址码 功能码 运行模式 灯方向  状态寄存器 数据性质 时间 CRC校验
     
        说 明：  运行模式 :  0X01为调试模式 (仅调试模式有效）
                信号灯方向: 0X00 代表东西方向信号灯,0X01代表南北方向信号灯
                状态寄存器: 0X00代表是灯灭,0X01代表是红灯, 0X02代表是黄灯,0X04代表是绿灯
                数据性质 :  0X00为表示当前灯长亮,0X01为表示当前灯闪烁,0X02为时间自行递减
                时   间 :  0X10代表当前显示剩余时间16秒
        '''

        cmd_buff = [0x01,0x2C,0x01,0x00,0x04,0x00,0x10,0x71,0xA1]
        cmd_buff[0] = info["id"]
        cmd_buff[2] = 0
        if info["slave_mode"] == "debug":
            cmd_buff[2] = 1

        for i in range(info["phase"].__len__()):
            cmd_buff[3] = i #信号灯方向
            if info["phase"][i]["light"] == LIGHT_STATE_UNAVAILABLE:
                cmd_buff[4] = 0x00
            elif info["phase"][i]["light"] == LIGHT_STATE_DARK:
                cmd_buff[4] = 0x00
            elif info["phase"][i]["light"] == LIGHT_STATE_FLASHING_RED:
                cmd_buff[4] = 0x01
                cmd_buff[5] = 0x01
            elif info["phase"][i]["light"] == LIGHT_STATE_RED:
                cmd_buff[4] = 0x01
                cmd_buff[5] = 0x00
            elif info["phase"][i]["light"] == LIGHT_STATE_FLASHING_GREEN:
                cmd_buff[4] = 0x04
                cmd_buff[5] = 0x01
            elif info["phase"][i]["light"] == LIGHT_STATE_PERMISSIVE_GREEN:
                cmd_buff[4] = 0x04
                cmd_buff[5] = 0x00
            elif info["phase"][i]["light"] == LIGHT_STATE_PROTECTED_GREEN:
                cmd_buff[4] = 0x04
                cmd_buff[5] = 0x00
            elif info["phase"][i]["light"] ==  LIGHT_STATE_YELLOW:
                cmd_buff[4] = 0x02
                cmd_buff[5] = 0x00
            elif info["phase"][i]["light"] ==  LIGHT_STATE_FLASHING_YELLOW:
                cmd_buff[4] = 0x02
                cmd_buff[5] = 0x01
            else:
                pass

            code_crc16 = libscrc.modbus(bytearray(cmd_buff[:-2]))
            cmd_buff[-2] = code_crc16 % 256
            cmd_buff[-1] = int(code_crc16/256)

            header = self.header_gen(cmd_buff.__len__() + 2)
            data = self.traffic_light_playload_gen(header,cmd_buff)
            # self.v2x_send(data)
            print(bytearray(data).hex())
            self.push_to_serial_send_queue(data)

    def force_set_light_tow_phase_2(self,info):
        ##################
        ## 四面红绿灯
        ##################
        '''
        强置从机状态: v2.3

              01   2C    01     04 00 10        01 00 10       40A5
            地址码 功能码 运行模式 东西方状态性质时间 南北方状态性质时间 CRC校验

        读取成功：
            返回数据： 01 2C 01 04 00 10 01 00 10 40A5
                     地址码 功能码 运行模式 东西方状态性质时间 南北方状态性质时间 CRC校验
        说 明:
            运行模式  : 0X01为调试模式 (仅调试模式有效)
            状态寄存器： 0X00代表是灯灭,  0X01代表是红灯, 0X02代表是黄灯, 0X04代表是绿灯
            数据性质  : 0X00为表示当前灯长亮,0X01为表示当前灯闪烁, 0X02为时间自行递减
            时   间  :  0X10代表当前显示剩余时间16秒

        '''

        mode_data = bytearray()

        mode_data.append(0x00)
        mode_data.append(0x01)
        mode_data.append(0x00)
        

        if info["phase"][0]["light"] == LIGHT_STATE_UNAVAILABLE:
            mode_data.append(0x00)
        elif info["phase"][0]["light"]  == LIGHT_STATE_DARK:
            mode_data.append(0x01)
        elif info["phase"][0]["light"] == LIGHT_STATE_FLASHING_RED:
            mode_data.append(0x02)
        elif info["phase"][0]["light"] == LIGHT_STATE_RED:
            mode_data.append(0x03)
        elif info["phase"][0]["light"] == LIGHT_STATE_FLASHING_GREEN:
            mode_data.append(0x04)
        elif info["phase"][0]["light"] == LIGHT_STATE_PERMISSIVE_GREEN:
            mode_data.append(0x05)
        elif info["phase"][0]["light"] == LIGHT_STATE_PROTECTED_GREEN:
            mode_data.append(0x06)
        elif info["phase"][0]["light"] ==  LIGHT_STATE_YELLOW:
            mode_data.append(0x07)
        elif info["phase"][0]["light"] ==  LIGHT_STATE_FLASHING_YELLOW:
            mode_data.append(0x08)
        else:
            pass
        
        count = 0  

        while count < 13:  

            mode_data.append(0x00)  
            count += 1
        
        if info["phase"][1]["light"] == LIGHT_STATE_UNAVAILABLE:
            mode_data.append(0x00)
        elif info["phase"][1]["light"]  == LIGHT_STATE_DARK:
            mode_data.append(0x01)
        elif info["phase"][1]["light"] == LIGHT_STATE_FLASHING_RED:
            mode_data.append(0x02)
        elif info["phase"][1]["light"] == LIGHT_STATE_RED:
            mode_data.append(0x03)
        elif info["phase"][1]["light"] == LIGHT_STATE_FLASHING_GREEN:
            mode_data.append(0x04)
        elif info["phase"][1]["light"] == LIGHT_STATE_PERMISSIVE_GREEN:
            mode_data.append(0x05)
        elif info["phase"][1]["light"] == LIGHT_STATE_PROTECTED_GREEN:
            mode_data.append(0x06)
        elif info["phase"][1]["light"] ==  LIGHT_STATE_YELLOW:
            mode_data.append(0x07)
        elif info["phase"][1]["light"] ==  LIGHT_STATE_FLASHING_YELLOW:
            mode_data.append(0x08)
        else:
            pass
        data = self.holdingRegisters_data_gen(0x01,0x0000,mode_data)
        self.push_to_serial_send_queue(data)


    def force_set_traffic_light_signal(self,info):
        ##################
        ## 双面红绿灯
        ##################
        # print(info["phase"].__len__())
        if info["phase"].__len__() == 1:
            self.force_set_light_one_phase(info)

        ##################
        ## 四面红绿灯
        ##################
        if info["phase"].__len__() == 2:
            # self.force_set_light_tow_phase(info)
            pass
            self.force_set_light_tow_phase_2(info)


    def v2x_send(self,data):
        # print("[ data send to serial port ] : -> %s - len %d"%(bytearray(data).hex(),data.__len__()))
        self.serial.write(data)
        self.serial.flush()

    def push_to_serial_send_queue(self,data):
        # print("push queue  ",bytearray(data).hex())
        if not self.serial_data_queue.full():
            self.serial_data_queue.put(data)        
        else:
            print("[ ERROR ] : push queue faild , queue is full")

    def get_from_serial_send_queue(self):
        try:
            data = self.serial_data_queue.get_nowait()        
            if data == None:
                return [] 
            return data
        except Empty as e:
            return [] 

    def get_header_from_serial_port(self):
        while True:
            try:
                data = self.serial.read(1)
                if data == b'' or data == None or data.__len__() == 0 :
                    print("Serial Read Null: ",data )
                    return 0
                if data[0] == 0xAB:
                    data = self.serial.read(5)
                    if data == b'' or data == None or data.__len__() == 0 :
                        return 0
                    if data.__len__() != 5:
                        print(f"Serial Read  Len : %d" % data.__len__())
                        return 0
                    checksum = 0 ^ 0xAB
                    for byte in data[0:4]:
                        checksum ^= byte
                    if checksum == data[4]:
                        lenght = data[0] + data[1]*256
                        #print("Head : "  ,bytearray([0xab] + list(data)).hex() )
                        return lenght
                    else:
                        print("[ Head xor check error ] :  "  ,bytearray([0xab] + list(data)).hex() )
                return 0
            except serial.SerialTimeoutException as e:
                print(f"SerialTimeoutException: {e}")
                return 0
            except serial.SerialException as e:
                print(f"SerialException: {e}")
                return 0

    def get_data_from_serial_port(self):
        while True:
            try:
                data = self.serial.read()
                if data == b'' or data == None or data.__len__() == 0 :
                    return None
            except serial.SerialTimeoutException as e:
                print(f"SerialTimeoutException: {e}")
                return 0
            except serial.SerialException as e:
                print(f"SerialException: {e}")
                return 0
    def get_payload_from_serial_port(self):


        # flag = False
        # counter = 0 
        # while flag == False :
        #     counter = counter + 1
            
        #     print(flag)
            data = self.serial.read(40)
            if len(data) > 4 :
                flag = True
                print("len data")
                print(len(data))
                code_crc16 = libscrc.modbus(bytearray(data[:-2]))
                crc16test = bytearray(data[:-2])
                crc16test.append(code_crc16 % 256)
                crc16test.append(int(code_crc16/256))
                # print(crc16test)
                if (data[:-2] == crc16test[:-2]):
                    print("crc16test OK")
                    print(data)
                    return data
            # if counter > 5:
            return [0x00]
    

    def payload_pares(self,data):
        TRAFFIC_LIGHT_STATE_FLAG = 0x04
        TRAFFIC_LIGHT_SET_FLAG   = 0x02
        V2X_DATA_FLAG   = 0x01
        info = {}
        cmd =  data[0]
        if cmd  &  TRAFFIC_LIGHT_STATE_FLAG > 0:
            # print("TRAFFIC_LIGHT_STATE_FLAG"," length: %d" %data.__len__() )
            # if data.__len__() >= 51 :
            if ((data.__len__() -3) / 24) > 0 :
                traffic_data = data[3:]
                traffic_array = []
                for i in range(traffic_data.__len__()):
                    try: 
                        if (i%24) == 0:
                            traffic_light_0_data = traffic_data[i:i+24]
                            # print(traffic_light_0_data.hex())
                            traffic_array.append(traffic_light_0_data)
                    except Exception as e: 
                        pass

                info["traffic_light"] = []
                for tl_data  in traffic_array:
                    info["traffic_light"].append(self.traffic_light_pares(tl_data)) 
                return info
            return {}

        if cmd  &  TRAFFIC_LIGHT_SET_FLAG > 0:
            print("TRAFFIC_LIGHT_SET_FLAG: ")
            return {}

        if cmd  &  V2X_DATA_FLAG > 0:
            # print("[ data recive from serial port ] : -> %s - len %d"%(bytearray(data).hex(),data.__len__()))
            if data.__len__() >= (3+2):
                v2x_data = data[3:]
                # print("[ data recive from serial port ] : -> %s - len %d"%(bytearray(v2x_data).hex(),v2x_data.__len__()))

                code_crc16 = libscrc.modbus(bytearray(v2x_data[:-2]))
                # print("%02x : %02x" %(code_crc16 % 256, int(code_crc16/256)))
                if v2x_data[-2] == code_crc16 % 256 and v2x_data[-1] == int(code_crc16/256):
                    info = {"v2x_data": v2x_data[:-2]}
                    return info 
                else:
                    print("ERROR : v2x data CRC check error")
                    pass
            return {}
        return {}
    def traffic_light_pares(self,data):
        # print(data.hex())
        info = {} 

        info["data"] = bytearray(data).hex()

        info["id"] = data[0]  ## addr
        info["version"] = data[1]
        info["mode"] = "auto"    # 模式 _
        if data[2] == 0x01:
            info["mode"] = "debug"    # 模式 _

        # self.traffic_keys[info["addr"]] =  info["version"]

        sub_info = {}
        sub_info["light"] = data[5]               # 灯状态

        _maps = {
            0x01: "RED",
            0x02: "YELLOW",
            0x04: "GREEN",
        }


        sub_info["light_text"] = "UNAVAILABLE"   # 灯状态 text
        if sub_info["light"] in _maps.keys():
            sub_info["light_text"] = _maps[sub_info["light"]]   # 灯状态 text


        sub_info["current_time_remaining"] = data[6] * 256 + data[7] # 剩余时间
        sub_info["red_times"] = data[12]          # 预置时间_红灯_
        sub_info["flashing_red_times"] = data[13]
        sub_info["yellow_times"]  = data[14]
        sub_info["flashing_yellow_times"] = data[15]
        sub_info["green_times"] = data[16]
        sub_info["flashing_green_times"] = data[17]

        info["phase"] = [sub_info]


        if info["version"] == 0x02:
            sub_info = {}
            sub_info["light"] = data[9]   # 灯状态 # 0X00代表是灯灭，0X01代表是红灯, 0X02代表是黄灯，0X04代表是绿灯

            sub_info["light_text"] = "UNAVAILABLE"   # 灯状态 text
            if sub_info["light"] in _maps.keys():
                sub_info["light_text"] = _maps[sub_info["light"]]   # 灯状态 text

            sub_info["current_time_remaining"] = data[10] + data[11]*256 # 剩余时间_

            sub_info["red_times"] = data[18]          # 预置时间_红灯_
            sub_info["flashing_red_times"] = data[19]
            sub_info["yellow_times"] = data[20]
            sub_info["flashing_yellow_times"] = data[21]
            sub_info["green_times"] = data[22]
            sub_info["flashing_green_times"] = data[23]

            sub_info["red_times"] = data[18]          # 预置时间_红灯_
            sub_info["flashing_red_times"] = data[19]
            sub_info["yellow_times"] = data[20]
            sub_info["flashing_yellow_times"] = data[21]
            sub_info["green_times"] = data[22]
            info["phase"].append(sub_info)
        # print(info)
        return info

        # data = GPS.readline().strip()
        # data =GPS.read_until(bytes([0xAA,0x55]))

def test_recive():
    pre_time_cnt = int(0)      #毫秒级时间戳

    pm_v2x_serial =   PmV2xSerial()
    pm_v2x_serial.open(serial_port="/dev/ttyUSB0",serial_baud=115200)
    while True:

        import time
        import datetime
        t = time.time()
        # print (t)                       #原始时间数据
        # print (int(t))                  #秒级时间戳
        # print (int(round(t * 1000)))      #毫秒级时间戳



        ret = pm_v2x_serial.get_header_from_serial_port()
        # print(ret)
        if  ret > 0:
            print(pm_v2x_serial.get_payload_from_serial_port(ret))
        print("times ms : " , int(round(t * 1000)) -  pre_time_cnt )

        pre_time_cnt  = int(round(t * 1000))


def test_send():

    pm_v2x_serial =   PmV2xSerial()
    # pm_v2x_serial.open(serial_port="/dev/ttyUSB1",serial_baud=115200)

    info = {"slave_mode": "auto"}
    info["id"] = 0x01
    info["slave_mode"] = "debug"
    pm_v2x_serial.set_slave_mode(info)

    info["phase"] = []
    sub_info = {}
    sub_info["light"] = 5
    sub_info["flashing_red_times"] = 10  
    sub_info["red_times"] = 10
    sub_info["flashing_green_times"] = 10  
    sub_info["green_times"] = 10
    sub_info["flashing_yellow_times"] = 10
    sub_info["yellow_times"]  = 10
    info["phase"].append(sub_info)
    print("")
    pm_v2x_serial.set_traffic_light_cycle_time(info)
    print("")
    pm_v2x_serial.force_set_traffic_light_signal(info)

    info["phase"].append(sub_info)
    print("")
    pm_v2x_serial.set_traffic_light_cycle_time(info)
    print("")
    pm_v2x_serial.force_set_traffic_light_signal(info)

if __name__ ==  "__main__":
    test_recive()
    # test_send()
