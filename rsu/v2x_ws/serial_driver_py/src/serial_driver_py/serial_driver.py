import imp
import serial

from std_msgs.msg import String
# from nmea_msgs.msg import Sentence
from asn_msgs.msg import SPAT,IntersectionState,Phase,PhaseState
# from autoware_auto_perception_msgs.msg import TrafficSignalArray,TrafficSignal, TrafficLight

from std_msgs.msg import UInt8MultiArray
from md_serial.v2x_serial import  PmV2xSerial
import struct 


import rclpy
from rclpy.node import Node


class InputRegistersPhaseState:  
    def __init__(self, data):
        format_string = "<HIII"  # Little-endian: uint16_t, uint32_t, uint32_t, uint32_t  
        # 将data信息解析为 灯光状态、开始时间、剩余时间和预设时间 。
        (self.light, self.startTime_ms, self.remainingTime_ms, self.presetTime_ms) = struct.unpack(format_string, data)  
    def get_remaining_time(self):  
        return self.remainingTime_ms / 10.0

class SerialPortMgr(Node):

    def __init__(self,node_name):
        super(SerialPortMgr, self).__init__(node_name)

        self._port = self.declare_parameter('port', "/dev/ttyUSB0").value
        self._baud = self.declare_parameter('baud', 115200).value
        ## v2.3.standard :  包含单，双面两种协议
        ## v2.3.smart :    双面种协议, 按照单面的设备
        self._traffic_light_version = self.declare_parameter('traffic_light_version', "v2.3.smart").value

        self.pm_v2x_serial =  PmV2xSerial()

        # publish
        self.intersection_state_msg_pub = self.create_publisher(IntersectionState, "/intersection_state_msg", 10)
        self.spat_pub = self.create_publisher(SPAT, "/spat_state_msg", 10)
        self.uwb_bytes_recive_pub = self.create_publisher(UInt8MultiArray,"uwb_bytes_recive", 10)
        # subscription

        self.subscription = self.create_subscription(
             IntersectionState,
            '/intersection_state_set',
            self.intersection_state_set_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription_uwb_bytes_send = self.create_subscription(
             UInt8MultiArray,
            '/uwb_bytes_send',
            self.uwb_bytes_send_callback,
            10)
 
        self.subscription_uwb_bytes_send  # prevent unused variable warning

        self.create_timer(0.1, self.timer_callback)
        self.timer_count = 0

    def timer_callback(self):
        # self.timer_count +=1
        if 1:
        # if self.timer_count % 5 == 0:
            # print("write request_data")
            # print(self.pm_v2x_serial.request_data_gen().hex())
            self.pm_v2x_serial.serial.write(self.pm_v2x_serial.request_data_gen())
            # data1 = self.pm_v2x_serial.serial.read(40)
            # print(data1)
            data = self.pm_v2x_serial.get_payload_from_serial_port()
            # print(data)
            self.publis_intersection_state_msg(data)
            self.timer_count = 0
        # if self.timer_count % 5 == 3:
        if 0:
            data =  self.pm_v2x_serial.get_from_serial_send_queue()
            # print("get_from_serial_send_queue")
            # print(data.__len__())
            if data.__len__() != 0 :
                # print(data)
                self.pm_v2x_serial.serial.write(data)
                self.pm_v2x_serial.serial.flush()
                # self.pm_v2x_serial.v2x_send(data)


    def intersection_state_set_callback(self,msg):

        mode = msg.mode  

        set_cmd = msg.set_cmd  #  1:设置丛机模式, 2:设置红黄绿时间, 4:强制从机器状态
        intersection_id = msg.intersection_id 

        self.get_logger().info("recived set mode ..")
        info = {"slave_mode": "auto"}
        info["id"] = intersection_id
        if mode == True:
            info["slave_mode"] = "set"
        # self.pm_v2x_serial.set_slave_mode(info)

        info["phase"] = []
        for phase in msg.phases:
            # phase_id_ = phase.id 
            for phase_state in phase.phase_states:
                sub_info = {}
                sub_info["light"] = phase_state.light
                sub_info["flashing_red_times"] = phase_state.flashing_red_times
                sub_info["red_times"] = phase_state.red_times
                sub_info["flashing_green_times"] = phase_state.flashing_green_times  
                sub_info["green_times"] = phase_state.green_times
                sub_info["flashing_yellow_times"] = phase_state.flashing_yellow_times
                sub_info["yellow_times"]  = phase_state.yellow_times
                info["phase"].append(sub_info)

        ## v2.3.smart :    双面种协议, 单面的设备
        ## v2.3.standard :  包含单，双面两种协议
        # if self._traffic_light_version == "v2.3.smart":
        #     if info["phase"].__len__() == 1:
        #         sub_info = {}
        #         sub_info["light"] = 0
        #         sub_info["flashing_red_times"] = 0
        #         sub_info["red_times"] = 0
        #         sub_info["flashing_green_times"] = 0  
        #         sub_info["green_times"] = 0
        #         sub_info["flashing_yellow_times"] = 0
        #         sub_info["yellow_times"]  = 0
        #         info["phase"].append(sub_info)

        if set_cmd == 0x02 : #and mode == True:
            print("set_traffic_light_cycle_time")
            self.pm_v2x_serial.set_traffic_light_cycle_time(info)

        if set_cmd == 0x04 : #and mode == True:
            print("force_set_traffic_light_signal")
            self.pm_v2x_serial.force_set_traffic_light_signal(info)
        
        # print(info)

    def uwb_bytes_send_callback(self,msg):
        self.pm_v2x_serial.set_v2x_data(list(msg.data))

    def run(self):
        try:
            # self.pm_v2x_serial.open(serial_port="/dev/ttyUSB1",serial_baud=115200)
            if self.pm_v2x_serial.open(self._port,self._baud) == False:
                self.get_logger().error("Open %s is failed" % self._port )
                raise Exception("Open %s is failed" % self._port)

            # 循环读取串口数据
            while rclpy.ok():
                rclpy.spin_once(self)
            self.pm_v2x_serial.close()
        except Exception as ex:
            # 处理异常
            self.get_logger().error(str(ex))

    def publis_spat_state_msg(self,tl_info_array):
        spat_msg = SPAT()
        for info in tl_info_array:
            intersection_state_msg = IntersectionState() 
            intersection_state_msg.intersection_id = info["id"]

            intersection_state_msg.mode = False
            if info["mode"] == "debug":    # 调试模式
                intersection_state_msg.mode = True

            for i in range(info["phase"].__len__()):
                phase = Phase() # 交通灯
                phase.id = i
                phase_state = PhaseState()

                #/********* 交通灯状态***********/
                if info["phase"][i]["light"] == 0x01:
                    phase_state.light = PhaseState.LIGHT_STATE_RED

                if info["phase"][i]["light"] == 0x02:
                    phase_state.light = PhaseState.LIGHT_STATE_YELLOW
                if info["phase"][i]["light"] == 0x04:
                    phase_state.light = PhaseState.LIGHT_STATE_PERMISSIVE_GREEN # 绿灯状态

                phase_state.current_time_remaining = info["phase"][i]["current_time_remaining"]

                phase_state.flashing_red_times   = info["phase"][i]["flashing_red_times"] 
                phase_state.red_times            = info["phase"][i]["red_times"] 
                phase_state.flashing_green_times = info["phase"][i]["flashing_green_times"] 
                phase_state.green_times          = info["phase"][i]["green_times"] 
                phase_state.flashing_yellow_times= info["phase"][i]["flashing_yellow_times"] 
                phase_state.yellow_times         = info["phase"][i]["yellow_times"] 

                phase.phase_states.append(phase_state)
                intersection_state_msg.phases.append(phase)
                spat_msg.intersections.append(intersection_state_msg)

                ## v2.3.smart :    双面种协议, 单面的设备
                ## v2.3.standard :  包含单，双面两种协议
                if self._traffic_light_version == "v2.3.smart":
                    break


        self.spat_pub.publish(spat_msg)

    def publis_intersection_state_msg(self,data):
        # import json
        # formatted_str = json.dumps(info, indent=4)
        # print(formatted_str)

        intersection_state_msg = IntersectionState() 
        intersection_state_msg.intersection_id = 27
        
        if len(data) >= struct.calcsize("<HIII"):  
            phase = Phase()
            phase.id = 6
            phase_state = PhaseState()
            start_offset = 3
            data_to_parse = data[start_offset:start_offset+14]
            # print(data_to_parse)
            light = data_to_parse[0] * 256  + data_to_parse[1]
            if light ==  3:
                phase_state.light = PhaseState.LIGHT_STATE_RED
            if light ==  6:
                phase_state.light = PhaseState.LIGHT_STATE_PROTECTED_GREEN
            if light ==  7:
                phase_state.light = PhaseState.LIGHT_STATE_YELLOW
            # print(data_to_parse[2] * 256 + data_to_parse[3]  + data_to_parse[4] * 256 * 256 * 256 + data_to_parse[5] * 256 * 256 )
            phase_state.current_time_remaining = (data_to_parse[6] * 256 + data_to_parse[7]  + data_to_parse[8] * 256 * 256 * 256 + data_to_parse[9] * 256 * 256 )
            # print(data_to_parse[10] * 256 + data_to_parse[11]  + data_to_parse[12] * 256 * 256 * 256 + data_to_parse[13] * 256 * 256 )
            # data_to_parse = data[start_offset:start_offset+14]
            # state = InputRegistersPhaseState(data_to_parse)
    
            #     # LightState_protected_green
            # phase_state.current_time_remaining = state.remainingTime_ms 
            phase.phase_states.append(phase_state)


            phase_state1 = PhaseState()
            start_offset1 = 17
            data_to_parse1 = data[start_offset1:start_offset1+14]
            light1 = data_to_parse1[0] * 256  + data_to_parse1[1]
            if light1 ==  3:
                phase_state1.light = PhaseState.LIGHT_STATE_RED
            if light1 ==  6:
                phase_state1.light = PhaseState.LIGHT_STATE_PROTECTED_GREEN
            if light1 ==  7:
                phase_state1.light = PhaseState.LIGHT_STATE_YELLOW
            # print(data_to_parse[2] * 256 + data_to_parse[3]  + data_to_parse[4] * 256 * 256 * 256 + data_to_parse[5] * 256 * 256 )
            phase_state1.current_time_remaining = (data_to_parse1[6] * 256 + data_to_parse1[7]  + data_to_parse1[8] * 256 * 256 * 256 + data_to_parse1[9] * 256 * 256 )
            phase.phase_states.append(phase_state1)
            intersection_state_msg.phases.append(phase)
            self.intersection_state_msg_pub.publish(intersection_state_msg)
            # phase0 = intersection_state_msg.phases[0]
            # phase0.phase_states[0].light
            print("publis_intersection_state_msg")
        

    def publish_uwb_bytes_recive(self,data):
        try:
            _uint_barray =  UInt8MultiArray
            _uint_barray.data = data
            self.uwb_bytes_recive_pub.publish(_uint_barray)
        except Exception as ex:
            # 处理异常
            print("Excepitin : ", ex)
            self.get_logger().error(str(ex))

def main(args=None):

    try:
        rclpy.init(args=args)
        prcocess = SerialPortMgr("v2x_serial_driver")
        prcocess.run()
        # rclpy.spin(prcocess)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        prcocess.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print (e)



 
