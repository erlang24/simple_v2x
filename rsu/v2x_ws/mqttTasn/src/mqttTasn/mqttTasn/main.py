from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import UInt8MultiArray
from asn_msgs.msg import SPAT,IntersectionState,Phase,PhaseState
import asn1tools

# from mqttTasn.emq_pub import EmqPub
# from mqtt_bridge.emq_sub import EmqSub

# from asn_recive_msg
import rclpy

from rclpy.node import Node


encoded = bytes(0)

class MqttTasnMgr(Node):

    def __init__(self,node_name):
        super(MqttTasnMgr, self).__init__(node_name)


        # self._passwd     = str(self.declare_parameter('passwd', "public").value)
        # publish
        self.spat_pub = self.create_publisher(UInt8MultiArray, "/asn2mqtt", 10)
        self.publisher_ = self.create_publisher(String, '/vehicleStatues', 10)
        self.subscription2 = self.create_subscription(
             IntersectionState,
            '/intersection_state_msg',
            self.intersection_state_msg2asn,
            10)
        self.subscription2  # prevent unused variable warning

        self.subscription = self.create_subscription(
             UInt8MultiArray,
            '/mqtt_recive',
            self.vehiclePosition2msg,
            10)
        
        self.create_timer(0.05, self.timer_callback)
        self.asn = asn1tools.compile_files(['BSM.asn','MsgFrame.asn','DefTime.asn','DefPosition.asn','VehStatus.asn',
                                        'DefMotion.asn','DefAcceleration.asn','VehBrake.asn','VehSize.asn','VehClass.asn',
                                        'VehSafetyExt.asn','DefPositionOffset.asn','VehEmgExt.asn','Map.asn','MapNode.asn',
                                        'MapLink.asn','MapSpeedLimit.asn','MapLane.asn','MapPoint.asn','SPATIntersectionState.asn',
                                        'RSM.asn','SignalPhaseAndTiming.asn','RSI.asn'
                                        ], numeric_enums=True)

    def __del__(self):
        pass


    
    def intersection_state_msg2asn(self, msg):


        print(f"收到的msg:",msg)
        

        
        print("time 0")
        print(msg.phases[0].phase_states[0].current_time_remaining)
        print("time 1")
        print(msg.phases[0].phase_states[1].current_time_remaining)
        
        
# 收到的msg: asn_msgs.msg.IntersectionState(intersection_id=0, status=[], mode=False, set_cmd=0, phases=[asn_msgs.msg.Phase(id=0, phase_states=[asn_msgs.msg.PhaseState(light=7, mode=False, current_time_remaining=22, flashing_red_times=0, red_times=0, flashing_green_times=0, green_times=0, flashing_yellow_times=0, yellow_times=0, set_cmd=0), asn_msgs.msg.PhaseState(light=6, mode=False, current_time_remaining=23, flashing_red_times=0, red_times=0, flashing_green_times=0, green_times=0, flashing_yellow_times=0, yellow_times=0, set_cmd=0)])])
# time 0
# 22
# time 1
# 23



        _spat= {
        "msgCnt": 101,
        "intersections": 
            [
            {'intersectionId': {"id":msg.intersection_id},
             'status': (b'manualControlIsEnabled', 16*11),#length
             "phases": 
             [
                {"id":0,"phaseStates":[{"light":msg.phases[0].phase_states[0].light,
                                        "timing":("utcTiming",{"startUTCTime":0,
                                                               "likelyEndUTCTime":msg.phases[0].phase_states[0].current_time_remaining}
                                                  )}]},
                {"id":1,"phaseStates":[{"light":msg.phases[0].phase_states[1].light,
                                        "timing":("utcTiming",{"startUTCTime":0,
                                                               "likelyEndUTCTime":msg.phases[0].phase_states[1].current_time_remaining}
                                                  )}]}

             ]
            }
            ]
        }



        # self.intersection_state_msg_pub.publish(intersection_state_msg)
        # phase0 = intersection_state_msg.phases[0]
        # phase0.phase_states[0].light
        
        global encoded
        encoded = self.asn.encode("MessageFrame", (
                                    'spatFrame', _spat
                                ) 
                            )
    def timer_callback(self):
        global encoded
        if len(encoded) > 3:
            msg = UInt8MultiArray()
            msg.data = list(encoded)           
            self.spat_pub.publish(msg)
    def vehiclePosition2msg(self , msg):
        # (msg.data)
        # print("vehiclePosition2msg")
        #emergency
        try:
            bytes_obj = bytes(msg.data) 
            string_obj = bytes_obj.decode('utf-8')
            # print(string_obj)
            # print(msg.data.decode('utf-8'))
            vehicleStatues = String()
            vehicleStatues.data = string_obj
            self.publisher_.publish(vehicleStatues)
            # print("publist")
        except  Exception as e:
            # print("error")
            pass


def main(args=None):
    rclpy.init(args=args)
    prcocess = MqttTasnMgr("mqttTsan")
    rclpy.spin(prcocess)
    prcocess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()