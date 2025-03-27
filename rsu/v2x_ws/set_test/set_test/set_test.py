from asn_msgs.msg import SPAT,IntersectionState,Phase,PhaseState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class TRLightMgr(Node):

    def __init__(self,node_name):
        super(TRLightMgr, self).__init__(node_name)
        self.intersection_state_msg_pub = self.create_publisher(IntersectionState, "/intersection_state_set", 10)
        self.light = "auto" 
        # self.create_timer(1, self.set_traffic_light_cycle_time)
        self.vehicleStatues_sub = self.create_subscription(String, '/vehicleStatues',self.vehiclePassCtrl,10)
    def force_set_light_one_phase(self,mode):
        intersection_state_msg = IntersectionState() 
        intersection_state_msg.mode = mode
        intersection_state_msg.set_cmd =  0x04
        intersection_state_msg.intersection_id = 0
        phase = Phase()
        phase.id = 0
        phase_state = PhaseState()

        phase_state.light = PhaseState.LIGHT_STATE_PROTECTED_GREEN
        phase_state.red_times = 10
        phase_state.green_times = 15
        phase_state.yellow_times = 8
        phase_state.current_time_remaining = 11
        phase.phase_states.append(phase_state)
        intersection_state_msg.phases.append(phase)
        self.intersection_state_msg_pub.publish(intersection_state_msg)
        print("publis_intersection_state_msg")


    def set_traffic_light_cycle_time(self):
        intersection_state_msg = IntersectionState() 
        intersection_state_msg.mode = True
        intersection_state_msg.set_cmd =  0x02
        intersection_state_msg.intersection_id = 0
        phase = Phase()
        phase.id = 0
        phase_state = PhaseState()

        phase_state.light = PhaseState.LIGHT_STATE_RED
        phase_state.red_times = 25
        phase_state.green_times = 15
        phase_state.yellow_times = 3
        phase_state.current_time_remaining = 11
        phase.phase_states.append(phase_state)
        intersection_state_msg.phases.append(phase)
        self.intersection_state_msg_pub.publish(intersection_state_msg)
        print("publis_intersection_state_msg")
    def vehiclePassCtrl(self, msg):
        # print(msg.data)
        setting_list = msg.data.split("/")
        print(setting_list)
        if setting_list[0] == "emergency":
            print(self.light)
            if (float(setting_list[2])**2 + float(setting_list[3])**2 +float(setting_list[4])**2) < 10 :#and  self.light != "green":
                self.force_set_light_one_phase(True)
                
                self.light = "green"
            if (float(setting_list[2])**2 + float(setting_list[3])**2 +float(setting_list[4])**2) >= 10 :
                # if self.light != "auto":
                    print(False)
                    self.force_set_light_one_phase(False)
        else :
            # if self.light != "auto":
            self.force_set_light_one_phase(False)
def main(args=None):
    try:
        rclpy.init(args=args)
        prcocess = TRLightMgr("TRLightMgr")

        rclpy.spin(prcocess)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        prcocess.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print (e)



if __name__ == '__main__':
    main()