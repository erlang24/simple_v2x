from std_msgs.msg import String
from asn_msgs.msg import IntersectionState
import asn1tools
import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta, timezone
import json
from std_msgs.msg import UInt8MultiArray


class TrafficLightReceiver(Node):

    def __init__(self):
        super().__init__('handle_topic_node')

        # 订阅 /intersection_state_msg 话题
        self.intersection_state_sub = self.create_subscription(
            IntersectionState,
            '/intersection_state_msg',
            self.intersection_state_msg2asn,
            10)
        
        # self.udp_publisher = self.create_publisher(UInt8MultiArray, '/rsu_json', 10)
        self.rsu_map_publisher = self.create_publisher(UInt8MultiArray, '/rsu_map', 10)

        
        # 编译 ASN.1 文件
        self.asn = asn1tools.compile_files([
            '/home/promote/rsu/rsu_handle/asn/BSM.asn',
            '/home/promote/rsu/rsu_handle/asn/MapSpeedLimit.asn',
            '/home/promote/rsu/rsu_handle/asn/VehClass.asn',
            '/home/promote/rsu/rsu_handle/asn/RSM.asn',
            '/home/promote/rsu/rsu_handle/asn/DefPosition.asn',
            '/home/promote/rsu/rsu_handle/asn/MapLink.asn',
            '/home/promote/rsu/rsu_handle/asn/MapPoint.asn',
            '/home/promote/rsu/rsu_handle/asn/MapNode.asn',
            '/home/promote/rsu/rsu_handle/asn/DefTime.asn',
            '/home/promote/rsu/rsu_handle/asn/DefMotion.asn',
            '/home/promote/rsu/rsu_handle/asn/MapLane.asn',
            '/home/promote/rsu/rsu_handle/asn/VehSize.asn',
            '/home/promote/rsu/rsu_handle/asn/MsgFrame.asn',
            '/home/promote/rsu/rsu_handle/asn/Map.asn',
            '/home/promote/rsu/rsu_handle/asn/VehSafetyExt.asn',
            '/home/promote/rsu/rsu_handle/asn/DefPositionOffset.asn',
            '/home/promote/rsu/rsu_handle/asn/RSI.asn',
            '/home/promote/rsu/rsu_handle/asn/VehStatus.asn',
            '/home/promote/rsu/rsu_handle/asn/DefAcceleration.asn',
            '/home/promote/rsu/rsu_handle/asn/VehEmgExt.asn',
            '/home/promote/rsu/rsu_handle/asn/VehBrake.asn',
            '/home/promote/rsu/rsu_handle/asn/SPATIntersectionState.asn',
            '/home/promote/rsu/rsu_handle/asn/SignalPhaseAndTiming.asn',
        ], numeric_enums=True)



    def intersection_state_msg2asn(self, msg):
        self.get_logger().info(f"收到的消息: {msg}")
        try:
            spat_message = {
                'msgCnt': 101,
                'intersections': [{
                    'intersectionId': {
                        'region': 500,
                        'id': msg.intersection_id
                    },
                    'status': (b'0000100000000000', 16 *8),
                    'phases': []
                }]
            }

            current_time = 0
            for phase in msg.phases:
                phase_data = {
                    'id': phase.id,
                    'phaseStates': []
                }

                for i, phase_state in enumerate(phase.phase_states):
                    # 如果有上一个灯的结束时间，更新 startTime
                    start_time = current_time if i == 0 else phase_data['phaseStates'][-1]['timing'][1]['likelyEndTime']
                    
                    # 更新 nextDuration 和 likelyEndTime
                    next_duration = calculate_next_duration(phase_state)
                    likely_end_time = start_time + phase_state.current_time_remaining

                    # 创建 phase state 数据
                    state_data = {
                        'light': phase_state.light,
                        'timing': ('counting', {
                            'startTime': start_time,
                            'likelyEndTime': likely_end_time,
                            'nextDuration': next_duration,
                            # 'timeConfidence': 100
                        })
                    }
                    
                    # 更新current_time为最后一个灯的likelyEndTime
                    current_time = likely_end_time
                    phase_data['phaseStates'].append(state_data)

                spat_message['intersections'][0]['phases'].append(phase_data)

            # ASN.1 编码
            encoded_spat = self.asn.encode('MessageFrame', ('spatFrame', spat_message))
            
            # 创建 ROS 消息并发布
            ros_msg = UInt8MultiArray()
            ros_msg.data = list(encoded_spat)
            self.rsu_map_publisher.publish(ros_msg)
            
            self.get_logger().info("SPAT消息编码并发布成功")
            decoded_spat = self.asn.decode('MessageFrame', encoded_spat)
            print("解码后的SPAT消息:", decoded_spat)

        except Exception as e:
            self.get_logger().error(f"SPAT消息处理错误: {e}")



        # def spat_state_callback(self, msg):
        #     self.get_logger().info("Received SPAT Message")
        #     for intersection in msg.intersections:
        #         self.get_logger().info(f"Intersection ID: {intersection.intersection_id}")
        #         for phase in intersection.phases:
        #             self.get_logger().info(f"  Phase ID: {phase.id}")
        #             for phase_state in phase.phase_states:
        #                 self.get_logger().info(f"    Light State: {phase_state.light}")
        #                 self.get_logger().info(f"    Current Time Remaining: {phase_state.current_time_remaining}")


def calculate_next_duration(phase_state):
    # 根据信号灯状态返回持续时长（单位为毫秒）
    if phase_state.light == 3:  # 红灯
        return 80  # 红灯持续时长8s
    elif phase_state.light == 6:  # 绿灯
        return 50  # 绿灯持续时长5s
    elif phase_state.light == 7:  # 黄灯
        return 30  # 黄灯持续时长3s
    return 0  # 默认值



def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# 红灯8s light=3
# 绿灯5s light=6
# 黄灯3s light=7

# 灯的顺序是 红绿黄红绿黄……



"""
收到的消息: 
asn_msgs.msg.IntersectionState(intersection_id=85, 
                               status=[], mode=False, set_cmd=0, 
                               phases=[asn_msgs.msg.Phase(id=0, 
                                                          phase_states=[asn_msgs.msg.PhaseState(light=3, 
                                                                                                mode=False, 
                                                                                                current_time_remaining=29, 
                                                                                                flashing_red_times=0, 
                                                                                                red_times=0, flashing_green_times=0, 
                                                                                                green_times=0, flashing_yellow_times=0, 
                                                                                                yellow_times=0, set_cmd=0), 

                                                                        asn_msgs.msg.PhaseState(light=6, mode=False, 
                                                                                                current_time_remaining=2, 
                                                                                                flashing_red_times=0, red_times=0, 
                                                                                                flashing_green_times=0, green_times=0, 
                                                                                                flashing_yellow_times=0, yellow_times=0, 
                                                                                                set_cmd=0)])])

{
    'msgCnt': 101, 
    'intersections': 
    [
        {
        'intersectionId': {'region': 500, 'id': 85}, 
        'status': ['0000000000100000'], 
        'phases': 
            [
                {'id': 0, 
                    'phaseStates': [{
                        'light': 3, 
                        'timing': {
                            'counting': {
                                'startTime': 0, 
                                'likelyEndTime': 290, 
                                'nextDuration': 800
                                }
                            }
                        }, 
                        {'light': 6, 
                         'timing': {
                             'counting': {
                                 'startTime': 0, 
                                 'likelyEndTime': 20, 
                                 'nextDuration': 500
                                }
                            }
                        }
                    ]
                }
            ]
        }
    ]
}
"""
