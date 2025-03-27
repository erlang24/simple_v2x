import asn1tools
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import ast

class RsuMapNode(Node):
    def __init__(self):
        super().__init__('rsu_map_node')
        self.publisher = self.create_publisher(UInt8MultiArray, '/rsu_map', 10)
        
        self.asn1_spec = asn1tools.compile_files([
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/BSM.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MapSpeedLimit.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehClass.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/RSM.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/DefPosition.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MapLink.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MapPoint.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MapNode.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/DefTime.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/DefMotion.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MapLane.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehSize.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/MsgFrame.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/Map.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehSafetyExt.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/DefPositionOffset.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/RSI.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehStatus.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/DefAcceleration.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehEmgExt.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/VehBrake.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/SPATIntersectionState.asn',
            # '/home/erlang/simple_v2x/rsu/rsu_handle/asn/SignalPhaseAndTiming.asn',

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

        self.map_data = self.read_map_file()
        self.encoded_data = None

        if self.map_data is not None:
            self.encode_map_data()
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def read_map_file(self):
        try:
            # with open('/home/erlang/simple_v2x/rsu/JG_asn_es.json.py', 'r') as f:
            with open('/home/promote/rsu/JG_asn_center.json.py', 'r') as f:
                content = f.read()
                return ast.literal_eval(content)
        except Exception as e:
            self.get_logger().error(f'Error reading map file: {str(e)}')
            return None

    def encode_map_data(self):
        try:
            self.get_logger().info('正在编码地图数据...')
            self.encoded_data = self.asn1_spec.encode("MessageFrame", ('mapFrame', self.map_data))
            self.get_logger().info('地图数据编码成功.')
        except Exception as e:
            self.get_logger().error(f"编码地图数据时出错: {str(e)}")

    def timer_callback(self):
        if self.encoded_data is None:
            self.get_logger().error('Encoded data is not available.')
            return 
        try:
            msg = UInt8MultiArray()
            msg.data = self.encoded_data

            self.publisher.publish(msg)
            self.get_logger().info(f"Published map data to /rsu_map")
        except Exception as e:
            self.get_logger().error(f"发布地图数据时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    rsu_map_node = RsuMapNode()
    rclpy.spin(rsu_map_node)
    rsu_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
