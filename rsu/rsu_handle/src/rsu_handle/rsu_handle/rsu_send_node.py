import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import socket

class RsuMapReceiver(Node):
    def __init__(self):
        super().__init__('rsu_map_receiver')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/rsu_map',
            self.map_callback,
            10)
            
        # 设置目标IP和端口
        self.targetip = '192.168.1.199'
        self.targetport = 30300
        
        # 创建UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info('RsuMapReceiver 初始化完成')

    def map_callback(self, msg):
        try:
            # 获取二进制数据
            data = bytes(msg.data)
            # print(data)
            
            # 通过UDP发送数据
            self.sock.sendto(data, (self.targetip, self.targetport))
            
            self.get_logger().info(f'编码数据已发送到 {self.targetip}:{self.targetport}')
            
        except Exception as e:
            self.get_logger().error(f'发送数据时出错: {str(e)}')

    def __del__(self):
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    receiver = RsuMapReceiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
