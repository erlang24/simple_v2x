import imp
import serial

from std_msgs.msg import String

from std_msgs.msg import UInt8MultiArray

from mqtt_bridge.emq_pub import EmqPub
# from mqtt_bridge.emq_sub import EmqSub

# from asn_recive_msg
import rclpy

from rclpy.node import Node


class SerialPortMgr(Node):

    def __init__(self,node_name):
        super(SerialPortMgr, self).__init__(node_name)

        self._host = self.declare_parameter('host', '127.0.0.1').value
        self._host = self.declare_parameter('mqtt_host', self._host).value
        print("connect mqtt_host === >  ",self._host)

        self._port = self.declare_parameter('port', 1883).value
        self._sub_topic = self.declare_parameter('sub_tpic', "test_pub").value
        self._pub_topic = self.declare_parameter('pub_tpic', "test_sub").value
        self._user_name = self.declare_parameter('user_name', "admin").value
        self._passwd     = str(self.declare_parameter('passwd', "public").value)


        # publish
        # self.spat_pub = self.create_publisher(String, "mqtt_recive", 10)
        self.spat_pub = self.create_publisher(UInt8MultiArray, "mqtt_recive", 10)

        # subscription
        # self.subscription = self.create_subscription(
        #      String,
        #     '/mqtt_sent',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
             UInt8MultiArray,
            '/mqtt_sent',
            self.listener_callback2,
            10)
        self.subscription2  # prevent unused variable warning



        # print( " host : %s, port %s , pub_topic : %s, sub_topic: %s ,user_name : %s ,passwd: %s " % 
        #         (self._host, self._port, self._pub_topic,self._sub_topic, self._user_name, self._passwd)) 

        self.emq_pub = EmqPub(
                host   = self._host,
                port   = self._port,
                topic  = self._pub_topic,
                user   = self._user_name,
                passwd = self._passwd
        )


        # self.create_timer(0.5, self.timer_callback)

    def __del__(self):
        self.emq_sub.disconnect()

    # def timer_callback(self):
    #     msg = String()
    #     print(type(msg.data))
    #     msg.data =  bytes.fromhex("0A0B0C0D").decode("utf-8");
    #     # msg.data =  bytes.fromhex("0A0B0C0D").decode("ascii");
    #     self.spat_pub.publish(msg)

    # def listener_callback(self, msg):
    #     try:
    #         ########################
    #         ##  send to mqtt server
    #         ########################
    #         print("=====================\n")
    #         # self.get_logger().info("hex :  %s\n" % msg.data.encode('utf-8').hex())
    #         # self.emq_pub.publish(msg.data.encode('utf-8'))

    #     except Exception as e:
    #         self.get_logger().error("------- %s\n" % e)
    #         pass

    def listener_callback2(self, msg):
        try:
            ########################
            ##  send to mqtt server
            ########################
            # self.get_logger().info("hex :  %s\n" % msg.data.encode('utf-8').hex())
            self.emq_pub.publish(bytes(msg.data))

        except Exception as e:
            self.get_logger().error("------- %s\n" % e)
            pass


def main(args=None):
    rclpy.init(args=args)


    prcocess = SerialPortMgr("mqtt_bridge")

    rclpy.spin(prcocess)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    prcocess.destroy_node()
    rclpy.shutdown()
