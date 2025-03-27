from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',
            emulate_tty=True,
            # parameters=[
            #     { 'host': "192.168.2.100",
            #       'port': 1883,
            #       'sub_tpic': "mqtt_api_cmd",
            #       'pub_tpic': "mqtt_api_report",
            #       'user_name': "v2x",
            #       'passwd': "654321" }
            # ]
            parameters=[
                { 'host': "39.106.250.159",
                  'port': 1883,
                  'sub_tpic': "mqtt_api_cmd",
                  'pub_tpic': "mqtt_api_cmd",
                  'user_name': "v2x",
                  'passwd': "654321" }
            ]
        ),
        # Node(
        #     package='mqtt_bridge',
        #     executable='mqtt_sub',
        #     name='mqtt_sub',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         { 'host': "192.168.2.100",
        #           'port': 1883,
        #           'sub_tpic': "mqtt_api_cmd",
        #           'pub_tpic': "mqtt_api_report",
        #           'user_name': "v2x",
        #           'passwd': "654321" }
        #     ]
        # )
    ])