from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='serial_driver_py',
            executable='serial_driver',
            name='serial_driver_py',
            output='screen',
            emulate_tty=True,

            parameters=[
                { 'port': "/dev/ttyUSB0" }
            ]
        ),
        Node(
            package='mqttTasn',
            executable='mqttTasn',
            name='mqttTasn',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='mqtt_bridge',
        #     executable='mqtt_bridge',
        #     name='mqtt_bridge',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         { 'host': "39.106.250.159",
        #           'port': 1883,
        #           'sub_tpic': "mqtt_api_cmd1",
        #           'pub_tpic': "mqtt_api_cmd0",
        #           'user_name': "v2x",
        #           'passwd': "654321" }
        #     ]
        # ),
    ])