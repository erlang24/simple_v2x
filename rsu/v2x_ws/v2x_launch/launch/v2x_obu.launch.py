from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asn2msgs',
            executable='asn2msgs',
            name='asn2msgs',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'url': 'ws://127.0.0.1:9090'
                }
            ]
        ),
        Node(
            package='mqtt_bridge',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[
                { 'host': "39.106.250.159",
                  'port': 1883,
                  'sub_tpic': "mqtt_api_cmd0",
                  'pub_tpic': "mqtt_api_cmd1",
                  'user_name': "v2x",
                  'passwd': "654321" }
            ]
        ),
        GroupAction(
        actions=[
            # push_ros_namespace to set namespace of included nodes
            # PushRosNamespace('chatter_xml_ns'),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('rosbridge_server'),
                        'launch/rosbridge_websocket_launch.xml'))
            ),
        ]
    )
    ])
