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
        )
    ])
