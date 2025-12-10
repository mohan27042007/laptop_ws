from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='laptop_teleop',
            executable='custom_teleop',
            name='teleop_key',
            output='screen',
        )
    ])
