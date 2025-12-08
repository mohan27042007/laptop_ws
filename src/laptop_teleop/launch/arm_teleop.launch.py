from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laptop_teleop',
            executable='arm_teleop',
            name='arm_teleop',
            output='screen',
        )
    ])
