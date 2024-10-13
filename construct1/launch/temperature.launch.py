from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='construct1',
            executable='temperature',
            output='screen',
            emulate_tty=True
        )]
    )