from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'construct1',
            executable = 'node_test1',
            output = 'screen'
            ),
        Node(
            package = 'construct1',
            executable = 'node_test2',
            output = 'screen'
            ),
        
    ])