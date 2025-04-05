from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='usi_angry_turtle',
            executable='move2goal',
            name='move2goal',
            output='screen',
            parameters=[{'tolerance': 0.1}],
        ),
        Node(
            package='usi_angry_turtle',
            executable='state_manager',
            name='state_manager',
            output='screen',
            parameters=[{'k1': 1.0, 'k2': 3.0, 'm': 1.0}],
        ),
        Node(
            package='usi_angry_turtle',
            executable='enemy_turtle',
            name='enemy_turtle',
            output='screen',
        ),
    ])
