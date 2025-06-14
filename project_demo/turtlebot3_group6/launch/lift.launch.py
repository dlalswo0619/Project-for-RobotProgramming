from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_group6',
            executable='object_to_goal',
            name='object_to_goal',
            output='screen'
        ),
        Node(
            package='turtlebot3_group6',
            executable='path_node',
            name='path_node',
            output='screen'
        )
    ])
