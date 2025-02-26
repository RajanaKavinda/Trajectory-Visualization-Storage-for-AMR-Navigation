import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    trajectory_pub_server_node = Node(
            package='trajectory_pkg',
            executable='trajectory_pub_saver',
            name='trajectory_pub_saver',
            output='screen',
            parameters=[{
                'pose_topic': '/odom',
                'visualization_topic': '/path_visual',
                'reference_frame': 'odom',
                'update_rate': 8.0
            }]
        )
    trajectory_reader_node = Node(
            package='trajectory_pkg',
            executable='trajectory_reader',
            name='trajectory_reader',
            output='screen',
            parameters=[{
                'visualization_topic': "/loaded_path_markers",
                'reference_frame': 'odom'
            }]
    )
    return LaunchDescription([
        trajectory_pub_server_node,
        trajectory_reader_node
    ])
