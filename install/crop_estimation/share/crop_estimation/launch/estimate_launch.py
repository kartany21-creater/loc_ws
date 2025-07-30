from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crop_estimation',
            executable='estimate_node',
            name='estimate_node',
            output='screen'
        ),
        Node(
            package='crop_estimation',
            executable='um7_node',
            name='um7_node',
            output='screen'
        ),
        Node(
            package='crop_estimation',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),
    ])

