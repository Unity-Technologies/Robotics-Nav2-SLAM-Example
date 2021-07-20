from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unity_slam_example',
            executable='server_endpoint',
            parameters=[
                {'/ROS_IP': '172.17.0.2'},
                {'/ROS_TCP_PORT': 10000}
            ],
        ),
    ])
