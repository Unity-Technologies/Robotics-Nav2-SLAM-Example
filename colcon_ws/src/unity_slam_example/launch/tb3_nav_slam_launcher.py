import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
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

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen', 
            arguments=['-d', os.path.join(get_package_share_directory('unity_slam_example'), 'nav2_unity.rviz')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'verbose': 'true'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'verbose': 'true'
            }.items()
        )
    ])
    