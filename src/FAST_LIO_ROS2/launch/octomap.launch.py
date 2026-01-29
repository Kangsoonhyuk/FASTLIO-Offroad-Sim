
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_ros2_dir = get_package_share_directory('fast_lio')
    nav2_octomap_params_file = os.path.join(fast_lio_ros2_dir, 'config', 'nav2_octomap_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_in', default_value='/odometry/points',
            description='Input point cloud topic for Octomap'),
        
        DeclareLaunchArgument(
            'params_file', default_value=nav2_octomap_params_file,
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('cloud_in', LaunchConfiguration('cloud_in'))
            ]
        )
    ])
