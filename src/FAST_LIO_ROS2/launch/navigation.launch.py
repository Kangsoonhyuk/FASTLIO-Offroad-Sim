
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_ros2_dir = get_package_share_directory('fast_lio')
    
    # 1. Define Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    scenario = LaunchConfiguration('scenario')

    # 2. Define Include Actions
    
    # Simulation (Gazebo + Robot + Fast-LIO) - Starts Immediately
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_ros2_dir, 'launch', 'fastlio_sim.launch.py')
        ),
        launch_arguments={
            'scenario': scenario,
            'gui': gui
        }.items()
    )

    # Octomap - Starts after delay (e.g. 10s)
    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_ros2_dir, 'launch', 'octomap.launch.py')
        ),
        launch_arguments={
            'cloud_in': '/odometry/points'
        }.items()
    )

    # Nav2 - Starts after delay (e.g. 15s)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_ros2_dir, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Define Delayed Actions
    delayed_octomap = TimerAction(
        period=15.0,
        actions=[octomap_launch]
    )

    delayed_nav2 = TimerAction(
        period=20.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI if true'
        ),

        DeclareLaunchArgument(
            'scenario',
            default_value='base',
            description='Simulation scenario'
        ),

        simulation_launch,
        delayed_octomap,
        delayed_nav2,
        
        # Static Transforms for TF glue
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
            output='screen'
        )
    ])
