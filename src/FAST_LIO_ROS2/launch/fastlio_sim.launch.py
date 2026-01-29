from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directories
    # Directories
    clearpath_gz_dir = get_package_share_directory('clearpath_gz')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # Launch Files
    inspection_launch = PathJoinSubstitution([clearpath_gz_dir, 'launch', 'launch_sim.launch.py'])
    
    # Arguments
    arg_scenario = DeclareLaunchArgument(
        'scenario', 
        default_value='test',
        description='Simulation scenario'
    )

    arg_gui = DeclareLaunchArgument(
        'gui', 
        default_value='true',
        description='Launch Gazebo GUI if true'
    )

    # 1. Launch Simulation (Gazebo + Robot + Sensors)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([inspection_launch]),
        launch_arguments={
            'scenario': LaunchConfiguration('scenario'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    # 2. Launch FAST-LIO directly (Override Parameters)
    # We choose velodyne_simulation.yaml as base, but overrides are crucial.
    config_file = PathJoinSubstitution([fast_lio_dir, 'config', 'velodyne_simulation.yaml'])
    rviz_config = PathJoinSubstitution([fast_lio_dir, 'rviz', 'fastlio.rviz'])

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'common.lid_topic': '/cpr_a200_0000/sensors/lidar3d_0/points',
                'preprocess.lidar_type': 5, # Generic
                'map_file_path': '',        # No map loading
                'pcd_save.pcd_save_en': False
            }
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fastlio',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_fast_lio = TimerAction(
        period=10.0,
        actions=[fast_lio_node, rviz_node]
    )

    return LaunchDescription([
        arg_scenario,
        arg_gui,
        sim_launch,
        delayed_fast_lio
    ])
