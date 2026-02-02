import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    clearpath_gz_dir = get_package_share_directory('clearpath_gz')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # Launch Files
    inspection_launch = PathJoinSubstitution([clearpath_gz_dir, 'launch', 'launch_sim.launch.py'])
    
    # Octomap Params (Reused or new params for elevation map can be added here)
    # nav2_octomap_params_file = os.path.join(fast_lio_dir, 'config', 'nav2_octomap_params.yaml')

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
    
    # 3. Launch Elevation Mapping Node (Instead of Octomap)
    elevation_mapping_node = Node(
        package='fast_lio',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'grid_resolution': 0.2,
                'robot_height': 2.0,
                'obstacle_height_thresh': 0.3,
                'max_slope_degree': 50.0,
                'input_topic': '/cloud_registered',
                'map_frame_id': 'camera_init'
             }
        ]
    )

    # Delay FAST-LIO, RViz, and Elevation Mapping
    delayed_launch = TimerAction(
        period=10.0,
        actions=[fast_lio_node, rviz_node, elevation_mapping_node]
    )

    return LaunchDescription([
        arg_scenario,
        arg_gui,
        sim_launch,
        delayed_launch
    ])
