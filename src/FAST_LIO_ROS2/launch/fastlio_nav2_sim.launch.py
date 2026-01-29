import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # Directories
    clearpath_gz_dir = get_package_share_directory('clearpath_gz')
    fast_lio_dir = get_package_share_directory('fast_lio')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') 

    # Launch Files
    inspection_launch = PathJoinSubstitution([clearpath_gz_dir, 'launch', 'launch_sim.launch.py'])
    
    # Octomap Params
    nav2_octomap_params_file = os.path.join(fast_lio_dir, 'config', 'nav2_octomap_params.yaml')

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
    
    # 3. Launch Octomap Server
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            nav2_octomap_params_file,
            {
                'filter_ground_plane': True, 
                'ground_filter.angle': 0.8,
                'ground_filter.distance': 0.2,       # 20cm (노이즈 허용)
                'ground_filter.plane_distance': 0.3  # 30cm (높이 오차 허용)
            }
        ],
        remappings=[
            ('cloud_in', '/cloud_registered'),
        ]
    )

    # 4. Launch Nav2 Stack with Remapping
    nav2_launcher = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cpr_a200_0000/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true', 
                    'params_file': nav2_octomap_params_file,
                    'autostart': 'true',
                    'use_composition': 'True',
                    'container_name': 'nav2_container'
                }.items()
            )
        ]
    )

    # Delay FAST-LIO and RViz and Octomap and Nav2
    delayed_fast_lio_octomap = TimerAction(
        period=10.0,
        actions=[fast_lio_node, rviz_node, octomap_node, nav2_launcher]
    )

    

    return LaunchDescription([
        arg_scenario,
        arg_gui,
        sim_launch,
        delayed_fast_lio_octomap
    ])
