import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Paths
    fast_lio_dir = get_package_share_directory('fast_lio')
    clearpath_gz_dir = get_package_share_directory('clearpath_gz')
    groundgrid_dir = get_package_share_directory('groundgrid')
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Launch Simulation (Clearpath Gazebo)
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_gz_dir, 'launch', 'launch_sim.launch.py')
        ),
        launch_arguments=[
            ('world', 'inspection_world'),
            ('setup_path', '/home/kangsoonhyuk/clearpath_ws/src/clearpath_simulator/clearpath_gz')
        ]
    )

    # 2. Launch FAST-LIO Mapping Node
    # 2. Launch FAST-LIO Mapping Node
    fast_lio_config = os.path.join(fast_lio_dir, 'config', 'velodyne_simulation.yaml')
    
    launch_fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fast_lio_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/points_raw', '/cpr_a200_0000/sensors/lidar3d_0/points'),
            ('/imu_raw', '/cpr_a200_0000/sensors/imu_0/data')
        ]
    )

    # 3. Launch GroundGrid Node
    # GroundGrid requires:
    # - PointCloud Input (remapped to /cloud_registered from FAST-LIO)
    # - Config file (using default kitti.yaml structure but adjusted as needed)
    
    groundgrid_config = os.path.join(groundgrid_dir, 'param', 'kitti.yaml')

    launch_groundgrid = Node(
        package='groundgrid',
        executable='groundgrid_node',
        name='groundgrid_node',
        output='screen',
        parameters=[
            groundgrid_config,
            {'groundgrid/dataset_name': 'live'}, # Live mode for ROS 2 execution
            {'use_sim_time': use_sim_time},
            {'groundgrid/sensor': 'Velodyne'}, # Using generic velodyne config as base
            {'groundgrid/visualize': True},
            {'groundgrid/map_frame': 'camera_init'},
            {'groundgrid/base_frame': 'body'},
            {'groundgrid/sensor_frame': 'body'}, # Assuming sensor is at body origin or close enough for now
        ],
        remappings=[
            ('/pointcloud', '/cloud_registered'), # Input from FAST-LIO
            ('/groundgrid/odometry_in', '/Odometry') # Input from FAST-LIO
        ]
    )

    # 4. Launch RViz for Visualization
    rviz_config_path = os.path.join(fast_lio_dir, 'rviz', 'fastlio_groundgrid.rviz')
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Static TF for map to camera_init (if needed, FAST-LIO usually handles this)
    # But often we need a link between map and camera_init if not provided
    # fast_lio publishes camera_init -> body. We need map -> camera_init = Identity usually.
    # Actually, FAST-LIO publishes Odometry in camera_init frame. 
    # Let's see if we need a static transform. Usually not if config is right.

    # Delayed Actions to Ensure Sim Ready
    delayed_fast_lio = TimerAction(
        period=10.0,
        actions=[launch_fast_lio]
    )

    delayed_groundgrid = TimerAction(
        period=12.0,
        actions=[launch_groundgrid]
    )

    delayed_rviz = TimerAction(
        period=15.0,
        actions=[launch_rviz]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        launch_sim,
        delayed_fast_lio,
        delayed_groundgrid,
        delayed_rviz
    ])
