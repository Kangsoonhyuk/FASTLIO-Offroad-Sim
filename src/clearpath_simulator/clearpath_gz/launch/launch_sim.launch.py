from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    
    # Declare scenario argument
    # 'test' = pipeline.sdf (with trees)
    # 'base' = inspection_baseline.sdf (without row trees)
    arg_scenario = DeclareLaunchArgument(
        'scenario', 
        default_value='test',
        description='Simulation scenario: "test" (with obstacles) or "base" (baseline, no row trees)')

    arg_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI if true')

    # However, substitutions are resolved at runtime.
    # Let's use Python conditionals if possible? No, we need OpaqueFunction or just rely on the fact 
    # that we can construct the string. But '.sdf' is appended by the launch args logic.
    
    # Ideally:
    # if scenario == 'base': world = '.../inspection_baseline'
    # else: world = '.../inspection'
    
    # Since we can't easily use if/else with LaunchConfiguration in this scope without OpaqueFunction,
    # let's use OpaqueFunction to determine the world path.
    from launch.actions import OpaqueFunction

    def launch_setup(context, *args, **kwargs):
        scenario_val = context.launch_configurations['scenario']
        gui_val = context.launch_configurations['gui']
        base_path = os.path.join(get_package_share_directory('clearpath_gz'), 'worlds')
        
        if scenario_val == 'base':
            world_name = 'inspection_baseline'
        else:
            world_name = 'pipeline'
            
        world_path = os.path.join(base_path, world_name)
        
        # --- Launch Logic (moved inside function to access context) ---
        
        pkg_clearpath_gz = get_package_share_directory('clearpath_gz') # Redundant but safe
        
        launch_args = [
            ('world', world_path),
            ('setup_path', pkg_clearpath_gz),
            ('use_sim_time', 'true'),
            ('x', '32.0'),
            ('y', '26.0'),
            ('z', '3.0'),
            ('yaw', '0.0')
        ]
        
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        gz_sim_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        gui_config = PathJoinSubstitution([pkg_clearpath_gz, 'config', 'gui.config'])

        # Calculate resource path
        packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':')]
        custom_model_path = os.path.join(get_package_share_directory('clearpath_gz'), 'meshes')
        resource_paths = [os.path.join(pkg_clearpath_gz, 'worlds'), custom_model_path] + packages_paths
        
        set_resource_path = SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=':'.join(resource_paths))

        gz_args_list = [world_path, '.sdf', ' -r', ' -v 4']
        
        if gui_val != 'true':
            gz_args_list.append(' -s')

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_sim_launch_path]),
            launch_arguments=[
                ('gz_args', gz_args_list)
            ]
        )

        robot_spawn_launch = PathJoinSubstitution(
            [pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
            
        robot_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_spawn_launch]),
            launch_arguments=launch_args
        )
        
        from launch_ros.actions import Node
        clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='clock_bridge', output='screen',
                            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])
                            
        return [set_resource_path, gz_sim, robot_spawn, clock_bridge]


    return LaunchDescription([
        arg_scenario,
        arg_gui,
        OpaqueFunction(function=launch_setup)
    ])
