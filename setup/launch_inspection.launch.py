from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    
    # Path to the custom world file (without .sdf extension, as simulation.launch.py appends it)
    world_path = '/home/kangsoonhyuk/clearpath_ws/src/cpr_gazebo/cpr_inspection_gazebo/worlds/inspection'
    
    # Resource path for cpr_inspection_gazebo assets
    cpr_gazebo_path = '/home/kangsoonhyuk/clearpath_ws/src/cpr_gazebo'
    
    # Set IGN_GAZEBO_RESOURCE_PATH to include cpr_gazebo
    # We append to the existing path if possible, or just set it.
    # Note: clearpath_gz sets it too, checking if we conflict.
    # clearpath_gz/launch/gz_sim.launch.py uses SetEnvironmentVariable which might overwrite if not careful,
    # but it constructs it from AMENT_PREFIX_PATH.
    # Since cpr_gazebo is in src but NOT installed/built effectively as a ros2 package yet (it's a ros1 package), 
    # we need to explicity add it.
    
    # Ideally we should append to the variable. But Launch action SetEnvironmentVariable sets it.
    # However, gz_sim.launch.py sets 'IGN_GAZEBO_RESOURCE_PATH' using value=[..., ':', ...].
    # If we set it here, it might get overwritten by gz_sim.launch.py unless we hook into it.
    
    # Actually, gz_sim.launch.py does:
    # packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    # value=[os.path.join(pkg_clearpath_gz, 'worlds'), ':' + ':'.join(packages_paths)]
    
    # It seems gz_sim.launch.py recreates the variable from AMENT_PREFIX_PATH.
    # It does NOT respect an existing IGN_GAZEBO_RESOURCE_PATH.
    
    # This is a problem. 'cpr_inspection_gazebo' is NOT a built ROS 2 package, so it won't be in AMENT_PREFIX_PATH.
    # We must patch this by pointing to the src directory in the launch arguments? No, gz_sim doesn't take that.
    
    # Wait, we can modify the environment of the process we are launching?
    # No, gz_sim.launch.py is an included launch file.
    
    # Strategy: We can use the 'AppendEnvironmentVariable' action if it exists (it does in newer ROS 2), 
    # but for Humble it's safer to pre-calculate and pass it?
    # No, gz_sim.launch.py strictly overwrites it.
    
    # WORKAROUND:
    # Instead of using simulation.launch.py, we can just invoke gz_sim.launch.py + robot_spawn.launch.py ourselves 
    # and set the env var correctly in OUR list.
    
    # Let's copy the logic from simulation.launch.py but add our resource path.
    
    gz_sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
        
    # We need to set the env var BEFORE including gz_sim.
    # BUT gz_sim.launch.py overwrites it!
    # Let's check gz_sim.launch.py content again.
    # Line 87: gz_sim_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=...)
    # Yes, it overwrites.
    
    # However, it includes ':'.join(packages_paths).
    # If we add our path to AMENT_PREFIX_PATH, it might be picked up?
    # AMENT_PREFIX_PATH expects 'share' subdirs usually.
    
    # Alternative: We can define a simplified launch that simply runs `ign gazebo` cmd directly?
    # No, we want the bridges and stuff.
    
    # Let's try to set IGN_GAZEBO_RESOURCE_PATH *after*? No, declaration order matters.
    # If gz_sim.launch.py is included, its actions are added to the launch description.
    
    # Wait, if `cpr_gazebo` structure is `src/cpr_gazebo/cpr_inspection_gazebo`, we need `src/cpr_gazebo` in the path.
    
    # HACK: changing the environment variable 'AMENT_PREFIX_PATH' to include our source dir might trick it?
    # packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    # It appends '/share'. Our source dir doesn't have a share folder structure like that.
    
    # Solution: We will NOT use simulation.launch.py. We will recreate its logic but fix the env var.
    
    pass

    # Re-writing the content below to be a full replacement of simulation.launch.py logic
    
    launch_args = [
        ('world', world_path),
        ('setup_path', '/home/kangsoonhyuk/clearpath_ws/setup/'),
        ('use_sim_time', 'true'),
        ('x', '18.0'),
        ('y', '10.0'),
        ('z', '2.0'),
        ('yaw', '0.0')
    ]
    
    # Re-implement gz_sim launch logic locally so we can control the environment
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    
    gui_config = PathJoinSubstitution([pkg_clearpath_gz, 'config', 'gui.config'])
    
    # Calculate resource path
    # Get standard paths
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':')]
    
    # Add our custom path
    custom_model_path = '/home/kangsoonhyuk/clearpath_ws/src/cpr_gazebo'
    resource_paths = [os.path.join(pkg_clearpath_gz, 'worlds'), custom_model_path] + packages_paths
    
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join(resource_paths))
        
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_path]),
        launch_arguments=[
            ('gz_args', [world_path, '.sdf', ' -r', ' -v 4', ' --gui-config ', gui_config])
        ]
    )
    
    # Robot Spawn
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=launch_args
    )
    
    # Clock bridge (from gz_sim.launch.py)
    # We need to add this too since we are bypassing gz_sim.launch.py from clearpath_gz
    from launch_ros.actions import Node
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge', output='screen',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_spawn,
        clock_bridge
    ])
