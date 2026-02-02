from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    pkg_clearpath_gz = FindPackageShare('clearpath_gz')
    launch_file_lidar3d_0 = PathJoinSubstitution([
        pkg_clearpath_gz, 'config', 'setup', 'sensors', 'launch', 'lidar3d_0.launch.py'])
    launch_file_imu_0 = PathJoinSubstitution([
        pkg_clearpath_gz, 'config', 'setup', 'sensors', 'launch', 'imu_0.launch.py'])

    # Include launch files
    launch_lidar3d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_0]),
    )

    launch_imu_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_0]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_lidar3d_0)
    ld.add_action(launch_imu_0)
    return ld
