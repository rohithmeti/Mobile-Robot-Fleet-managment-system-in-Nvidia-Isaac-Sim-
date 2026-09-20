import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directories
    pkg_navigation = FindPackageShare('iw_hub_navigation')
    pkg_slam = FindPackageShare('slam_toolbox')

    # Parameter File Paths
    merger_params_path = PathJoinSubstitution([pkg_navigation, 'params', 'merger_params.yaml'])
    slam_params_path = PathJoinSubstitution([pkg_navigation, 'params', 'slam_params.yaml'])

    # Declare Simulation Time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Force nodes to use Isaac Sim clock'
    )

    # 1. LiDAR Merger Node
    laser_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[merger_params_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 2. SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        laser_merger_node,
        slam_toolbox_launch
    ])
