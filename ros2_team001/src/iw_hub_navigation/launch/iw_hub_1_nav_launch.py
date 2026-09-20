import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('iw_hub_navigation')

    map_yaml_file = os.path.join(my_nav_dir, 'maps', 'my_warehouse_map.yaml')
    params_file = os.path.join(my_nav_dir, 'params', 'iw_hub_1_nav2_params.yaml')

    ld = LaunchDescription()

    namespace = 'iw_hub_1'

    # Launch Nav2 for AMR 1 ONLY
    robot_nav2_group = GroupAction([
        # Force Nav2's internal nodes to use our exact Isaac Sim topics
        SetRemap('odom', '/chassis/odom_iw_1'),
        SetRemap('cmd_vel', '/cmd_vel_iw_1'),
        
        # Share the global TF tree
        SetRemap('/tf', '/tf'),
        SetRemap('/tf_static', '/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'True',
                'map': map_yaml_file,
                'params_file': params_file,
                'autostart': 'True',
                'use_sim_time': 'True',
            }.items()
        )
    ])

    ld.add_action(robot_nav2_group)
    return ld
