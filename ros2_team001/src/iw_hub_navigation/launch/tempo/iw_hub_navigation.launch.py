# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # FIXED: Updated the map name to exactly match what is in your folder
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"), "maps", "my_warehouse_map.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"), "params", "iw_hub_navigation_params.yaml"
        ),
    )

    merger_param_dir = LaunchConfiguration(
        "merger_params_file",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"), "params", "merger_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("iw_hub_navigation"), "rviz2", "iw_hub_navigation.rviz")

    # The LiDAR Merger Node
    laser_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[merger_param_dir, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ADDED: Automatic footprint publisher so you never have to open a separate terminal for it again
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock if true"
            ),
            
            # Remap cmd_vel to cmd_vel_iw_1
            SetRemap(src='cmd_vel', dst='cmd_vel_iw_1'),
            
            # Launch the custom nodes
            laser_merger_node,
            base_footprint_tf,
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),
        ]
    )
