import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.actions import Node
def generate_launch_description():
    my_nav_dir = get_package_share_directory('iw_hub_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Updated map file - pointing directly to your specific YAML
    map_yaml_file = os.path.join(my_nav_dir, 'maps', 'my_new_warehouse1_map.yaml')

    # Add back the other AMRs here when you are ready to test the full fleet
    robots = [
        {'name': 'iw_hub_1', 'id': '1', 'params': 'iw_hub_1_nav2_params.yaml'},
        {'name': 'iw_hub_2', 'id': '2', 'params': 'iw_hub_2_nav2_params.yaml'},
        {'name': 'iw_hub_3', 'id': '3', 'params': 'iw_hub_3_nav2_params.yaml'},
        {'name': 'iw_hub_4', 'id': '4', 'params': 'iw_hub_4_nav2_params.yaml'},
        {'name': 'iw_hub_5', 'id': '5', 'params': 'iw_hub_5_nav2_params.yaml'}
    ]

    ld = LaunchDescription()

    for robot in robots:
        params_file_path = os.path.join(my_nav_dir, 'params', robot['params'])

        robot_nav2_group = GroupAction([
            # Dynamic remaps for each specific AMR's Isaac Sim topics
            SetRemap('odom', f"/chassis/odom_iw_{robot['id']}"),
            SetRemap('cmd_vel', f"/cmd_vel_iw_{robot['id']}"),
            
            # Share the global TF tree
            SetRemap('/tf', '/tf'),
            SetRemap('/tf_static', '/tf_static'),

            # THE STATIC TF CHEAT HAS BEEN REMOVED. 
            # AMCL will now publish the map -> odom transform natively.

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': robot['name'],
                    'use_namespace': 'True',
                    'map': map_yaml_file,
                    'params_file': params_file_path,
                    'autostart': 'True',
                    'use_sim_time': 'True'
                }.items()
            )
        ])

        ld.add_action(robot_nav2_group)

# =========================================================
    # THE MASTER MAP SERVER (Shared by all AMRs)
    # =========================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'topic_name': "map",
            'frame_id': "map",
            'use_sim_time': True
        }]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_map)
    return ld
