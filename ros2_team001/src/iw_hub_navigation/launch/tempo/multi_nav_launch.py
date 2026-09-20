import os
import yaml
import copy
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_nav_dir = get_package_share_directory('iw_hub_navigation')

    map_yaml_file = os.path.join(my_nav_dir, 'maps', 'my_warehouse_map.yaml')
    base_params_file = os.path.join(my_nav_dir, 'params', 'iw_hub_navigation_params.yaml')

    ld = LaunchDescription()

    # Read our clean, universal base YAML
    with open(base_params_file, 'r') as f:
        base_yaml = yaml.safe_load(f)

    # =================================================================
    # EXTRACTED ISAAC SIM STARTING COORDINATES (Converted to Radians)
    # =================================================================
    initial_poses = {
        1: {'x': -22.3, 'y': 10.74, 'yaw': 3.14159},
        2: {'x': -22.3, 'y': 12.35, 'yaw': 3.14159},
        3: {'x': -22.3, 'y': 14.15, 'yaw': 3.14159},
        4: {'x': -22.3, 'y': 15.91, 'yaw': 3.14159},
        5: {'x': -22.3, 'y': 17.65, 'yaw': 3.14159},
    }

    for i in range(1, 6):
        namespace = f'iw_hub_{i}'

        # Create an exact copy of the base YAML for this specific robot
        robot_yaml = copy.deepcopy(base_yaml)

        # 1. Precision TF Frame Overrides
        robot_yaml['amcl']['ros__parameters']['base_frame_id'] = f'{namespace}/base_link'
        robot_yaml['amcl']['ros__parameters']['odom_frame_id'] = f'{namespace}/odom'
        
        robot_yaml['bt_navigator']['ros__parameters']['robot_base_frame'] = f'{namespace}/base_link'
        
        robot_yaml['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = f'{namespace}/odom'
        robot_yaml['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'] = f'{namespace}/base_link'
        
        robot_yaml['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame'] = f'{namespace}/base_link'
        
        robot_yaml['behavior_server']['ros__parameters']['global_frame'] = f'{namespace}/odom'
        robot_yaml['behavior_server']['ros__parameters']['robot_base_frame'] = f'{namespace}/base_link'

        # 2. INJECT THE INITIAL POSES SO AMCL WAKES UP INSTANTLY
        pose = initial_poses[i]
        robot_yaml['amcl']['ros__parameters']['set_initial_pose'] = True
        robot_yaml['amcl']['ros__parameters']['initial_pose'] = {
            'x': pose['x'],
            'y': pose['y'],
            'z': 0.0,
            'yaw': pose['yaw']
        }

        # Save this perfect configuration to a temporary hidden file
        tmp_param_path = os.path.join(tempfile.gettempdir(), f'{namespace}_nav2_params.yaml')
        with open(tmp_param_path, 'w') as f:
            yaml.dump(robot_yaml, f)

        # Launch the robot group using the specific temp file
        robot_nav2_group = GroupAction([
            SetRemap('scan', f'/iw_hub_{i}/scan_merged_{i}'),
            SetRemap('odom', f'/chassis/odom_iw_{i}'),
            SetRemap('cmd_vel', f'/cmd_vel_iw_{i}'),
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
                    'params_file': tmp_param_path,
                    'autostart': 'True',
                    'use_sim_time': 'True',
                }.items()
            )
        ])

        ld.add_action(robot_nav2_group)

    return ld
