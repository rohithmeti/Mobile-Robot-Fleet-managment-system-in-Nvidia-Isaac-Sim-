import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap, PushRosNamespace

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    
    map_dir = os.path.join(
        get_package_share_directory("iw_hub_navigation"), 
        "maps", 
        "my_warehouse_map.yaml"
    )

    template_param_dir = os.path.join(
        get_package_share_directory("iw_hub_navigation"), 
        "params", 
        "multi_iw_hub_params_template.yaml"
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    robots = [
        {'name': 'iw_hub_1', 'n': '1', 'x': '-22.3', 'y': '10.74', 'yaw': '3.14159'},
        {'name': 'iw_hub_2', 'n': '2', 'x': '-22.3', 'y': '12.35', 'yaw': '3.14159'},
        {'name': 'iw_hub_3', 'n': '3', 'x': '-22.3', 'y': '14.15', 'yaw': '3.14159'},
        {'name': 'iw_hub_4', 'n': '4', 'x': '-22.3', 'y': '15.91', 'yaw': '3.14159'},
        {'name': 'iw_hub_5', 'n': '5', 'x': '-22.3', 'y': '17.65', 'yaw': '3.14159'}
    ]

    def launch_robots(context):
        launch_actions = []
        with open(template_param_dir, 'r') as f:
            template_yaml = f.read()

        for robot in robots:
            r_name = robot['name']
            r_num = robot['n']
            
            custom_yaml = template_yaml.replace('[ROBOT_NAME]', r_name)
            custom_yaml = custom_yaml.replace('[N]', r_num)
            custom_yaml = custom_yaml.replace('[START_X]', robot['x'])
            custom_yaml = custom_yaml.replace('[START_Y]', robot['y'])
            custom_yaml = custom_yaml.replace('[START_YAW]', robot['yaw'])
            
            tmp_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
            tmp_file.write(custom_yaml)
            tmp_file.close()

            nav_group = GroupAction(actions=[
                PushRosNamespace(r_name),
                
                # Punch holes for these specific topics BACK to the global space
                SetRemap('map', '/map'),
                SetRemap('tf', '/tf'),
                SetRemap('tf_static', '/tf_static'),
                SetRemap('clock', '/clock'),
                
                # Route cmd_vel uniquely per robot
                SetRemap('cmd_vel', f'/cmd_vel_iw_{r_num}'),
                
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
                    launch_arguments={
                        'namespace': '',               # Let PushRosNamespace do the work securely
                        'use_namespace': 'False',      # STOPS Nav2 from hijacking and breaking our TF remaps
                        'map': map_dir,
                        'use_sim_time': 'True',
                        'params_file': tmp_file.name,
                        'autostart': 'True'
                    }.items()
                )
            ])
            launch_actions.append(nav_group)
        return launch_actions

    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server', 
             parameters=[{'yaml_filename': map_dir}, {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True}, {'node_names': ['map_server']}]),
        OpaqueFunction(function=launch_robots)
    ])
