import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ira_laser_tools_share = get_package_share_directory('ira_laser_tools')

    merger_params_file = os.path.join(ira_laser_tools_share, 'params', 'merger_params.yaml')

    return LaunchDescription([
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            namespace='iw_hub_1',
            parameters=[merger_params_file],
            output='screen'
        ),
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            namespace='iw_hub_2',
            parameters=[merger_params_file],
            output='screen'
        ),
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            namespace='iw_hub_3',
            parameters=[merger_params_file],
            output='screen'
        ),
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            namespace='iw_hub_4',
            parameters=[merger_params_file],
            output='screen'
        ),
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            namespace='iw_hub_5',
            parameters=[merger_params_file],
            output='screen'
        )
    ])
