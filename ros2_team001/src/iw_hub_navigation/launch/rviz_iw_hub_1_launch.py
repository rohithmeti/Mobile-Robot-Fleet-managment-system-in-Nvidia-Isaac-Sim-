import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # UPDATE THIS PATH to where you actually want to save your perfect RViz config
    rviz_config_path = '/home/carrubuntu/ros2_team001/my_perfect_config.rviz'

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_iw_hub_1',
            output='screen',
            # 1. Automatically force Simulation Time
            parameters=[{'use_sim_time': True}],
            # 2. Automatically fix the Tools! (No more editing Tool Properties)
            remappings=[
                ('/initialpose', '/iw_hub_1/initialpose'),
                ('/goal_pose', '/iw_hub_1/goal_pose'),
                ('/clicked_point', '/iw_hub_1/clicked_point')
            ],
            # 3. Load your saved visual layout
            arguments=['-d', rviz_config_path]
        )
    ])
