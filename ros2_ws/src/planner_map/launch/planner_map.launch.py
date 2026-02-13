from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the planner_map nodes"""
    
    return LaunchDescription([
        Node(
            package='planner_map',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='planner_map',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])
