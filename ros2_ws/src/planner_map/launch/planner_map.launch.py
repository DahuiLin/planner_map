from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Launch the planner_map nodes with OSM support"""

    # Declare launch arguments
    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value='',
        description='Path to OSM file (.osm)'
    )

    # Get launch configuration
    osm_file = LaunchConfiguration('osm_file')

    return LaunchDescription([
        osm_file_arg,

        Node(
            package='planner_map',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'osm_file': osm_file},
                {'publish_rate': 1.0}
            ]
        ),
        Node(
            package='planner_map',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'osm_file': osm_file}
            ]
        ),
        Node(
            package='planner_map',
            executable='ros2_web_bridge',
            name='ros2_web_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])
