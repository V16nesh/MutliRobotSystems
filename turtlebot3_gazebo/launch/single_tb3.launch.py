#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# Import the get_package_share_directory function
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='tb3_0',
            description='Name of the TurtleBot3 instance'),
        DeclareLaunchArgument(
            'x_pos',
            default_value='0.0',
            description='X position of the TurtleBot3 instance'),
        DeclareLaunchArgument(
            'y_pos',
            default_value='0.0',
            description='Y position of the TurtleBot3 instance'),
        DeclareLaunchArgument(
            'z_pos',
            default_value='0.0',
            description='Z position of the TurtleBot3 instance'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')]),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot_name'),
                '-x', LaunchConfiguration('x_pos'),
                '-y', LaunchConfiguration('y_pos'),
                '-z', LaunchConfiguration('z_pos'),
                '-file', os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')
            ],
            output='screen'
        ),
    ])
