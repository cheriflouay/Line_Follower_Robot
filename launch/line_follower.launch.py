#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # Get the package directories
    pkg_name = 'line_follower_robot'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths to files
    world_file = os.path.join(pkg_share, 'worlds', 'line_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'line_follower_robot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    
    # Process the URDF/XACRO file
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # Declare launch argument for RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )
    
    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'line_follower_robot',
            '-x', '-1.5',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Line Follower Node
    line_follower_node = Node(
        package=pkg_name,
        executable='line_follower_node',
        name='line_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.25,
            'kp': 0.006,
            'kd': 0.002
        }]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        line_follower_node,
        rviz_node
    ])