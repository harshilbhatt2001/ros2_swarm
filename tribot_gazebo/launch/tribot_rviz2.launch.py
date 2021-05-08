#!/usr/bin/env python

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tribot_gazebo = get_package_share_directory('tribot_gazebo')

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    #Rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        #arguments=['-d', os.path.join(pkg_gazebo_ros, 'rviz', 'tribot_gazebo2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        #DeclareLaunchArgument(
        #    'world'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz.'),
        gazebo,
        rviz
    ])