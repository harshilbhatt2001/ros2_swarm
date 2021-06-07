import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('tribot_fake_node'), 'launch')),
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'tribot.urdf'
    urdf = os.path.join(
        get_package_share_directory('tribot_description'), 'urdf', urdf_file_name)
    
    return LaunchDescription([
        Node(
            package='tribot_fake_node',
            executable='tribot_fake_node',
            output='screen'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            arguments=[urdf]),
    ])
    