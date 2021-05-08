import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(get_package_share_directory("tribot_description"), 'urdf', 'tribot.urdf.xacro')

    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        gazebo,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
    ])