import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare(package='tribot_description').find('tribot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'tribot.urdf')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(get_package_share_directory("tribot_description"), 'urdf', 'tribot.urdf.xacro')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )


    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    spawn_entity = Node(
            package='tribot_gazebo',
            executable='inject_entity.py',
            output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='gui', default_value='false',
                              description='Flag to enable joint_state_publisher_gui'),
        # TODO: Gazebo doesn't load
        # gazebo, 
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #spawn_entity,
    ])