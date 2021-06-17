import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #pkg_share = FindPackageShare(package='tribot_description').find('tribot_description')
    pkg_share = get_package_share_directory('tribot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'tribot.urdf')

    urdf_file_name = 'tribot.urdf'
    doc = xacro.parse(open(os.path.join(
        get_package_share_directory('tribot_description'), 'urdf', default_model_path)))
    xacro.process_doc(doc)
    urdf = doc.toxml()


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
                #{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
                {'robot_description': urdf}  

            ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'), 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    #gazebo = IncludeLaunchDescription(
    #    get_package_share_directory(os.path.join('gazebo_ros','gazebo.launch.py'))
    #)

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'tribot', '-topic', 'robot_description'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='gui', default_value='false',
                              description='Flag to enable joint_state_publisher_gui'),
        ## libgazebo_ros_factory.so is used as it contains service call to /spawn_entity which libgazebo_ros_init.so does not 
        #ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        #gazebo,
        #joint_state_publisher_node,
        robot_state_publisher_node,
        #spawn_entity,
        robot_localization_node,
    ])