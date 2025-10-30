from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    pkg_share = get_package_share_directory('armando_description')
    
    #urdf_path = os.path.join(pkg_share, 'urdf', 'arm.urdf')

    xacro_armando = os.path.join(pkg_share, "urdf", "arm.urdf.xacro")
    rviz_config_path = os.path.join(pkg_share, 'config', 'armando_display.rviz')

    '''with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    params = {'robot_description': robot_description}'''

    robot_description_arm_xacro = {"robot_description": Command(['xacro ', xacro_armando])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_arm_xacro,
                    {"use_sim_time": True},
            ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
       arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
