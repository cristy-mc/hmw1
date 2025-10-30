import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Gazebo interface'
    )

    controller_type_arg = DeclareLaunchArgument(
        name='controller_type',
        default_value='position',
        description='Controller to use: position or trajectory'
    )

    pkg_share = get_package_share_directory('armando_description')
    xacro_armando = os.path.join(pkg_share, "urdf", "arm.urdf.xacro")
    robot_description_arm_xacro = {"robot_description": Command(['xacro ', xacro_armando])}

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'gz_args': '-r empty.sdf'
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_arm_xacro, {"use_sim_time": True}],
    )

    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_armando',
        arguments=['-topic', '/robot_description',
                   '-entity', 'armando',
                   '-z', '0.3',
                   '-unpause'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'position'"]))
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'trajectory'"]))
    )

    # Delay controllers after URDF is spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node,
            on_exit=[joint_state_broadcaster],
        )
    )

    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_controller],
        )
    )

    delay_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller],
        )
    )

    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/camera:=/camera1',
        ],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        controller_type_arg,
        empty_world_launch,
        robot_state_publisher_node,
        urdf_spawner_node,
        delay_joint_state_broadcaster,
        delay_position_controller,
        delay_trajectory_controller,
        bridge_camera,
    ])
