from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    controller_type_arg = DeclareLaunchArgument(
        name='controller_type',
        default_value='position',
        description='Controller to use: position or trajectory'
    )

    arm_controller_node = Node(
        package='armando_controller',
        executable='arm_controller_node',
        name='arm_controller_node',
        output='screen',
        parameters=[{'controller_type': LaunchConfiguration('controller_type')}],
    )

    return LaunchDescription([
        controller_type_arg,
        arm_controller_node
    ])
