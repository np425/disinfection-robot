from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('disinfectbot_control')

    robot_controllers = PathJoinSubstitution([
        pkg_description,
        'config',
        'ros2_control.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
        condition=UnlessCondition(LaunchConfiguration("use_sim"))
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    uv_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'uv_controller',
            '--param-file',
            robot_controllers,
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim"))
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        control_node,
        diff_drive_controller,
        uv_controller,
        joint_state_broadcaster,
    ])
