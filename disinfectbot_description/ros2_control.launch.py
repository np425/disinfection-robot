from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('disinfectbot_control')

    robot_controllers = PathJoinSubstitution([
        pkg_description,
        'config',
        'diff_drive_controller.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
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

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='disinfectbot'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value=LaunchConfiguration('use_sim')),
        control_node,
        joint_state_broadcaster,
        diff_drive_controller,
    ])
