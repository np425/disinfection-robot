# Inspired by https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_description/launch/robot_description.launch.py
# https://github.com/turtlebot/turtlebot4/commit/2201b469ff0ce6bcf0757a7cbf2ae6b9965a0211

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, PythonExpression

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='eddiebot',
                          description='Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('model'),
                          description='Robot name'),
    # DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
    #                       description='Robot namespace'),
]


def generate_launch_description():
    pkg_description = get_package_share_directory('eddiebot_description')

    xacro_filename = PythonExpression(["'", LaunchConfiguration("model"), "' + '.urdf.xacro'"])

    xacro_file = PathJoinSubstitution([
        pkg_description,
        'urdf',
        'robots',
        xacro_filename
    ])
    # namespace = LaunchConfiguration('namespace')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                # 'namespace:=', namespace
            ]), value_type=str)},
        ],
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         # {'source_list': ['/model/eddiebot/joint_states']},
    #     ],
    # )

    robot_controllers = PathJoinSubstitution(
        [
            pkg_description,
            'config',
            'diff_drive_controller.yaml',
        ]
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

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    # Add nodes to LaunchDescription
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(diff_drive_controller)
    ld.add_action(joint_state_broadcaster)
    return ld
