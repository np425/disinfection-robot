from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('disinfectbot_description')

    xacro_filename = PythonExpression(["'", LaunchConfiguration("model"), "' + '.urdf.xacro'"])
    xacro_file = PathJoinSubstitution([
        pkg_description,
        'urdf',
        'robots',
        xacro_filename
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'use_sim:=', LaunchConfiguration('use_sim'), ' '
            ]), value_type=str)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='disinfectbot'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value=LaunchConfiguration('use_sim')),
        robot_state_publisher,
    ])
