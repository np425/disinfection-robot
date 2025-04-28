from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('disinfectbot_rviz')

    rviz_config_file = PathJoinSubstitution([
        pkg_description,
        'config',
        LaunchConfiguration('rviz_config')
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value='disinfectbot.rviz'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        rviz_node,
    ])
