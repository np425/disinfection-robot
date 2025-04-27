from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    # DeclareLaunchArgument('namespace', default_value='',
    #                       description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Eddie World'),
    # DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
    #                       choices=['eddie_kinect_v1'],
    #                       description='Eddiebot Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),

]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_eddiebot_gazebo = get_package_share_directory(
        'disinfectbot_gazebo')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_eddiebot_gazebo, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_eddiebot_gazebo, 'launch', 'disinfectbot_spawn.launch.py'])
    gz_bridge_launch = PathJoinSubstitution(
        [pkg_eddiebot_gazebo, 'launch', 'ros_gz_bridge.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            # ('namespace', LaunchConfiguration('namespace')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_bridge_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_bridge)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    return ld
