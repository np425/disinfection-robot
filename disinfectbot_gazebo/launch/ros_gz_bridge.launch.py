from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument('description', default_value='false',
        #                       description='Launch eddiebot description'),
        # DeclareLaunchArgument('model', default_value='eddie_kinect_v1',
        #                       choices=['eddie_kinect_v1', 'eddie_kinect_v2'],
        #                       description='Eddiebot Model')
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # jointstate_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='joint_states_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/model/eddiebot/joint_states' +
    #          '@sensor_msgs/msg/JointState@gz.msgs.Model']
    #     ])

    # cmd_vel_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='cmd_vel_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }],
    #     arguments=[
    #         ['/diff_drive_controller/cmd_vel' + '@geometry_msgs/msg/Twist' + '@gz.msgs.Twist'],
    #     ])

    # odom_base_tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='odom_base_tf_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #         }],
    #     arguments=[
    #         ['/model/eddiebot/tf' + '@tf2_msgs/msg/TFMessage' + '[gz.msgs.Pose_V']
    #     ],
    #     remappings=[
    #         ('/model/eddiebot/tf', '/tf')
    #     ])

    # odom_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='odom_base_tf_bridge',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #         }],
    #     arguments=[
    #         ['/odom' + '@nav_msgs/msg/Odometry' + '[gz.msgs.Odometry']
    #     ])

    # rgbd_camera_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='rgbd_camera_bridge',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #         }],
    #     arguments=[
    #         ['/rgbd_camera/camera_info' + '@sensor_msgs/msg/CameraInfo'  + '[gz.msgs.CameraInfo'],
    #         ['/rgbd_camera/depth_image' + '@sensor_msgs/msg/Image'       + '[gz.msgs.Image'],
    #         ['/rgbd_camera/image'       + '@sensor_msgs/msg/Image'       + '[gz.msgs.Image'],
    #         ['/rgbd_camera/points'      + '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked']
    #     ])

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
        arguments=[
            ['/scan' + '@sensor_msgs/msg/LaserScan'  + '[gz.msgs.LaserScan'],
        ])

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(jointstate_bridge)
    # ld.add_action(odom_base_tf_bridge)
    # ld.add_action(odom_bridge)
    # ld.add_action(cmd_vel_bridge)
    # ld.add_action(rgbd_camera_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(clock_bridge)
    # ld.add_action(cmd_vel_bridge)
    return ld
