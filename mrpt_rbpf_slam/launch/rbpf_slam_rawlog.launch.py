"""Launch file for RBPF SLAM with rawlog playback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RBPF SLAM with rawlog playback."""
    # Get package directory
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')

    # Declare launch arguments
    example_arg = DeclareLaunchArgument(
        'example',
        default_value='',
        description='Optional suffix for rawlog file (e.g., "_2" for grid_slam_demo_2.rawlog)'
    )

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'rosconsole.config'])
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            mrpt_rbpf_slam_share, 'rviz', 'rviz_conf.rviz'
        ])]
    )

    # MRPT RBPF SLAM node
    slam_node = Node(
        package='mrpt_rbpf_slam',
        executable='mrpt_rbpf_slam',
        name='mrpt_rbpf_slam',
        output='screen',
        parameters=[
            {
                'rawlog_play_delay': 0.01,
                'ini_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo.ini'
                ]),
                'rawlog_filename': [
                    PathJoinSubstitution([
                        mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo'
                    ]),
                    LaunchConfiguration('example'),
                    '.rawlog'
                ],
                'odom_frame_id': 'r1/odom',
                'global_frame_id': 'map',
                'base_frame_id': 'r1/base_link',
                'sensor_source': 'r1/front_laser/scan',
            },
            PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'default.yaml'])
        ]
    )

    return LaunchDescription([
        example_arg,
        set_rosconsole_config,
        rviz_node,
        slam_node,
    ])
