"""Launch file for RO-SLAM (Range-Only SLAM) with direct rawlog playback."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RO-SLAM with direct rawlog playback."""
    # Get package directory
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')

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

    # MRPT RBPF SLAM node for RO-SLAM
    slam_node = Node(
        package='mrpt_rbpf_slam',
        executable='mrpt_rbpf_slam',
        name='mrpt_rbpf_slam',
        output='screen',
        parameters=[
            {
                'rawlog_play_delay': 0.2,
                'rawlog_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'RO-SLAM_demo.rawlog'
                ]),
                'ini_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'RO-SLAM_demo.ini'
                ]),
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_source': '/beacon',
            },
            PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'default.yaml'])
        ]
    )

    return LaunchDescription([
        set_rosconsole_config,
        rviz_node,
        slam_node,
    ])
