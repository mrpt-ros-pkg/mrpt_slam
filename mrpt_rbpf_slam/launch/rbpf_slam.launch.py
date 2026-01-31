"""Launch file for RBPF SLAM with rosbag playback."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RBPF SLAM."""
    # Get package directories
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')
    mrpt_rawlog_share = FindPackageShare('mrpt_rawlog')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'rosconsole.config'])
    )

    # Include demo rosbag launch file
    include_demo_rosbag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mrpt_rawlog_share, 'launch', 'demo_rosbag.launch.py'])
        ])
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
                'ini_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo.ini'
                ]),
                'odom_frame_id': 'r1/odom',
                'global_frame_id': 'map',
                'base_frame_id': 'r1/base_link',
                'sensor_source': 'r1/front_laser/scan',
            },
            PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'default.yaml'])
        ]
    )

    return LaunchDescription([
        set_rosconsole_config,
        include_demo_rosbag,
        rviz_node,
        slam_node,
    ])
