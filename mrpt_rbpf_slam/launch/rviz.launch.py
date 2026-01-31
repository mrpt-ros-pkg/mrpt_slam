"""Launch file for standalone RViz visualization."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for standalone RViz."""
    # Get package directory
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')

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

    return LaunchDescription([
        rviz_node,
    ])
