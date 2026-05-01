# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for standalone RViz visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for standalone RViz."""
    # Get package directory
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')

    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            mrpt_rbpf_slam_share, 'rviz', 'rviz_conf.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    node_namespace_arg = DeclareLaunchArgument(
        'node_namespace',
        default_value='',
        description='Namespace for the RViz node'
    )

    # Get launch configurations
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_namespace = LaunchConfiguration('node_namespace')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav',
        namespace=node_namespace,
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch arguments
        rviz_config_arg,
        use_sim_time_arg,
        node_namespace_arg,
        # Actions
        rviz_node,
    ])
