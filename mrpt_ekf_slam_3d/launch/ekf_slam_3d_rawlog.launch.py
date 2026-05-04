# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for EKF SLAM 3D with rawlog playback."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for EKF SLAM 3D with rawlog playback."""
    # Get package directory
    mrpt_ekf_slam_3d_share = FindPackageShare('mrpt_ekf_slam_3d')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'config', 'rosconsole.config'
        ])
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'rviz', 'rviz_conf_ekf_3d.rviz'
        ])]
    )

    # MRPT EKF SLAM 3D node
    slam_node = Node(
        package='mrpt_ekf_slam_3d',
        executable='mrpt_ekf_slam_3d',
        name='mrpt_ekf_slam_3d',
        output='screen',
        parameters=[
            {
                'rawlog_play_delay': 0.1,
                'ellipse_scale': 1.0,
                'ini_filename': PathJoinSubstitution([
                    mrpt_ekf_slam_3d_share, 'tutorial', 'kf-slam_6D_demo.ini'
                ]),
                'rawlog_filename': PathJoinSubstitution([
                    mrpt_ekf_slam_3d_share, 'tutorial',
                    'kf-slam_6D_demo.rawlog'
                ]),
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_source': 'landmark',
            },
        ]
    )

    return LaunchDescription([
        set_rosconsole_config,
        rviz_node,
        slam_node,
    ])
