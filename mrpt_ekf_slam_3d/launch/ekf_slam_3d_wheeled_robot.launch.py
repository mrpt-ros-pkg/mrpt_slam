# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for EKF SLAM 3D with wheeled robot (2D motion, 3D landmarks)."""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for EKF SLAM 3D wheeled robot config."""
    # Get package directories
    mrpt_ekf_slam_3d_share = FindPackageShare('mrpt_ekf_slam_3d')
    mrpt_rawlog_share = FindPackageShare('mrpt_rawlog')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'config', 'rosconsole.config'
        ])
    )

    # Include wheeled robot demo rosbag launch file
    include_demo_rosbag_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mrpt_rawlog_share, 'launch',
                'demo_play_ekf_wheeled_robot.launch.py'
            ])
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

    # MRPT EKF SLAM 3D node — wheeled robot config uses kf-slam_6D_demo.ini
    slam_node = Node(
        package='mrpt_ekf_slam_3d',
        executable='mrpt_ekf_slam_3d',
        name='mrpt_ekf_slam_3d',
        output='screen',
        parameters=[
            {
                'ellipse_scale': 1.0,
                'ini_filename': PathJoinSubstitution([
                    mrpt_ekf_slam_3d_share, 'tutorial', 'kf-slam_6D_demo.ini'
                ]),
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_source': 'landmark',
            },
        ],
    )

    return LaunchDescription([
        set_rosconsole_config,
        include_demo_rosbag_action,
        rviz_node,
        slam_node,
    ])
