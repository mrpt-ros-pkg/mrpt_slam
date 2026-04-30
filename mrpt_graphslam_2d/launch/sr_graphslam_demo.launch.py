"""Demo launch file for single-robot graphSLAM with default config.

Copyright (C) 2024-2026 Maintainers
Licensed under BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for single-robot graphSLAM demo."""
    pkg_share = FindPackageShare('mrpt_graphslam_2d')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'ros_odometry_2DRangeScans.ini'
        ]),
        description='Full path to the MRPT .ini configuration file'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Include main graphslam launch
    graphslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'graphslam.launch.py'])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'disable_MRPT_visuals': 'true',
        }.items(),
    )

    return LaunchDescription([
        config_file_arg,
        launch_rviz_arg,
        graphslam_launch,
    ])
