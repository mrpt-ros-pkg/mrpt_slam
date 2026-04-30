"""Launch file for mrpt_graphslam_2d single-robot mode.

Copyright (C) 2024-2026 Maintainers
Licensed under BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for single-robot graphSLAM."""
    pkg_share = FindPackageShare('mrpt_graphslam_2d')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'ros_laser_odometry.ini'
        ]),
        description='Full path to the MRPT .ini configuration file'
    )

    anchor_frame_arg = DeclareLaunchArgument(
        'anchor_frame_id',
        default_value='map',
        description='Frame ID for the anchor/map frame'
    )

    base_link_frame_arg = DeclareLaunchArgument(
        'base_link_frame_id',
        default_value='base_link',
        description='Frame ID for the robot base frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Frame ID for the odometry frame'
    )

    nrd_arg = DeclareLaunchArgument(
        'NRD',
        default_value='CFixedIntervalsNRD',
        description='Node Registration Decider class name'
    )

    erd_arg = DeclareLaunchArgument(
        'ERD',
        default_value='CICPCriteriaERD',
        description='Edge Registration Decider class name'
    )

    gso_arg = DeclareLaunchArgument(
        'GSO',
        default_value='CLevMarqGSO',
        description='Graph SLAM Optimizer class name'
    )

    disable_visuals_arg = DeclareLaunchArgument(
        'disable_MRPT_visuals',
        default_value='false',
        description='Disable MRPT GUI windows'
    )

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='1',
        description='Logging verbosity level (0=DEBUG, 1=INFO, 2=WARN, 3=ERROR)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_share, 'rviz', 'sr_graphslam.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    # GraphSLAM node
    graphslam_node = Node(
        package='mrpt_graphslam_2d',
        executable='mrpt_graphslam_2d_node',
        name='mrpt_graphslam_2d',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'anchor_frame_id': LaunchConfiguration('anchor_frame_id'),
            'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'NRD': LaunchConfiguration('NRD'),
            'ERD': LaunchConfiguration('ERD'),
            'GSO': LaunchConfiguration('GSO'),
            'disable_MRPT_visuals': LaunchConfiguration('disable_MRPT_visuals'),
            'verbosity': LaunchConfiguration('verbosity'),
        }],
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_graphslam',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        config_file_arg,
        anchor_frame_arg,
        base_link_frame_arg,
        odom_frame_arg,
        nrd_arg,
        erd_arg,
        gso_arg,
        disable_visuals_arg,
        verbosity_arg,
        launch_rviz_arg,
        rviz_config_arg,
        graphslam_node,
        rviz_node,
    ])
