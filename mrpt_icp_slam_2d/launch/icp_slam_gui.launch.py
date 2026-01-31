"""Launch file for ICP SLAM without RViz (uses MRPT GUI).

Copyright (C) 2024-2026 Jose Luis Blanco-Claraco
Licensed under BSD-3-Clause
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for ICP SLAM with MRPT GUI."""
    # Get package directories
    mrpt_icp_slam_2d_share = FindPackageShare('mrpt_icp_slam_2d')
    mrpt_rawlog_share = FindPackageShare('mrpt_rawlog')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    ini_file_arg = DeclareLaunchArgument(
        'ini_filename',
        default_value=PathJoinSubstitution([
            mrpt_icp_slam_2d_share, 'tutorial', 'icp_slam_demo.ini'
        ]),
        description='Full path to the MRPT ini configuration file'
    )

    global_frame_arg = DeclareLaunchArgument(
        'global_frame_id',
        default_value='map',
        description='Frame ID for the global (map) frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='r1/odom',
        description='Frame ID for the odometry frame'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='r1/base_link',
        description='Frame ID for the robot base frame'
    )

    sensor_source_arg = DeclareLaunchArgument(
        'sensor_source',
        default_value='r1/front_laser/scan',
        description='Sensor topic name'
    )

    trajectory_update_rate_arg = DeclareLaunchArgument(
        'trajectory_update_rate',
        default_value='10.0',
        description='Trajectory update rate (Hz)'
    )

    trajectory_publish_rate_arg = DeclareLaunchArgument(
        'trajectory_publish_rate',
        default_value='5.0',
        description='Trajectory publish rate (Hz)'
    )

    include_demo_rosbag_arg = DeclareLaunchArgument(
        'include_demo_rosbag',
        default_value='true',
        description='Include demo rosbag launch file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    ini_filename = LaunchConfiguration('ini_filename')
    global_frame_id = LaunchConfiguration('global_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    sensor_source = LaunchConfiguration('sensor_source')
    trajectory_update_rate = LaunchConfiguration('trajectory_update_rate')
    trajectory_publish_rate = LaunchConfiguration('trajectory_publish_rate')
    include_demo_rosbag = LaunchConfiguration('include_demo_rosbag')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_icp_slam_2d_share, 'config', 'rosconsole.config'])
    )

    # Include demo rosbag launch file (conditional)
    include_demo_rosbag_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mrpt_rawlog_share, 'launch', 'demo_rosbag.launch.py'])
        ]),
        condition=IfCondition(include_demo_rosbag)
    )

    # MRPT ICP SLAM node (no RViz)
    slam_node = Node(
        package='mrpt_icp_slam_2d',
        executable='mrpt_icp_slam_2d',
        name='mrpt_icp_slam_2d',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'ini_filename': ini_filename,
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'sensor_source': sensor_source,
                'trajectory_update_rate': trajectory_update_rate,
                'trajectory_publish_rate': trajectory_publish_rate,
            }
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        ini_file_arg,
        global_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        sensor_source_arg,
        trajectory_update_rate_arg,
        trajectory_publish_rate_arg,
        include_demo_rosbag_arg,
        # Actions
        set_rosconsole_config,
        include_demo_rosbag_action,
        slam_node,
    ])
