"""Launch file for EKF SLAM 3D with live sensor data.

Copyright (C) 2024-2026 Maintainers
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
    """Generate launch description for EKF SLAM 3D."""
    # Get package directories
    mrpt_ekf_slam_3d_share = FindPackageShare('mrpt_ekf_slam_3d')
    mrpt_rawlog_share = FindPackageShare('mrpt_rawlog')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    ini_file_arg = DeclareLaunchArgument(
        'ini_filename',
        default_value=PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'tutorial', 'kf-slam_6D_demo.ini'
        ]),
        description='Full path to the MRPT ini configuration file'
    )

    ellipse_scale_arg = DeclareLaunchArgument(
        'ellipse_scale',
        default_value='1.0',
        description='Scale of covariance ellipses'
    )

    global_frame_arg = DeclareLaunchArgument(
        'global_frame_id',
        default_value='map',
        description='Frame ID for the global (map) frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Frame ID for the odometry frame'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Frame ID for the robot base frame'
    )

    sensor_source_arg = DeclareLaunchArgument(
        'sensor_source',
        default_value='landmark',
        description=(
            'Topic name(s) for sensor data '
            '(comma-separated for multiple sensors)'
        )
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'rviz', 'rviz_conf_ekf_3d.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    include_demo_rosbag_arg = DeclareLaunchArgument(
        'include_demo_rosbag',
        default_value='true',
        description='Include demo rosbag launch file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    ini_filename = LaunchConfiguration('ini_filename')
    ellipse_scale = LaunchConfiguration('ellipse_scale')
    global_frame_id = LaunchConfiguration('global_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    sensor_source = LaunchConfiguration('sensor_source')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    include_demo_rosbag = LaunchConfiguration('include_demo_rosbag')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([
            mrpt_ekf_slam_3d_share, 'config', 'rosconsole.config'
        ])
    )

    # Include demo rosbag launch file (conditional)
    include_demo_rosbag_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                mrpt_rawlog_share, 'launch', 'demo_play_ekf.launch.py'
            ])
        ]),
        condition=IfCondition(include_demo_rosbag)
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    )

    # MRPT EKF SLAM 3D node
    slam_node = Node(
        package='mrpt_ekf_slam_3d',
        executable='mrpt_ekf_slam_3d',
        name='mrpt_ekf_slam_3d',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'ini_filename': ini_filename,
                'ellipse_scale': ellipse_scale,
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'sensor_source': sensor_source,
            },
        ],
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        ini_file_arg,
        ellipse_scale_arg,
        global_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        sensor_source_arg,
        launch_rviz_arg,
        rviz_config_arg,
        include_demo_rosbag_arg,
        # Actions
        set_rosconsole_config,
        include_demo_rosbag_action,
        rviz_node,
        slam_node,
    ])
