# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for RBPF SLAM with optional demo rosbag playback."""

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
    """Generate launch description for RBPF SLAM."""
    # Get package directory
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            mrpt_rbpf_slam_share, 'config', 'default.yaml'
        ]),
        description='Full path to the parameter configuration file'
    )

    ini_file_arg = DeclareLaunchArgument(
        'ini_filename',
        default_value=PathJoinSubstitution([
            mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo.ini'
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
            mrpt_rbpf_slam_share, 'rviz', 'rviz_conf.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    include_demo_rosbag_arg = DeclareLaunchArgument(
        'include_demo_rosbag',
        default_value='false',
        description='Include demo rosbag launch file'
    )

    node_namespace_arg = DeclareLaunchArgument(
        'node_namespace',
        default_value='',
        description='Namespace for the SLAM node'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    ini_filename = LaunchConfiguration('ini_filename')
    global_frame_id = LaunchConfiguration('global_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    sensor_source = LaunchConfiguration('sensor_source')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    include_demo_rosbag = LaunchConfiguration('include_demo_rosbag')
    node_namespace = LaunchConfiguration('node_namespace')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'rosconsole.config'])
    )

    # Include demo rosbag launch file (conditional)
    include_demo_rosbag_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mrpt_rawlog'),
                'launch',
                'demo_rosbag.launch.py'
            ])
        ]),
        condition=IfCondition(include_demo_rosbag)
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_nav',
        namespace=node_namespace,
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    )

    # MRPT RBPF SLAM node
    slam_node = Node(
        package='mrpt_rbpf_slam',
        executable='mrpt_rbpf_slam',
        name='mrpt_rbpf_slam',
        namespace=node_namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'ini_filename': ini_filename,
                'global_frame_id': global_frame_id,
                'odom_frame_id': odom_frame_id,
                'base_frame_id': base_frame_id,
                'sensor_source': sensor_source,
            },
            config_file
        ],
        remappings=[
            # Add topic remappings here if needed
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        config_file_arg,
        ini_file_arg,
        global_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        sensor_source_arg,
        launch_rviz_arg,
        rviz_config_arg,
        include_demo_rosbag_arg,
        node_namespace_arg,
        # Actions
        set_rosconsole_config,
        include_demo_rosbag_action,
        rviz_node,
        slam_node,
    ])
