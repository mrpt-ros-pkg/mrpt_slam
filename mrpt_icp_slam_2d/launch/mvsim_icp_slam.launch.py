# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for ICP SLAM with MVSim simulator."""

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
    """Generate launch description for ICP SLAM with MVSim."""
    # Get package directories
    mrpt_icp_slam_2d_share = FindPackageShare('mrpt_icp_slam_2d')
    mvsim_share = FindPackageShare('mvsim')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (should be true for MVSim)'
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

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            mrpt_icp_slam_2d_share, 'rviz', 'rviz_conf.rviz'
        ]),
        description='Full path to the RViz configuration file'
    )

    mvsim_world_arg = DeclareLaunchArgument(
        'mvsim_world',
        default_value='demo_2robots.world.xml',
        description='MVSim world file name'
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
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    mvsim_world = LaunchConfiguration('mvsim_world')

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_icp_slam_2d_share, 'config', 'rosconsole.config'])
    )

    # Include MVSim launch file
    # Note: Adjust the launch file name based on actual mvsim package structure
    include_mvsim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mvsim_share, 'launch', 'mvsim.launch.py'])
        ]),
        launch_arguments={
            'world_file': mvsim_world,
            'use_sim_time': use_sim_time,
        }.items()
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

    # MRPT ICP SLAM node
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
        launch_rviz_arg,
        rviz_config_arg,
        mvsim_world_arg,
        # Actions
        set_rosconsole_config,
        include_mvsim,
        rviz_node,
        slam_node,
    ])
