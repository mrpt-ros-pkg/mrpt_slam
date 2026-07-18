# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file for RBPF SLAM with MVSim simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RBPF SLAM with MVSim simulator."""
    # Get package directories
    mrpt_rbpf_slam_share = FindPackageShare('mrpt_rbpf_slam')
    mvsim_share = FindPackageShare('mvsim')

    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            mvsim_share, 'mvsim_tutorial', 'mvsim_slam.world.xml'
        ]),
        description='Path to the MVSim world file'
    )

    mvsim_headless_arg = DeclareLaunchArgument(
        'mvsim_headless',
        default_value='false',
        description='Run MVSim without its graphical window'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # MVSim simulator node
    mvsim_node = Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim_simulator',
        output='screen',
        parameters=[{
            'world_file': LaunchConfiguration('world_file'),
            'headless': LaunchConfiguration('mvsim_headless'),
            'do_fake_localization': False,  # Needed to run an external localization / SLAM system
            'publish_tf_odom2baselink': True,
            'force_publish_vehicle_namespace': False,
        }]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            mvsim_share, 'mvsim_tutorial', 'mvsim_slam.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # Set ROS console configuration
    set_rosconsole_config = SetEnvironmentVariable(
        name='ROSCONSOLE_CONFIG_FILE',
        value=PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'rosconsole.config'])
    )

    # MRPT RBPF SLAM node
    slam_node = Node(
        package='mrpt_rbpf_slam',
        executable='mrpt_rbpf_slam',
        name='mrpt_rbpf_slam',
        output='screen',
        parameters=[
            {
                'ini_filename': PathJoinSubstitution([
                    mrpt_rbpf_slam_share, 'tutorial', 'grid_slam_demo.ini'
                ]),
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_link',
                'sensor_source': 'laser1',
            },
            PathJoinSubstitution([mrpt_rbpf_slam_share, 'config', 'default.yaml'])
        ]
    )

    return LaunchDescription([
        world_file_arg,
        mvsim_headless_arg,
        launch_rviz_arg,
        mvsim_node,
        rviz_node,
        set_rosconsole_config,
        slam_node,
    ])
