"""Test launch files can be loaded without errors.

Copyright (C) 2024-2026 Jose Luis Blanco-Claraco
Licensed under BSD-3-Clause
"""

import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


@pytest.fixture
def package_share_dir():
    """Get the package share directory."""
    return get_package_share_directory('mrpt_icp_slam_2d')


def test_icp_slam_launch(package_share_dir):
    """Test that icp_slam.launch.py can be loaded."""
    launch_file = os.path.join(package_share_dir, 'launch', 'icp_slam.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"

    # Try to include the launch file
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'include_demo_rosbag': 'false',
            'launch_rviz': 'false'
        }.items()
    )

    # Create a launch description with the include
    ld = LaunchDescription([include])
    assert ld is not None


def test_icp_slam_rawlog_launch(package_share_dir):
    """Test that icp_slam_rawlog.launch.py can be loaded."""
    launch_file = os.path.join(package_share_dir, 'launch', 'icp_slam_rawlog.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_rviz': 'false'
        }.items()
    )

    ld = LaunchDescription([include])
    assert ld is not None


def test_icp_slam_gui_launch(package_share_dir):
    """Test that icp_slam_gui.launch.py can be loaded."""
    launch_file = os.path.join(package_share_dir, 'launch', 'icp_slam_gui.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'include_demo_rosbag': 'false'
        }.items()
    )

    ld = LaunchDescription([include])
    assert ld is not None


def test_mvsim_icp_slam_launch(package_share_dir):
    """Test that mvsim_icp_slam.launch.py can be loaded."""
    launch_file = os.path.join(package_share_dir, 'launch', 'mvsim_icp_slam.launch.py')
    assert os.path.exists(launch_file), f"Launch file not found: {launch_file}"

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_rviz': 'false'
        }.items()
    )

    ld = LaunchDescription([include])
    assert ld is not None
