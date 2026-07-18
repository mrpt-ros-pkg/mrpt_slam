# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch test for the ICP SLAM and MVSim integration."""

import atexit
import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions
import pytest
import rclpy


def _make_headless_ini(package_share):
    """Copy the demo INI with its separate MRPT window disabled."""
    ini_path = os.path.join(package_share, 'tutorial', 'icp_slam_demo.ini')
    with open(ini_path, encoding='utf-8') as ini_file:
        ini_contents = ini_file.read()

    gui_setting = 'SHOW_PROGRESS_3D_REAL_TIME=1'
    if gui_setting not in ini_contents:
        raise RuntimeError(f'Missing expected GUI setting in {ini_path}')

    with tempfile.NamedTemporaryFile(
        mode='w', prefix='icp_slam_headless_', suffix='.ini', delete=False
    ) as temporary_ini:
        temporary_ini.write(ini_contents.replace(
            gui_setting, 'SHOW_PROGRESS_3D_REAL_TIME=0', 1
        ))

    atexit.register(os.unlink, temporary_ini.name)
    return temporary_ini.name


@pytest.mark.launch_test
def generate_test_description():
    """Start the real MVSim integration launch without graphical processes."""
    package_share = get_package_share_directory('mrpt_icp_slam_2d')
    mvsim_share = get_package_share_directory('mvsim')
    launch_file = os.path.join(
        package_share, 'launch', 'mvsim_icp_slam.launch.py'
    )
    integration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'ini_filename': _make_headless_ini(package_share),
            'launch_rviz': 'false',
            'mvsim_headless': 'true',
            'mvsim_world': os.path.join(
                mvsim_share, 'mvsim_tutorial', 'mvsim_slam.world.xml'
            ),
        }.items()
    )

    return LaunchDescription([
        integration,
        TimerAction(
            period=5.0,
            actions=[launch_testing.actions.ReadyToTest()]
        ),
    ])


class TestMVSimICPIntegration(unittest.TestCase):
    """Verify that MVSim and ICP SLAM remain alive concurrently."""

    def test_simulator_and_slam_remain_alive(self):
        """Check both nodes after startup and again after a stability interval."""
        rclpy.init()
        node = rclpy.create_node('test_mvsim_icp_integration')
        expected_nodes = {'/mvsim', '/mrpt_icp_slam_2d'}

        try:
            deadline = time.monotonic() + 10.0
            discovered_nodes = set()
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                discovered_nodes = {
                    f'{namespace.rstrip("/")}/{name}'
                    for name, namespace in node.get_node_names_and_namespaces()
                }
                if expected_nodes <= discovered_nodes:
                    break

            self.assertTrue(
                expected_nodes <= discovered_nodes,
                f'Expected nodes not discovered: {expected_nodes - discovered_nodes}'
            )

            time.sleep(3.0)
            rclpy.spin_once(node, timeout_sec=0.2)
            surviving_nodes = {
                f'{namespace.rstrip("/")}/{name}'
                for name, namespace in node.get_node_names_and_namespaces()
            }
            self.assertTrue(
                expected_nodes <= surviving_nodes,
                f'Nodes exited during stability interval: {expected_nodes - surviving_nodes}'
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
