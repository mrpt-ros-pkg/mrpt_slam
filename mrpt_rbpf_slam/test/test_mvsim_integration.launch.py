# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch test for the RBPF SLAM and MVSim integration."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions
import rclpy


def generate_test_description():
    """Start the real MVSim integration launch without graphical processes."""
    package_share = get_package_share_directory('mrpt_rbpf_slam')
    launch_file = os.path.join(package_share, 'launch', 'mvsim_slam.launch.py')
    integration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_rviz': 'false',
            'mvsim_headless': 'true',
        }.items()
    )

    return LaunchDescription([
        integration,
        TimerAction(
            period=5.0,
            actions=[launch_testing.actions.ReadyToTest()]
        ),
    ])


class TestMVSimRBPFIntegration(unittest.TestCase):
    """Verify that MVSim and RBPF SLAM remain alive concurrently."""

    def test_simulator_and_slam_remain_alive(self):
        """Check both nodes after startup and again after a stability interval."""
        rclpy.init()
        node = rclpy.create_node('test_mvsim_rbpf_integration')
        expected_nodes = {'/mvsim_simulator', '/mrpt_rbpf_slam'}

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
