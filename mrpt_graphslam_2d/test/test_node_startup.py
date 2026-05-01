# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Smoke tests for MRPT GraphSLAM 2D node startup."""

import subprocess
import unittest

import rclpy
from rclpy.node import Node


class TestNodeStartup(unittest.TestCase):
    """Test that ROS 2 node infrastructure is working."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        rclpy.shutdown()

    def test_rclpy_initialization(self):
        """Test that rclpy can be initialized."""
        self.assertTrue(rclpy.ok())

    def test_create_test_node(self):
        """Test that a basic node can be created."""
        try:
            test_node = Node('test_node')
            self.assertIsNotNone(test_node)
            self.assertEqual(test_node.get_name(), 'test_node')
            test_node.destroy_node()
        except Exception as e:  # noqa: B902
            self.fail(f'Failed to create test node: {str(e)}')

    def test_node_namespace(self):
        """Test node namespace handling."""
        test_node = Node('test_ns_node', namespace='test')
        self.assertEqual(test_node.get_namespace(), '/test')
        test_node.destroy_node()

    def _assert_executable_discoverable(self, executable_name):
        """Assert that a ros2 pkg executable is listed for this package."""
        result = subprocess.run(
            ['ros2', 'pkg', 'executables', 'mrpt_graphslam_2d'],
            capture_output=True, text=True, timeout=10
        )
        self.assertIn(
            executable_name, result.stdout,
            f"Executable '{executable_name}' not found in "
            f"'ros2 pkg executables mrpt_graphslam_2d' output:\n"
            f'{result.stdout}'
        )

    def test_single_robot_node_discoverable(self):
        """Test that mrpt_graphslam_2d_node is discoverable."""
        self._assert_executable_discoverable('mrpt_graphslam_2d_node')

    def test_multi_robot_node_discoverable(self):
        """Test that mrpt_graphslam_2d_mr_node is discoverable."""
        self._assert_executable_discoverable('mrpt_graphslam_2d_mr_node')

    def test_map_merger_node_discoverable(self):
        """Test that map_merger_node is discoverable."""
        self._assert_executable_discoverable('map_merger_node')


if __name__ == '__main__':
    unittest.main()
