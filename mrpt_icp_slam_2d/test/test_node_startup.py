"""Test node can start without errors.

Copyright (C) 2024-2026 Jose Luis Blanco-Claraco
Licensed under BSD-3-Clause
"""

import unittest
import pytest
import rclpy
from rclpy.node import Node
import time


class TestNodeStartup(unittest.TestCase):
    """Test the SLAM node can be initialized."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy."""
        rclpy.shutdown()

    def test_node_exists(self):
        """Test that the node executable exists and can be found."""
        # This is a basic smoke test
        # A more complete test would actually launch the node
        # but that requires launch_testing framework
        import subprocess
        result = subprocess.run(
            ['ros2', 'pkg', 'executables', 'mrpt_icp_slam_2d'],
            capture_output=True,
            text=True
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('mrpt_icp_slam_2d', result.stdout)


if __name__ == '__main__':
    unittest.main()
