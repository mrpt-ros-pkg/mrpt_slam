# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Test node can start without errors."""

import subprocess
import unittest

import rclpy


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
        result = subprocess.run(
            ['ros2', 'pkg', 'executables', 'mrpt_icp_slam_2d'],
            capture_output=True,
            text=True
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn('mrpt_icp_slam_2d', result.stdout)


if __name__ == '__main__':
    unittest.main()
