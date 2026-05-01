# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Test parameter handling."""

import unittest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class TestParameters(unittest.TestCase):
    """Test parameter declarations and defaults."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy."""
        rclpy.shutdown()

    def test_parameter_defaults(self):
        """Test that default parameters are reasonable."""
        node = Node('test_params')

        test_params = {
            'rawlog_play_delay': 0.1,
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'sensor_source': 'scan',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 5.0,
        }

        for param_name, default_value in test_params.items():
            node.declare_parameter(param_name, default_value)
            param = node.get_parameter(param_name)

            if isinstance(default_value, str):
                self.assertEqual(param.type_, Parameter.Type.STRING)
            elif isinstance(default_value, float):
                self.assertEqual(param.type_, Parameter.Type.DOUBLE)
            elif isinstance(default_value, int):
                self.assertEqual(param.type_, Parameter.Type.INTEGER)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
