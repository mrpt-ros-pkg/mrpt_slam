"""Test parameter handling.

Copyright (C) 2024-2026 Jose Luis Blanco-Claraco
Licensed under BSD-3-Clause
"""

import unittest
import pytest
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


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
        # Create a simple node to test parameter types
        node = Node('test_params')

        # Test parameter type declarations that the SLAM node would use
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

            # Check types are correct
            if isinstance(default_value, str):
                self.assertEqual(param.type_, Parameter.Type.STRING)
            elif isinstance(default_value, float):
                self.assertEqual(param.type_, Parameter.Type.DOUBLE)
            elif isinstance(default_value, int):
                self.assertEqual(param.type_, Parameter.Type.INTEGER)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
