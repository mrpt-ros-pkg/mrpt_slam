# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Smoke tests for parameter and config file validation."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory

import yaml


class TestParameters(unittest.TestCase):
    """Test parameter and configuration files."""

    def test_ini_files_exist(self):
        """Test that required .ini configuration files exist."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        ini_files = [
            'ros_laser_odometry.ini',
            'ros_odometry_2DRangeScans.ini',
            'ros_odometry_2DRangeScans_LC.ini',
        ]

        for ini_file in ini_files:
            ini_path = os.path.join(pkg_dir, 'config', ini_file)
            self.assertTrue(
                os.path.exists(ini_path),
                f'INI file not found: {ini_path}'
            )

    def test_ini_files_not_empty(self):
        """Test that .ini files are not empty."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        config_dir = os.path.join(pkg_dir, 'config')

        for filename in os.listdir(config_dir):
            if filename.endswith('.ini'):
                full_path = os.path.join(config_dir, filename)
                file_size = os.path.getsize(full_path)
                self.assertGreater(
                    file_size, 0,
                    f'INI file is empty: {full_path}'
                )

    def test_rviz_configs_exist(self):
        """Test that RViz configuration files exist."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'sr_graphslam.rviz')
        self.assertTrue(
            os.path.exists(rviz_config),
            f'RViz config not found: {rviz_config}'
        )

    def test_rviz_config_is_valid_yaml(self):
        """Test that RViz config file is valid YAML."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'sr_graphslam.rviz')

        try:
            with open(rviz_config, 'r') as f:
                config = yaml.safe_load(f)
            self.assertIsNotNone(config)
        except ImportError:
            self.assertTrue(os.path.exists(rviz_config))
        except Exception as e:  # noqa: B902
            self.fail(f'RViz config has error: {str(e)}')


if __name__ == '__main__':
    unittest.main()
