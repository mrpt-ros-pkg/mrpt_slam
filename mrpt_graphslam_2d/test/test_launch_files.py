# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file integration tests for MRPT GraphSLAM 2D."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


class TestLaunchFiles(unittest.TestCase):
    """Test that launch files can be loaded and parsed correctly."""

    def _test_launch_file(self, launch_file_name):
        """Test if a launch file loads correctly."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        launch_file_path = os.path.join(pkg_dir, 'launch', launch_file_name)

        self.assertTrue(
            os.path.exists(launch_file_path),
            f'Launch file not found: {launch_file_path}'
        )

        try:
            source = PythonLaunchDescriptionSource(launch_file_path)
            ld = LaunchDescription([
                IncludeLaunchDescription(source)
            ])
            self.assertIsNotNone(ld)
        except Exception as e:  # noqa: B902
            self.fail(
                f'Failed to load launch file {launch_file_name}: {str(e)}'
            )

    def test_launch_files_exist(self):
        """Test that all ROS 2 launch files exist."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        expected_launch_files = [
            'graphslam.launch.py',
            'sr_graphslam_demo.launch.py',
            'mr_graphslam.launch.py',
        ]

        for launch_file in expected_launch_files:
            full_path = os.path.join(launch_dir, launch_file)
            self.assertTrue(
                os.path.exists(full_path),
                f'Launch file {launch_file} does not exist at {full_path}'
            )

    def test_launch_files_are_valid_python(self):
        """Test that all .launch.py files are valid Python syntax."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        for filename in os.listdir(launch_dir):
            if filename.endswith('.launch.py'):
                full_path = os.path.join(launch_dir, filename)
                with open(full_path, 'r') as f:
                    code = f.read()
                try:
                    compile(code, full_path, 'exec')
                except SyntaxError as e:
                    self.fail(
                        f'Launch file {filename} has syntax error: {str(e)}'
                    )

    def test_launch_files_have_generate_function(self):
        """Test that all .launch.py files have generate_launch_description."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        for filename in os.listdir(launch_dir):
            if filename.endswith('.launch.py'):
                full_path = os.path.join(launch_dir, filename)
                with open(full_path, 'r') as f:
                    code = f.read()
                self.assertIn(
                    'generate_launch_description',
                    code,
                    f'Launch file {filename} missing generate_launch_description'
                )

    def test_graphslam_launch(self):
        """Test graphslam.launch.py can be loaded."""
        self._test_launch_file('graphslam.launch.py')

    def test_mr_graphslam_launch(self):
        """Test mr_graphslam.launch.py can be loaded."""
        self._test_launch_file('mr_graphslam.launch.py')

    def test_mr_config_files_exist(self):
        """Test that MR config files referenced by mr_graphslam.launch.py exist."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        mr_config = os.path.join(
            pkg_dir, 'config', 'ros_odometry_2DRangeScans_LC_MR_real.ini'
        )
        self.assertTrue(
            os.path.exists(mr_config),
            f'MR config file does not exist: {mr_config}'
        )

    def test_config_files_exist(self):
        """Test that required config files exist."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')

        config_files = [
            os.path.join(pkg_dir, 'config', 'ros_laser_odometry.ini'),
            os.path.join(pkg_dir, 'config', 'ros_odometry_2DRangeScans.ini'),
        ]

        for config_file in config_files:
            self.assertTrue(
                os.path.exists(config_file),
                f'Config file does not exist: {config_file}'
            )

    def test_rviz_config_exists(self):
        """Test that RViz config file exists."""
        pkg_dir = get_package_share_directory('mrpt_graphslam_2d')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'sr_graphslam.rviz')
        self.assertTrue(
            os.path.exists(rviz_config),
            f'RViz config not found: {rviz_config}'
        )


if __name__ == '__main__':
    unittest.main()
