# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Launch file integration tests for MRPT RBPF SLAM."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml


class TestLaunchFiles(unittest.TestCase):
    """Test that launch files can be loaded and parsed correctly."""

    def _test_launch_file(self, launch_file_name):
        """Test if a launch file loads correctly."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
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
        """Test that all launch files exist."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        launch_dir = os.path.join(pkg_dir, 'launch')

        expected_launch_files = [
            'rbpf_slam.launch.py',
            'rviz.launch.py',
            'rbpf_slam_rawlog.launch.py',
            'rbpf_slam_turtlebot3.launch.py',
            'ro_slam.launch.py',
            'ro_slam_rawlog.launch.py',
            'mvsim_slam.launch.py',
        ]

        for launch_file in expected_launch_files:
            full_path = os.path.join(launch_dir, launch_file)
            self.assertTrue(
                os.path.exists(full_path),
                f'Launch file {launch_file} does not exist at {full_path}'
            )

    def test_launch_files_are_valid_python(self):
        """Test that all launch files are valid Python syntax."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
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
        """Test that all launch files have generate_launch_description."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
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

    def test_rviz_launch(self):
        """Test rviz.launch.py can be loaded."""
        self._test_launch_file('rviz.launch.py')

    def test_rbpf_slam_turtlebot3_launch(self):
        """Test rbpf_slam_turtlebot3.launch.py can be loaded."""
        self._test_launch_file('rbpf_slam_turtlebot3.launch.py')

    def test_rbpf_slam_rawlog_launch(self):
        """Test rbpf_slam_rawlog.launch.py can be loaded."""
        self._test_launch_file('rbpf_slam_rawlog.launch.py')

    def test_ro_slam_rawlog_launch(self):
        """Test ro_slam_rawlog.launch.py can be loaded."""
        self._test_launch_file('ro_slam_rawlog.launch.py')

    def test_config_files_exist(self):
        """Test that required config files exist."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')

        config_files = [
            os.path.join(pkg_dir, 'config', 'default.yaml'),
            os.path.join(pkg_dir, 'rviz', 'rviz_conf.rviz'),
        ]

        for config_file in config_files:
            self.assertTrue(
                os.path.exists(config_file),
                f'Config file does not exist: {config_file}'
            )

    def test_rviz_config_is_valid(self):
        """Test that RViz config file is valid YAML."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'rviz_conf.rviz')

        try:
            with open(rviz_config, 'r') as f:
                config = yaml.safe_load(f)
            self.assertIsNotNone(config)
            self.assertIn('Visualization Manager', config)
        except ImportError:
            self.assertTrue(os.path.exists(rviz_config))
        except yaml.YAMLError as e:
            self.fail(f'RViz config has YAML error: {str(e)}')


if __name__ == '__main__':
    unittest.main()
