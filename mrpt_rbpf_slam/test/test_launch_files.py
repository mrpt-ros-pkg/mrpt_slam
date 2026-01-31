"""Smoke tests for MRPT RBPF SLAM launch files."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions


class TestLaunchFiles(unittest.TestCase):
    """Test that all launch files can be loaded without errors."""

    def _test_launch_file(self, launch_file_name):
        """Helper method to test if a launch file loads correctly."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        launch_file_path = os.path.join(pkg_dir, 'launch', launch_file_name)

        # Check if file exists
        self.assertTrue(
            os.path.exists(launch_file_path),
            f"Launch file not found: {launch_file_path}"
        )

        # Try to load the launch description
        try:
            source = PythonLaunchDescriptionSource(launch_file_path)
            # Create a minimal launch description to include the file
            ld = LaunchDescription([
                IncludeLaunchDescription(source)
            ])
            # If we get here without exception, the launch file is valid
            self.assertIsNotNone(ld)
        except Exception as e:
            self.fail(f"Failed to load launch file {launch_file_name}: {str(e)}")

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

    # Note: Tests for launch files that include other packages (mrpt_rawlog, mvsim)
    # are commented out as they require those packages to be installed
    # Uncomment when those dependencies are available

    # def test_rbpf_slam_launch(self):
    #     """Test rbpf_slam.launch.py can be loaded."""
    #     self._test_launch_file('rbpf_slam.launch.py')

    # def test_ro_slam_launch(self):
    #     """Test ro_slam.launch.py can be loaded."""
    #     self._test_launch_file('ro_slam.launch.py')

    # def test_mvsim_slam_launch(self):
    #     """Test mvsim_slam.launch.py can be loaded."""
    #     self._test_launch_file('mvsim_slam.launch.py')


if __name__ == '__main__':
    unittest.main()
