"""Smoke tests for parameter loading and validation."""

import os
import unittest
from ament_index_python.packages import get_package_share_directory
import yaml


class TestParameters(unittest.TestCase):
    """Test parameter files can be loaded."""

    def test_ini_files_exist(self):
        """Test that required .ini configuration files exist."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        ini_files = [
            'kf-slam_6D_demo.ini',
        ]

        for ini_file in ini_files:
            ini_path = os.path.join(pkg_dir, 'tutorial', ini_file)
            self.assertTrue(
                os.path.exists(ini_path),
                f"INI file not found: {ini_path}"
            )

    def test_rviz_config_exists(self):
        """Test that RViz configuration file exists."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        rviz_path = os.path.join(pkg_dir, 'rviz', 'rviz_conf_ekf_3d.rviz')
        self.assertTrue(
            os.path.exists(rviz_path),
            f"RViz config not found: {rviz_path}"
        )

    def test_rawlog_file_exists(self):
        """Test that demo rawlog file exists."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        rawlog_path = os.path.join(
            pkg_dir, 'tutorial', 'kf-slam_6D_demo.rawlog'
        )
        self.assertTrue(
            os.path.exists(rawlog_path),
            f"Rawlog file not found: {rawlog_path}"
        )


if __name__ == '__main__':
    unittest.main()
