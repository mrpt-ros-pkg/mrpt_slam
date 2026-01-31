"""Smoke tests for parameter loading and validation."""

import os
import unittest
from ament_index_python.packages import get_package_share_directory
import yaml


class TestParameters(unittest.TestCase):
    """Test parameter files can be loaded."""

    def test_default_yaml_exists(self):
        """Test that default.yaml parameter file exists."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        yaml_path = os.path.join(pkg_dir, 'config', 'default.yaml')
        self.assertTrue(
            os.path.exists(yaml_path),
            f"Parameter file not found: {yaml_path}"
        )

    def test_default_yaml_valid(self):
        """Test that default.yaml is valid YAML."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        yaml_path = os.path.join(pkg_dir, 'config', 'default.yaml')

        try:
            with open(yaml_path, 'r') as f:
                params = yaml.safe_load(f)
            self.assertIsNotNone(params)
        except yaml.YAMLError as e:
            self.fail(f"Invalid YAML in default.yaml: {str(e)}")
        except Exception as e:
            self.fail(f"Failed to load default.yaml: {str(e)}")

    def test_ini_files_exist(self):
        """Test that required .ini configuration files exist."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        ini_files = [
            'grid_slam_demo.ini',
            'RO-SLAM_demo.ini',
        ]

        for ini_file in ini_files:
            ini_path = os.path.join(pkg_dir, 'tutorial', ini_file)
            self.assertTrue(
                os.path.exists(ini_path),
                f"INI file not found: {ini_path}"
            )

    def test_rviz_config_exists(self):
        """Test that RViz configuration file exists."""
        pkg_dir = get_package_share_directory('mrpt_rbpf_slam')
        rviz_path = os.path.join(pkg_dir, 'rviz', 'rviz_conf.rviz')
        self.assertTrue(
            os.path.exists(rviz_path),
            f"RViz config not found: {rviz_path}"
        )


if __name__ == '__main__':
    unittest.main()
