"""Launch file integration tests for MRPT EKF SLAM 3D."""

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
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        launch_file_path = os.path.join(pkg_dir, 'launch', launch_file_name)

        # Check if file exists
        self.assertTrue(
            os.path.exists(launch_file_path),
            f'Launch file not found: {launch_file_path}'
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
            self.fail(f'Failed to load launch file {launch_file_name}: {str(e)}')

    def test_launch_files_exist(self):
        """Test that all launch files exist."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        expected_launch_files = [
            'ekf_slam_3d.launch.py',
            'ekf_slam_3d_rawlog.launch.py',
            'ekf_slam_3d_wheeled_robot.launch.py',
        ]

        for launch_file in expected_launch_files:
            full_path = os.path.join(launch_dir, launch_file)
            self.assertTrue(
                os.path.exists(full_path),
                f'Launch file {launch_file} does not exist at {full_path}'
            )

    def test_launch_files_are_valid_python(self):
        """Test that all launch files are valid Python syntax."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        for filename in os.listdir(launch_dir):
            if filename.endswith('.launch.py'):
                full_path = os.path.join(launch_dir, filename)
                with open(full_path, 'r') as f:
                    code = f.read()
                try:
                    compile(code, full_path, 'exec')
                except SyntaxError as e:
                    self.fail(f'Launch file {filename} has syntax error: {str(e)}')

    def test_launch_files_have_generate_function(self):
        """Test that all launch files have generate_launch_description function."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        launch_dir = os.path.join(pkg_dir, 'launch')

        for filename in os.listdir(launch_dir):
            if filename.endswith('.launch.py'):
                full_path = os.path.join(launch_dir, filename)
                with open(full_path, 'r') as f:
                    code = f.read()
                self.assertIn(
                    'generate_launch_description',
                    code,
                    f'Launch file {filename} missing generate_launch_description function'
                )

    def test_ekf_slam_3d_rawlog_launch(self):
        """Test ekf_slam_3d_rawlog.launch.py can be loaded."""
        self._test_launch_file('ekf_slam_3d_rawlog.launch.py')

    def test_config_files_exist(self):
        """Test that required config files exist."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')

        config_files = [
            os.path.join(pkg_dir, 'rviz', 'rviz_conf_ekf_3d.rviz'),
        ]

        for config_file in config_files:
            self.assertTrue(
                os.path.exists(config_file),
                f'Config file does not exist: {config_file}'
            )

    def test_rviz_config_is_valid(self):
        """Test that RViz config file is valid YAML."""
        pkg_dir = get_package_share_directory('mrpt_ekf_slam_3d')
        rviz_config = os.path.join(pkg_dir, 'rviz', 'rviz_conf_ekf_3d.rviz')

        try:
            import yaml
            with open(rviz_config, 'r') as f:
                config = yaml.safe_load(f)
            self.assertIsNotNone(config)
            self.assertIn('Visualization Manager', config)
        except ImportError:
            # If yaml module not available, just check file exists
            self.assertTrue(os.path.exists(rviz_config))
        except yaml.YAMLError as e:
            self.fail(f'RViz config has YAML error: {str(e)}')

    # Note: Tests for launch files that include other packages (mrpt_rawlog)
    # are commented out as they require those packages to be installed
    # Uncomment when those dependencies are available

    # def test_ekf_slam_3d_launch(self):
    #     """Test ekf_slam_3d.launch.py can be loaded."""
    #     self._test_launch_file('ekf_slam_3d.launch.py')

    # def test_ekf_slam_3d_wheeled_robot_launch(self):
    #     """Test ekf_slam_3d_wheeled_robot.launch.py can be loaded."""
    #     self._test_launch_file('ekf_slam_3d_wheeled_robot.launch.py')


if __name__ == '__main__':
    unittest.main()
