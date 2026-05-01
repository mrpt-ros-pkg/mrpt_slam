# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Integration test: EKF SLAM 2D rawlog playback.

Launches the node with the bundled demo rawlog and asserts that at least
one MarkerArray is published on /state_viz within the timeout.

This test is slow (up to 90 s) and is only registered when the package is
built with -DENABLE_INTEGRATION_TESTS=ON.
"""

import os
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.node
from visualization_msgs.msg import MarkerArray


PKG = 'mrpt_ekf_slam_2d'
TIMEOUT_S = 90.0
# Keep a small delay so DDS discovery completes before most messages
# are published.  The default (0.1 s) is used; with the demo rawlog
# (~few-dozen steps) the total replay takes only a handful of seconds.
RAWLOG_PLAY_DELAY = '0.1'


class TestEKFSlamRawlogIntegration(unittest.TestCase):
    """Integration tests for EKF SLAM 2D rawlog playback."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        rclpy.shutdown()

    def test_state_viz_published_during_rawlog_replay(self):
        """Node plays the demo rawlog and publishes at least one /state_viz."""
        pkg_share = get_package_share_directory(PKG)
        ini_path = os.path.join(pkg_share, 'tutorial', 'kf-slam_demo_2d.ini')
        rawlog_path = os.path.join(
            pkg_share, 'tutorial', 'kf-slam_demo_2d.rawlog'
        )

        self.assertTrue(
            os.path.exists(ini_path),
            f'ini file not found: {ini_path}',
        )
        self.assertTrue(
            os.path.exists(rawlog_path),
            f'rawlog file not found: {rawlog_path}',
        )

        received = threading.Event()
        node = rclpy.create_node('test_ekf_slam_integration')
        node.create_subscription(
            MarkerArray,
            '/state_viz',
            lambda _msg: received.set(),
            10,
        )

        # Spin in a background thread so messages are processed while we wait.
        spin_thread = threading.Thread(
            target=lambda: rclpy.spin(node), daemon=True
        )
        spin_thread.start()

        # Give DDS a moment to complete discovery before the SLAM node starts
        # publishing.
        time.sleep(1.0)

        proc = subprocess.Popen(
            [
                'ros2',
                'run',
                PKG,
                PKG,
                '--ros-args',
                '-p', f'ini_filename:={ini_path}',
                '-p', f'rawlog_filename:={rawlog_path}',
                '-p', f'rawlog_play_delay:={RAWLOG_PLAY_DELAY}',
                # sensor_source must be non-empty to pass init(); it is not
                # used in rawlog playback mode but init() checks for it.
                '-p', 'sensor_source:=landmark',
            ],
        )

        try:
            received.wait(timeout=TIMEOUT_S)
            self.assertTrue(
                received.is_set(),
                f'/state_viz received no MarkerArray within {TIMEOUT_S} s',
            )
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
