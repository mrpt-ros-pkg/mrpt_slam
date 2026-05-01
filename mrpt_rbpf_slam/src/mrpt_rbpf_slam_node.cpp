// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
        // Initialize ROS2
  rclcpp::init(argc, argv);

        // Create SLAM node with default options
  auto slam_node = std::make_shared<mrpt_rbpf_slam::PFslamWrapper>();

        // Initialize parameters and SLAM
  if (!slam_node->getParams() || !slam_node->init()) {
    RCLCPP_ERROR(slam_node->get_logger(), "Failed to initialize SLAM");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

        // If rawlog playback mode, play and exit
  if (slam_node->rawlogPlay()) {
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }

        // Main loop with executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(slam_node);

        // Spin - callbacks will be invoked by subscribers
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
