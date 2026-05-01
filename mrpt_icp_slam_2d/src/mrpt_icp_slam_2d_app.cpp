// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: mrpt_icp_slam_2d_app.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
        // Initialize ROS2
  rclcpp::init(argc, argv);

        // Create SLAM node with default options
  auto slam_node = std::make_shared<mrpt_icp_slam_2d::ICPslamWrapper>();

        // Initialize parameters and SLAM
  slam_node->get_param();
  slam_node->init();

  RCLCPP_INFO_STREAM(slam_node->get_logger(), "About to enter the main spin loop.");

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
