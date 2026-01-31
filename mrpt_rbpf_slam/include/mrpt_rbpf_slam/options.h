/*
 * File: options.h
 * Author: Vladislav Tananaev
 */
#pragma once
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>
#include <rclcpp/rclcpp.hpp>

namespace mrpt_rbpf_slam{
bool loadOptions(rclcpp::Node::SharedPtr node, PFslam::Options& options);
}
