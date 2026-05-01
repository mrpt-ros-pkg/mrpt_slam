// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: options.h
 * Author: Vladislav Tananaev
 */
#pragma once
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>
#include <rclcpp/rclcpp.hpp>

namespace mrpt_rbpf_slam
{
bool loadOptions(rclcpp::Node::SharedPtr node, PFslam::Options & options);
}
