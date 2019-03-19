/*
 * File: options.h
 * Author: Vladislav Tananaev
 */
#pragma once
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>
#include <ros/node_handle.h>

namespace mrpt_rbpf_slam{
bool loadOptions(const ros::NodeHandle& nh, PFslam::Options& options);
}
