// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: options.cpp
 * Author: Vladislav Tananaev
 */

#include <mrpt_rbpf_slam/options.h>
#include <set>

namespace mrpt_rbpf_slam
{
namespace
{
using namespace mrpt::obs;

bool loadThrunModelParameters(
  rclcpp::Node::SharedPtr node,
  CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel & thrunModel)
{
  bool success = true;

    // Declare parameters with defaults
  node->declare_parameter("motion_model.thrun_motion_model_options.particle_count", 100);
  node->declare_parameter("motion_model.thrun_motion_model_options.alfa1_rot_rot", 0.1);
  node->declare_parameter("motion_model.thrun_motion_model_options.alfa2_rot_trans", 0.1);
  node->declare_parameter("motion_model.thrun_motion_model_options.alfa3_trans_trans", 0.1);
  node->declare_parameter("motion_model.thrun_motion_model_options.alfa4_trans_rot", 0.1);
  node->declare_parameter("motion_model.thrun_motion_model_options.additional_std_XY", 0.1);
  node->declare_parameter("motion_model.thrun_motion_model_options.additional_std_phi", 0.1);

    // Get parameters
  try {
    thrunModel.nParticlesCount =
      node->get_parameter("motion_model.thrun_motion_model_options.particle_count").as_int();
    thrunModel.alfa1_rot_rot =
      node->get_parameter("motion_model.thrun_motion_model_options.alfa1_rot_rot").as_double();
    thrunModel.alfa2_rot_trans =
      node->get_parameter("motion_model.thrun_motion_model_options.alfa2_rot_trans").as_double();
    thrunModel.alfa3_trans_trans =
      node->get_parameter("motion_model.thrun_motion_model_options.alfa3_trans_trans").as_double();
    thrunModel.alfa4_trans_rot =
      node->get_parameter("motion_model.thrun_motion_model_options.alfa4_trans_rot").as_double();
    thrunModel.additional_std_XY =
      node->get_parameter("motion_model.thrun_motion_model_options.additional_std_XY").as_double();
    thrunModel.additional_std_phi =
      node->get_parameter("motion_model.thrun_motion_model_options.additional_std_phi").as_double();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(node->get_logger(), "Parameter not declared: %s", e.what());
    success = false;
  }

  return success;
}

bool loadGaussianModelParameters(
  rclcpp::Node::SharedPtr node,
  CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel & gaussian_model)
{
  bool success = true;

    // Declare parameters with defaults
  node->declare_parameter("motion_model.gaussian_motion_model_options.a1", 0.034);
  node->declare_parameter("motion_model.gaussian_motion_model_options.a2", 0.057);
  node->declare_parameter("motion_model.gaussian_motion_model_options.a3", 0.014);
  node->declare_parameter("motion_model.gaussian_motion_model_options.a4", 0.097);
  node->declare_parameter("motion_model.gaussian_motion_model_options.minStdXY", 0.005);
  node->declare_parameter("motion_model.gaussian_motion_model_options.minStdPHI", 0.05);

    // Get parameters
  try {
    gaussian_model.a1 =
      node->get_parameter("motion_model.gaussian_motion_model_options.a1").as_double();
    gaussian_model.a2 =
      node->get_parameter("motion_model.gaussian_motion_model_options.a2").as_double();
    gaussian_model.a3 =
      node->get_parameter("motion_model.gaussian_motion_model_options.a3").as_double();
    gaussian_model.a4 =
      node->get_parameter("motion_model.gaussian_motion_model_options.a4").as_double();
    gaussian_model.minStdXY =
      node->get_parameter("motion_model.gaussian_motion_model_options.minStdXY").as_double();
    gaussian_model.minStdPHI =
      node->get_parameter("motion_model.gaussian_motion_model_options.minStdPHI").as_double();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(node->get_logger(), "Parameter not declared: %s", e.what());
    success = false;
  }

  return success;
}

bool loadMotionModelParameters(
  rclcpp::Node::SharedPtr node,
  CActionRobotMovement2D::TMotionModelOptions & motion_model_options)
{
  static const std::map<std::string,
    CActionRobotMovement2D::TDrawSampleMotionModel> motion_models = {
    {"thrun", CActionRobotMovement2D::mmThrun},
    {"gaussian", CActionRobotMovement2D::mmGaussian}
  };

  bool success = true;

  node->declare_parameter("motion_model.type", "thrun");
  std::string model_type = node->get_parameter("motion_model.type").as_string();

  if (!motion_models.count(model_type)) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
            "Specified motion model " << model_type << " is not supported.");
    return false;
  }

  motion_model_options.modelSelection = motion_models.at(model_type);
  success = success && loadThrunModelParameters(node, motion_model_options.thrunModel);
  success = success && loadGaussianModelParameters(node, motion_model_options.gaussianModel);

  return success;
}

bool loadVisualizationOptions(rclcpp::Node::SharedPtr node, PFslam::Options & options)
{
  bool success = true;

    // Declare parameters with defaults
  node->declare_parameter("mrpt_visualization_options.width", 600);
  node->declare_parameter("mrpt_visualization_options.height", 500);
  node->declare_parameter("mrpt_visualization_options.window_update_delay", 1);
  node->declare_parameter("mrpt_visualization_options.show_window", true);
  node->declare_parameter("mrpt_visualization_options.camera_follow_robot", false);

    // Get parameters
  try {
    options.PROGRESS_WINDOW_WIDTH_ =
      node->get_parameter("mrpt_visualization_options.width").as_int();
    options.PROGRESS_WINDOW_HEIGHT_ =
      node->get_parameter("mrpt_visualization_options.height").as_int();
    options.SHOW_PROGRESS_IN_WINDOW_DELAY_MS_ =
      node->get_parameter("mrpt_visualization_options.window_update_delay").as_int();
    options.SHOW_PROGRESS_IN_WINDOW_ =
      node->get_parameter("mrpt_visualization_options.show_window").as_bool();
    options.CAMERA_3DSCENE_FOLLOWS_ROBOT_ =
      node->get_parameter("mrpt_visualization_options.camera_follow_robot").as_bool();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(node->get_logger(), "Parameter not declared: %s", e.what());
    success = false;
  }

  return success;
}
}  // namespace

bool loadOptions(rclcpp::Node::SharedPtr node, PFslam::Options & options)
{
  bool success = true;

  success = success && loadMotionModelParameters(node, options.motion_model_options_);
  success = success && loadVisualizationOptions(node, options);

  node->declare_parameter("simplemap_save_folder", "/tmp/");
  try {
    options.simplemap_path_prefix =
      node->get_parameter("simplemap_save_folder").as_string();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_ERROR(node->get_logger(), "Parameter not declared: %s", e.what());
    success = false;
  }

  return success;
}

}  // namespace mrpt_rbpf_slam
