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

bool loadThrunModelParameters(const ros::NodeHandle& nh,
                              CActionRobotMovement2D::TMotionModelOptions::TOptions_ThrunModel& thrunModel)
{
  bool success = true;
  ros::NodeHandle sub_nh(nh, "thrun_motion_model_options");
  auto getParam = [&success, &sub_nh](const std::string param, auto& val) {
    success = success && sub_nh.getParam(param, val);
  };

  thrunModel.nParticlesCount = sub_nh.param<int>("particle_count", 100);
  getParam("alfa1_rot_rot", thrunModel.alfa1_rot_rot);
  getParam("alfa2_rot_trans", thrunModel.alfa2_rot_trans);
  getParam("alfa3_trans_trans", thrunModel.alfa3_trans_trans);
  getParam("alfa4_trans_rot", thrunModel.alfa4_trans_rot);
  getParam("additional_std_XY", thrunModel.additional_std_XY);
  getParam("additional_std_phi", thrunModel.additional_std_phi);
  return success;
}

bool loadGaussianModelParameters(const ros::NodeHandle& nh,
                                 CActionRobotMovement2D::TMotionModelOptions::TOptions_GaussianModel& gaussian_model)
{
  bool success = true;
  ros::NodeHandle sub_nh(nh, "gaussian_motion_model_options");
  auto getParam = [&success, &sub_nh](const std::string param, auto& val) {
    success = success && sub_nh.getParam(param, val);
  };

  getParam("a1", gaussian_model.a1);
  getParam("a2", gaussian_model.a2);
  getParam("a3", gaussian_model.a3);
  getParam("a4", gaussian_model.a4);
  getParam("minStdXY", gaussian_model.minStdXY);
  getParam("minStdPHI", gaussian_model.minStdPHI);
  return success;
}

bool loadMotionModelParameters(const ros::NodeHandle& nh,
                               CActionRobotMovement2D::TMotionModelOptions& motion_model_options)
{
  static const std::map<std::string, CActionRobotMovement2D::TDrawSampleMotionModel> motion_models = {
    { "thrun", CActionRobotMovement2D::mmThrun }, { "gaussian", CActionRobotMovement2D::mmGaussian }
  };

  bool success = true;
  ros::NodeHandle sub_nh(nh, "motion_model");
  auto getParam = [&success, &sub_nh](const std::string param, auto& val) {
    success = success && sub_nh.getParam(param, val);
  };

  std::string model_type;
  getParam("type", model_type);
  if (!motion_models.count(model_type))
  {
    ROS_ERROR_STREAM("Specified motion model " << model_type << " is not supported.");
    return false;
  }
  motion_model_options.modelSelection = motion_models.at(model_type);
  success = success && loadThrunModelParameters(sub_nh, motion_model_options.thrunModel);
  success = success && loadGaussianModelParameters(sub_nh, motion_model_options.gaussianModel);
  return success;
}

bool loadVisualizationOptions(const ros::NodeHandle& nh, PFslam::Options& options)
{
  bool success = true;
  ros::NodeHandle sub_nh(nh, "mrpt_visualization_options");
  success = success && sub_nh.getParam("width", options.PROGRESS_WINDOW_WIDTH_);
  success = success && sub_nh.getParam("height", options.PROGRESS_WINDOW_HEIGHT_);
  success = success && sub_nh.getParam("window_update_delay", options.SHOW_PROGRESS_IN_WINDOW_DELAY_MS_);
  success = success && sub_nh.getParam("show_window", options.SHOW_PROGRESS_IN_WINDOW_);
  success = success && sub_nh.getParam("camera_follow_robot", options.CAMERA_3DSCENE_FOLLOWS_ROBOT_);
  return success;
}
}  // namespace

bool loadOptions(const ros::NodeHandle& nh, PFslam::Options& options)
{
  bool success = true;
  success = success && loadMotionModelParameters(nh, options.motion_model_options_);
  success = success && loadVisualizationOptions(nh, options);
  success = success && nh.getParam("simplemap_save_folder", options.simplemap_path_prefix);
  return success;
}

}  // namespace mrpt_rbpf_slam
