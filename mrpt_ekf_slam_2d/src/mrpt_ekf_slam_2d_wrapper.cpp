// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: mrpt_ekf_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_2d/mrpt_ekf_slam_2d_wrapper.hpp"
#include "mrpt_ekf_slam_2d/ekf_slam_math.hpp"

#include <mrpt/serialization/CArchive.h>

namespace mrpt_ekf_slam_2d
{

EKFslamWrapper::EKFslamWrapper(const rclcpp::NodeOptions & options)
: Node("mrpt_ekf_slam_2d", options)
{
  rawlog_play_ = false;
  timeLastUpdate_ = mrpt::Clock::now();
}

bool EKFslamWrapper::is_file_exists(const std::string & name)
{
  return mrpt_ekf_slam_2d::is_file_exists(name);
}

void EKFslamWrapper::get_param()
{
  RCLCPP_INFO(this->get_logger(), "READ PARAM FROM LAUNCH FILE");

  this->declare_parameter<double>("ellipse_scale", 1.0);
  ellipse_scale_ = this->get_parameter("ellipse_scale").as_double();
  RCLCPP_INFO(this->get_logger(), "ellipse_scale: %f", ellipse_scale_);

  this->declare_parameter<double>("rawlog_play_delay", 0.1);
  rawlog_play_delay_ = this->get_parameter("rawlog_play_delay").as_double();
  RCLCPP_INFO(
                this->get_logger(), "rawlog_play_delay: %f", rawlog_play_delay_);

  this->declare_parameter<std::string>("rawlog_filename", "");
  rawlog_filename_ = this->get_parameter("rawlog_filename").as_string();
  RCLCPP_INFO(
                this->get_logger(), "rawlog_filename: %s", rawlog_filename_.c_str());

  this->declare_parameter<std::string>("ini_filename", "");
  ini_filename_ = this->get_parameter("ini_filename").as_string();
  RCLCPP_INFO(
                this->get_logger(), "ini_filename: %s", ini_filename_.c_str());

  this->declare_parameter<std::string>("global_frame_id", "map");
  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  RCLCPP_INFO(
                this->get_logger(), "global_frame_id: %s", global_frame_id_.c_str());

  this->declare_parameter<std::string>("odom_frame_id", "odom");
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  RCLCPP_INFO(
                this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());

  this->declare_parameter<std::string>("base_frame_id", "base_link");
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  RCLCPP_INFO(
                this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());

  this->declare_parameter<std::string>("sensor_source", "scan");
  sensor_source_ = this->get_parameter("sensor_source").as_string();
  RCLCPP_INFO(
                this->get_logger(), "sensor_source: %s", sensor_source_.c_str());
}

bool EKFslamWrapper::init()
{
        // get parameters from ini file
  if (!is_file_exists(ini_filename_)) {
    RCLCPP_ERROR_STREAM(
                        this->get_logger(), "CAN'T READ INI FILE: " << ini_filename_);
    return false;
  }

  EKFslam::read_iniFile(ini_filename_);

        // read rawlog file if it exists
  if (is_file_exists(rawlog_filename_)) {
    RCLCPP_WARN_STREAM(
                        this->get_logger(),
                        "PLAY FROM RAWLOG FILE: " << rawlog_filename_);
    rawlog_play_ = true;
  }

        /// Create TF2 components ///
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        /// Create publishers ///
  state_viz_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/state_viz", 1);
  data_association_viz_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/data_association_viz", 1);

        // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
  if (lstSources.empty()) {
    RCLCPP_FATAL(
                        this->get_logger(),
                        "*Fatal*: At least one sensor source must be provided in "
                        "~sensor_sources (e.g. \"landmark\")");
    return false;
  }

        /// Create subscribers ///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++) {
    if (lstSources[i].find("landmark") != std::string::npos) {
      sensorSub_[i] = this->create_subscription<
        mrpt_msgs::msg::ObservationRangeBearing>(
                                lstSources[i], 1,
                                std::bind(
                                        &EKFslamWrapper::landmarkCallback, this,
                                        std::placeholders::_1));
    } else {
      RCLCPP_ERROR(
                                this->get_logger(),
                                "Can't find the sensor topics. The sensor topics should "
                                "contain the word \"landmark\" in the name");
    }
  }

  init3Dwindow();
  return true;
}

void EKFslamWrapper::odometryForCallback(
  mrpt::obs::CObservationOdometry::Ptr & _odometry,
  const std_msgs::msg::Header & _msg_header)
{
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(
                        poseOdom, odom_frame_id_, base_frame_id_, _msg_header.stamp,
                        rclcpp::Duration::from_seconds(1.0)))
  {
    _odometry = mrpt::obs::CObservationOdometry::Create();
    _odometry->sensorLabel = odom_frame_id_;
    _odometry->hasEncodersInfo = false;
    _odometry->hasVelocities = false;
    _odometry->odometry.x() = poseOdom.x();
    _odometry->odometry.y() = poseOdom.y();
    _odometry->odometry.phi() = poseOdom.yaw();
  }
}

void EKFslamWrapper::updateSensorPose(const std::string & frame_id)
{
  geometry_msgs::msg::TransformStamped transformStmp;
  try {
    auto tf_timeout = std::chrono::seconds(1);
    transformStmp = tf_buffer_->lookupTransform(
                        base_frame_id_, frame_id, tf2::TimePointZero, tf_timeout);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
                        this->get_logger(),
                        "Failed to get transform target_frame (%s) to source_frame (%s): "
                        "%s",
                        base_frame_id_.c_str(), frame_id.c_str(), e.what());
    return;
  }
  tf2::Transform transform;
  tf2::fromMsg(transformStmp.transform, transform);
  const mrpt::poses::CPose3D pose = mrpt::ros2bridge::fromROS(transform);

  landmark_poses_[frame_id] = pose;
}

bool EKFslamWrapper::waitForTransform(
  mrpt::poses::CPose3D & des, const std::string & target_frame,
  const std::string & source_frame, const rclcpp::Time & /*time*/,
  const rclcpp::Duration & timeout)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    auto tf_timeout = std::chrono::nanoseconds(timeout.nanoseconds());
    transform = tf_buffer_->lookupTransform(
                        target_frame, source_frame, tf2::TimePointZero, tf_timeout);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
                        this->get_logger(),
                        "Failed to get transform target_frame (%s) to source_frame (%s): "
                        "%s",
                        target_frame.c_str(), source_frame.c_str(), e.what());
    return false;
  }
  tf2::Transform tx;
  tf2::fromMsg(transform.transform, tx);
  des = mrpt::ros2bridge::fromROS(tx);
  return true;
}

void EKFslamWrapper::landmarkCallback(
  const mrpt_msgs::msg::ObservationRangeBearing::SharedPtr msg)
{
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  CObservationBearingRange::Ptr landmark = CObservationBearingRange::Create();

  if (landmark_poses_.find(msg->header.frame_id) == landmark_poses_.end()) {
    updateSensorPose(msg->header.frame_id);
  } else {
    mrpt::poses::CPose3D pose = landmark_poses_[msg->header.frame_id];
    mrpt_msgs_bridge::fromROS(
                        *msg, landmark_poses_[msg->header.frame_id], *landmark);

    sf = mrpt::obs::CSensoryFrame::Create();
    CObservationOdometry::Ptr odometry;
    odometryForCallback(odometry, msg->header);

    CObservation::Ptr obs = CObservation::Ptr(landmark);
    sf->insert(obs);
    observation(sf, odometry);
    timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

    tictac_.Tic();
    mapping.processActionObservation(action, sf);
    t_exec_ = tictac_.Tac();
    RCLCPP_INFO(
                        this->get_logger(), "Map building executed in %.03fms",
                        1000.0f * t_exec_);
    rclcpp::sleep_for(std::chrono::nanoseconds(
                        static_cast<int64_t>(rawlog_play_delay_ * 1e9)));
    mapping.getCurrentState(
                        robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);
    viz_state();
    viz_dataAssociation();
    run3Dwindow();
    publishTF();
  }
}

bool EKFslamWrapper::rawlogPlay()
{
  if (rawlog_play_ == false) {
    return false;
  } else {
    size_t rawlogEntry = 0;
    mrpt::io::CFileGZInputStream rawlog_stream(rawlog_filename_);
    auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);

    mrpt::obs::CActionCollection::Ptr action;
    mrpt::obs::CSensoryFrame::Ptr observations;

    while (rclcpp::ok()) {
      if (!mrpt::obs::CRawlog::readActionObservationPair(
                                                rawlogFile, action, observations, rawlogEntry))
      {
        break;                                // file EOF
      }
      tictac_.Tic();
      mapping.processActionObservation(action, observations);
      t_exec_ = tictac_.Tac();
      RCLCPP_INFO(
                                        this->get_logger(),
                                        "Map building executed in %.03fms", 1000.0f * t_exec_);
      rclcpp::sleep_for(std::chrono::nanoseconds(
                                        static_cast<int64_t>(rawlog_play_delay_ * 1e9)));
      mapping.getCurrentState(
                                        robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);

      viz_state();
      viz_dataAssociation();
      run3Dwindow();
    }
    if (win3d) {
      std::cout << "\n Close the 3D window to quit the application.\n";
      win3d->waitForKey();
    }
    return true;
  }
}

// Delegates to the free function in ekf_slam_math.hpp for testability.
void EKFslamWrapper::makeRightHanded(
  Eigen::Matrix2d & eigenvectors, Eigen::Vector2d & eigenvalues)
{
  mrpt_ekf_slam_2d::makeRightHanded(eigenvectors, eigenvalues);
}

void EKFslamWrapper::computeEllipseOrientationScale2D(
  tf2::Quaternion & orientation, Eigen::Vector2d & scale,
  const mrpt::math::CMatrixDouble22 & covariance)
{
  tf2::Matrix3x3 tf3d;
  Eigen::Vector2d eigenvalues(Eigen::Vector2d::Identity());
  Eigen::Matrix2d eigenvectors(Eigen::Matrix2d::Zero());

        // NOTE: The SelfAdjointEigenSolver only references the lower triangular
        // part of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(
    covariance.asEigen());

        // Compute eigenvectors and eigenvalues
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  } else {
    RCLCPP_ERROR_STREAM(
                        this->get_logger(),
                        "failed to compute eigen vectors/values for position. Is "
                        "the covariance matrix correct?");
    eigenvalues = Eigen::Vector2d::Zero();              // Setting the scale to zero
                                                                                                // will hide it on the screen
    eigenvectors = Eigen::Matrix2d::Identity();
  }

        // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

        // Rotation matrix around  z axis
  tf3d.setValue(
                eigenvectors(0, 0), eigenvectors(0, 1), 0, eigenvectors(1, 0),
                eigenvectors(1, 1), 0, 0, 0, 1);

        // get orientation from rotation matrix
  tf3d.getRotation(orientation);
        // get scale
  scale[0] = eigenvalues[0];
  scale[1] = eigenvalues[1];
}

void EKFslamWrapper::viz_dataAssociation()
{
        // robot pose
  mrpt::poses::CPose3D robotPose;
  robotPose = mrpt::poses::CPose3D(robotPose_.mean);
  geometry_msgs::msg::Point pointRobotPose;
  pointRobotPose.z = 0;
  pointRobotPose.x = robotPose.x();
  pointRobotPose.y = robotPose.y();

        // visualization of the data association
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker line_strip;

  line_strip.header.frame_id = "map";
  line_strip.header.stamp = this->now();

  line_strip.id = 0;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;

  line_strip.lifetime = rclcpp::Duration::from_seconds(0.1);
  line_strip.pose.position.x = 0;
  line_strip.pose.position.y = 0;
  line_strip.pose.position.z = 0;
  line_strip.pose.orientation.x = 0.0;
  line_strip.pose.orientation.y = 0.0;
  line_strip.pose.orientation.z = 0.0;
  line_strip.pose.orientation.w = 1.0;
  line_strip.scale.x = 0.02;             // line uses only x component
  line_strip.scale.y = 0.0;
  line_strip.scale.z = 0.0;
  line_strip.color.a = 1.0;
  line_strip.color.r = 1.0;
  line_strip.color.g = 1.0;
  line_strip.color.b = 1.0;

        // Draw latest data association:
  const CRangeBearingKFSLAM2D::TDataAssocInfo & da =
    mapping.getLastDataAssociation();

  for (auto it = da.results.associations.begin();
    it != da.results.associations.end(); ++it)
  {
    const mrpt::slam::prediction_index_t idxPred = it->second;

                // This index must match the internal list of features in the map:
    CRangeBearingKFSLAM2D::KFArray_FEAT featMean;
    mapping.getLandmarkMean(idxPred, featMean);

    line_strip.points.clear();
    line_strip.points.push_back(pointRobotPose);
    geometry_msgs::msg::Point pointLm;
    pointLm.z = 0.0;
    pointLm.x = featMean[0];
    pointLm.y = featMean[1];
    line_strip.points.push_back(pointLm);
    ma.markers.push_back(line_strip);
    line_strip.id++;
  }

  data_association_viz_pub_->publish(ma);
}

void EKFslamWrapper::viz_state()
{
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

        // get the covariance matrix 2x2 for each ellipsoid including robot pose
  mrpt::opengl::CSetOfObjects::Ptr objs;
  objs = mrpt::opengl::CSetOfObjects::Create();
  mapping.getAs3DObject(objs);

        // Count the number of landmarks
  unsigned int objs_counter = 0;
  while (objs->getByClass<mrpt::opengl::CEllipsoid2D>(objs_counter)) {
    objs_counter++;
  }

  mrpt::opengl::CEllipsoid2D::Ptr landmark;
  for (size_t i = 0; i < objs_counter; i++) {
    landmark = objs->getByClass<mrpt::opengl::CEllipsoid2D>(i);

    const auto covariance = landmark->getCovMatrix();
    float quantiles = landmark->getQuantiles();

                // mean position
                // pose of the robot and landmarks (x,y,z=0)
    const auto pose = mrpt::poses::CPose3D(landmark->getPose());

                // covariance ellipses
    tf2::Quaternion orientation;
    Eigen::Vector2d scale;

    computeEllipseOrientationScale2D(orientation, scale, covariance);

    marker.id++;
    marker.color.a = 1.0;
    if (i == 0) {  // robot position
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position.x = pose.x();
    marker.pose.position.y = pose.y();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = ellipse_scale_ * quantiles * sqrt(scale[0]);
    marker.scale.y = ellipse_scale_ * quantiles * sqrt(scale[1]);
    marker.scale.z = 0.00001;              // Z can't be 0, limitation of ROS
    ma.markers.push_back(marker);

    marker.id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    if (i == 0) {
      marker.text = "robot";
    } else {
      marker.text = std::to_string(LM_IDs_[i]);
    }
    marker.pose.position.x = pose.x();
    marker.pose.position.y = pose.y();
    marker.pose.position.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    ma.markers.push_back(marker);
  }

  state_viz_pub_->publish(ma);
}

void EKFslamWrapper::publishTF()
{
  mapping.getCurrentState(robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);

        // Most of this code was copy and pase from ros::amcl
  const mrpt::poses::CPose3D robotPoseTF =
    mrpt::poses::CPose3D(robotPose_.mean);

  const rclcpp::Time stamp = mrpt::ros2bridge::toROS(timeLastUpdate_);

  geometry_msgs::msg::PoseStamped odom_to_map;

  try {
    tf2::Transform tmp_tf =
      mrpt::ros2bridge::toROS_tfTransform(robotPoseTF);

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (const tf2::TransformException &) {
    RCLCPP_INFO(
                        this->get_logger(),
                        "Failed to subtract global_frame (%s) from odom_frame (%s)",
                        global_frame_id_.c_str(), odom_frame_id_.c_str());
    return;
  }

  {
    tf2::Transform latest_tf;
    tf2::convert(odom_to_map.pose, latest_tf);

                // We want to send a transform that is good up until a
                // tolerance time so that odom can be used
    rclcpp::Duration transform_tolerance =
      rclcpp::Duration::from_seconds(0.1);

    rclcpp::Time transform_expiration = stamp + transform_tolerance;

    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = global_frame_id_;
    tmp_tf_stamped.header.stamp = transform_expiration;
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);

    tf_broadcaster_->sendTransform(tmp_tf_stamped);
  }
}

}  // namespace mrpt_ekf_slam_2d
