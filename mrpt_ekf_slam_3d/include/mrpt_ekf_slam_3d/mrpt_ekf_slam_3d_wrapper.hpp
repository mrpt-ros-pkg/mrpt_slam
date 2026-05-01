// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: mrpt_ekf_slam_3d_wrapper.hpp
 * Author: Vladislav Tananaev
 *
 */

#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d.h"

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 messages
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// MRPT bridge libs (ROS2 versions)
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt_msgs/msg/observation_range_bearing.hpp>
#include <mrpt_msgs_bridge/landmark.hpp>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>

namespace mrpt_ekf_slam_3d
{
/**
 * @brief The EKFslamWrapper class provides the ROS 2 wrapper for EKF SLAM 3d
 * from MRPT libraries.
 *
 */
class EKFslamWrapper : public EKFslam, public rclcpp::Node
{
public:
        /**
         * @brief constructor
         */
  explicit EKFslamWrapper(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        /**
         * @brief destructor
         */
  ~EKFslamWrapper() = default;
        /**
         * @brief read the parameters from launch file
         */
  void get_param();
        /**
         * @brief compute the correct orientation and scale of covariance ellipsoids
         * (make sure that  we output covariance ellipsoids for right handed system
         * of coordinates)
         *
         * @param eigenvectors the 3x3 matrix of eigenvectors
         * @param eigenvalues the 3d vector of eigen values
         */
  void makeRightHanded(
    Eigen::Matrix3d & eigenvectors, Eigen::Vector3d & eigenvalues);

        /**
         * @brief compute the orientation and scale of covariance ellipsoids
         *
         * @param orientation the orientation of the ellipsoid in Quaternions
         * @param scale the vector of the eigen values for calculating the size of
         * the ellipse
         * @param covariance covariance matrix for current landmarks or robot pose
         */
  void computeEllipseOrientationScale(
    tf2::Quaternion & orientation, Eigen::Vector3d & scale,
    const mrpt::math::CMatrixDouble33 & covariance);
        /**
         * @brief initialize publishers subscribers and EKF 3d slam
         */
  bool init();
        /**
         * @brief play rawlog file
         *
         * @return true if rawlog file exists and played
         */
  bool rawlogPlay();
        /**
         * @brief check the existance of the file
         *
         * @return true if file exists
         */
  bool is_file_exists(const std::string & name);
        /**
         * @brief visualize the covariance ellipsoids for robot and landmarks
         */
  void viz_state();
        /**
         * @brief visualize the data associations for the landmarks observed by
         * robot at the each step
         */
  void viz_dataAssociation();
        /**
         * @brief  get  the odometry for received observation
         *
         * @param _odometry odometry for received observation
         * @param _msg_header timestamp of the observation
         */
  void odometryForCallback(
    mrpt::obs::CObservationOdometry::Ptr & _odometry,
    const std_msgs::msg::Header & _msg_header);
        /**
         * @brief callback function for the landmarks
         *
         * Given the landmarks wait for odometry,
         * create the pair of action and observation,
         * implement one SLAM update,
         * publish map and pose.
         *
         * @param msg  the landmark message
         */
  void landmarkCallback(
    const mrpt_msgs::msg::ObservationRangeBearing::SharedPtr msg);
        /**
         * @brief  update the pose of the sensor with respect to the robot
         *
         * @param frame_id the frame of the sensors
         */
  void updateSensorPose(const std::string & frame_id);
        /**
         * @brief wait for transform between odometry frame and the robot frame
         *
         * @param des position of the robot with respect to odometry frame
         * @param target_frame the odometry tf frame
         * @param source_frame the robot tf frame
         * @param time timestamp of the observation for which we want to retrieve
         * the position of the robot
         * @param timeout timeout for odometry waiting
         *
         * @return true if there is transform from odometry to the robot
         */
  bool waitForTransform(
    mrpt::poses::CPose3D & des, const std::string & target_frame,
    const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout);

        /**
         * @brief  publish tf tree
         *
         */
  void publishTF();

private:
  double rawlog_play_delay_{0.1};        ///< delay of replay from rawlog file
  double ellipse_scale_{1.0};        ///< Scale of covariance ellipses
  bool rawlog_play_{false};        ///< true if rawlog file exists

        // Subscribers
  std::vector<rclcpp::SubscriptionBase::SharedPtr>
  sensorSub_;                ///< list of sensors topics

  std::string rawlog_filename_;        ///< name of rawlog file
  std::string ini_filename_;        ///< name of ini file
  std::string global_frame_id_{"map"};        ///< /map frame
  std::string odom_frame_id_{"odom"};        ///< /odom frame
  std::string base_frame_id_{"base_link"};        ///< robot frame

        // Sensor source
  std::string sensor_source_;        ///< landmark sensor topics

  std::map<std::string, mrpt::poses::CPose3D>
  landmark_poses_;                ///< landmark poses with respect to the map

  mrpt::system::CTicTac tictac_;        ///< timer for SLAM performance evaluation
  float t_exec_{0.0f};        ///< the time which take one SLAM update execution

        // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    data_association_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    state_viz_pub_;

        // TF2 infrastructure
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mrpt_ekf_slam_3d
