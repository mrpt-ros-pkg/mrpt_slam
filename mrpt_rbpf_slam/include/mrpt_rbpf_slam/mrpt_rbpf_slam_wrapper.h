// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: mrpt_rbpf_slam_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#pragma once

#include <iostream>  // std::cout
#include <fstream>  // std::ifstream
#include <string>
#include <memory>
#include "mrpt_rbpf_slam/mrpt_rbpf_slam.h"

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 messages
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_msgs/msg/map_meta_data.hpp"
#include <nav_msgs/srv/get_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// MRPT msgs
#include "mrpt_msgs/msg/observation_range_beacon.hpp"

// MRPT bridge libs (ROS2 versions)
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt_msgs_bridge/beacon.hpp>
#include <mrpt/ros2bridge/time.h>

#include <mrpt/obs/CObservationBeaconRanges.h>

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslamWrapper class provides  the ROS wrapper for
 *Rao-Blackwellized Particle filter SLAM from MRPT libraries.
 *
 */
class PFslamWrapper : public PFslam, public rclcpp::Node
{
public:
  PFslamWrapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PFslamWrapper() = default;

        /**
         * @brief Read the parameters from ROS2 node
         */
  bool getParams();

        /**
         * @brief Initialize publishers subscribers and RBPF slam
         */
  bool init();

        /**
         * @brief Play rawlog file
         *
         * @return true if rawlog file exists and played
         */
  bool rawlogPlay();

        /**
         * @brief Publish beacon or grid map and robot pose
         */
  void publishMapPose();

        /**
         * @brief Callback function for the beacons
         *
         * Given the range only observation wait for odometry,
         * create the pair of action and observation,
         * implement one SLAM update,
         * publish map and pose.
         *
         * @param msg  the beacon message
         */
  void callbackBeacon(const mrpt_msgs::msg::ObservationRangeBeacon::SharedPtr msg);

        /**
         * @brief Callback function for the laser scans
         *
         * Given the laser scans  wait for odometry,
         * create the pair of action and observation,
         * implement one SLAM update,
         * publish map and pose.
         *
         * @param msg  the laser scan message
         */
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        /**
         * @brief Wait for transform between odometry frame and the robot frame
         *
         * @param[out] des position of the robot with respect to odometry frame
         * @param[in]  target_frame the odometry tf frame
         * @param[in]  source_frame the robot tf frame
         * @param[in]  time timestamp of the observation for which we want to
         * retrieve the position of the robot
         * @param[in]  timeout timeout for odometry waiting
         * @param[in]  polling_sleep_duration timeout for transform wait
         *
         * @return true if there is transform from odometry to the robot
         */
  bool waitForTransform(
    mrpt::poses::CPose3D & des, const std::string & target_frame,
    const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration & timeout,
    const rclcpp::Duration & polling_sleep_duration = rclcpp::Duration::from_seconds(0.01));

        /**
         * @brief Get the odometry for received observation
         *
         * @param[out] odometry odometry for received observation
         * @param[in]  msg_header timestamp of the observation
         */
  void odometryForCallback(
    mrpt::obs::CObservationOdometry::Ptr & odometry,
    const std_msgs::msg::Header & msg_header);

        /**
         * @brief Update the pose of the sensor with respect to the robot
         *
         *@param frame_id the frame of the sensors
         */
  void updateSensorPose(const std::string & frame_id);

        /**
         * @brief Publish tf tree
         *
         */
  void publishTF();

        /**
         * @brief Correct visualization for ro slam
         *
         */
  void vizBeacons();

private:
  rclcpp::TimerBase::SharedPtr update_timer_;        ///< Timer for periodic updates in rawlog mode

  double rawlog_play_delay_;        ///< delay of replay from rawlog file
  bool rawlog_play_{false};        ///< true if rawlog file exists

  std::string rawlog_filename_;        ///< name of rawlog file
  std::string ini_filename_;        ///< name of ini file
  std::string global_frame_id_;        ///< /map frame
  std::string odom_frame_id_;        ///< /odom frame
  std::string base_frame_id_;        ///< robot frame

        // Sensor source
  std::string sensor_source_;        ///< 2D laser scans
  bool update_sensor_pose_;        ///< on true the sensor pose is updated on every
                                   ///< sensor reading

  std::map<std::string, mrpt::poses::CPose3D>
  laser_poses_;                ///< laser scan poses with respect to the map
  std::map<std::string, mrpt::poses::CPose3D>
  beacon_poses_;                ///< beacon poses with respect to the map

        // Subscribers
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sensorSub_;        ///< list of sensors topics

        // read rawlog file
        // vector of pairs of actions and observations from rawlog file
  std::vector<
    std::pair<mrpt::obs::CActionCollection, mrpt::obs::CSensoryFrame>>
  data_;

  std::vector<mrpt::opengl::CEllipsoid3D::Ptr> viz_beacons_;

        // Publishers for map and pose particles
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pub_metadata_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_beacons_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr beacon_viz_pub_;

        // TF2 infrastructure
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  mrpt::system::CTicTac tictac_;        ///< timer for SLAM performance evaluation
  float t_exec_;        ///< the time which take one SLAM update execution
};
}  // namespace mrpt_rbpf_slam
