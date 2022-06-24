/*
 * File: mrpt_rbpf_slam_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#pragma once

#include <iostream>	 // std::cout
#include <fstream>	// std::ifstream
#include <string>
#include "mrpt_rbpf_slam/mrpt_rbpf_slam.h"

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// mrpt msgs
#include "mrpt_msgs/ObservationRangeBeacon.h"
// mrpt bridge libs
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/logging.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt_msgs_bridge/beacon.h>
#include <mrpt/ros1bridge/time.h>

#include <mrpt/obs/CObservationBeaconRanges.h>

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslamWrapper class provides  the ROS wrapper for
 *Rao-Blackwellized Particle filter SLAM from MRPT libraries.
 *
 */
class PFslamWrapper : public PFslam
{
   public:
	PFslamWrapper();
	~PFslamWrapper() = default;

	/**
	 * @brief Read the parameters from launch file
	 */
	bool getParams(const ros::NodeHandle& nh_p);

	/**
	 * @brief Initialize publishers subscribers and RBPF slam
	 */
	bool init(ros::NodeHandle& nh);

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
	void callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& msg);

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
	void laserCallback(const sensor_msgs::LaserScan& msg);

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
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const ros::Duration& timeout,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

	/**
	 * @brief Get the odometry for received observation
	 *
	 * @param[out] odometry odometry for received observation
	 * @param[in]  msg_header timestamp of the observation
	 */
	void odometryForCallback(
		mrpt::obs::CObservationOdometry::Ptr& odometry,
		const std_msgs::Header& msg_header);

	/**
	 * @brief Update the pose of the sensor with respect to the robot
	 *
	 *@param frame_id the frame of the sensors
	 */
	void updateSensorPose(const std::string& frame_id);

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
	double rawlog_play_delay_;	///< delay of replay from rawlog file
	bool rawlog_play_{false};  ///< true if rawlog file exists

	std::string rawlog_filename_;  ///< name of rawlog file
	std::string ini_filename_;	///< name of ini file
	std::string global_frame_id_;  ///< /map frame
	std::string odom_frame_id_;	 ///< /odom frame
	std::string base_frame_id_;	 ///< robot frame

	// Sensor source
	std::string sensor_source_;	 ///< 2D laser scans
	bool update_sensor_pose_;  ///< on true the sensor pose is updated on every
							   ///< sensor reading

	std::map<std::string, mrpt::poses::CPose3D>
		laser_poses_;  ///< laser scan poses with respect to the map
	std::map<std::string, mrpt::poses::CPose3D>
		beacon_poses_;	///< beacon poses with respect to the map

	// Subscribers
	std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics

	// read rawlog file
	// vector of pairs of actions and obsrvations from rawlog file
	std::vector<
		std::pair<mrpt::obs::CActionCollection, mrpt::obs::CSensoryFrame>>
		data_;

	std::vector<mrpt::opengl::CEllipsoid3D::Ptr> viz_beacons_;

	// publishers for map and pose particles
	ros::Publisher pub_map_, pub_metadata_, pub_particles_,
		pub_particles_beacons_, beacon_viz_pub_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listenerTF_{tf_buffer_};
	tf2_ros::TransformBroadcaster tf_broadcaster_;	///< transform broadcaster

	mrpt::system::CTicTac tictac_;	///< timer for SLAM performance evaluation
	float t_exec_;	///< the time which take one SLAM update execution
};
}  // namespace mrpt_rbpf_slam
