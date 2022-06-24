/*
 * File: mrpt_ekf_slam_2d_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#pragma once
#include <iostream>	 // std::cout
#include <fstream>	// std::ifstream
#include <string>
#include "mrpt_ekf_slam_2d/mrpt_ekf_slam_2d.h"

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// add ros msgs
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// mrpt bridge libs
#include <mrpt/ros1bridge/pose.h>
#include <mrpt_msgs_bridge/landmark.h>
#include <mrpt/ros1bridge/logging.h>
#include <mrpt/ros1bridge/time.h>
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
#include <mrpt/opengl/CEllipsoid2D.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt_msgs/ObservationRangeBearing.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>

/**
 * @brief The EKFslamWrapper class provides the ROS wrapper for EKF SLAM 2d from
 * MRPT libraries.
 *
 */
class EKFslamWrapper : EKFslam
{
   public:
	/**
	 * @brief constructor
	 */
	EKFslamWrapper();
	/**
	 * @brief destructor
	 */
	~EKFslamWrapper();
	/**
	 * @brief read the parameters from launch file
	 */
	void get_param();
	/**
	 * @brief compute the orientation and scale of covariance ellipsoids
	 *
	 * @param orientation the orientation of the ellipsoid in Quaternions
	 * @param scale the vector of the eigen values for calculating the size of
	 * the ellipse
	 * @param covariance covariance matrix for current landmarks or robot pose
	 */
	void computeEllipseOrientationScale2D(
		tf2::Quaternion& orientation, Eigen::Vector2d& scale,
		const mrpt::math::CMatrixDouble22& covariance);
	/**
	 * @brief compute the correct orientation and scale of covariance ellipsoids
	 * (make sure that  we output covariance ellipsoids for right handed system
	 * of coordinates)
	 *
	 * @param eigenvectors the 2x2 matrix of eigenvectors
	 * @param eigenvalues the 2d vector of eigen values
	 */
	void makeRightHanded(
		Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues);
	/**
	 * @brief initialize publishers subscribers and EKF 2d slam
	 */
	void init();
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
	bool is_file_exists(const std::string& name);
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
		mrpt::obs::CObservationOdometry::Ptr& _odometry,
		const std_msgs::Header& _msg_header);
	/**
	 * @brief callback function for the landmarks
	 *
	 * Given the landmarks wait for odometry,
	 * create the pair of action and observation,
	 * implement one SLAM update,
	 * publish map and pose.
	 *
	 * @param _msg  the landmark message
	 */
	void landmarkCallback(const mrpt_msgs::ObservationRangeBearing& _msg);
	/**
	 * @brief  update the pose of the sensor with respect to the robot
	 *
	 * @param frame_id the frame of the sensors
	 */
	void updateSensorPose(std::string _frame_id);
	/**
	 * @brief wait for transform between odometry frame and the robot frame
	 *
	 * @param des position of the robot with respect to odometry frame
	 * @param target_frame the odometry tf frame
	 * @param source_frame the robot tf frame
	 * @param time timestamp of the observation for which we want to retrieve
	 * the position of the robot
	 * @param timeout timeout for odometry waiting
	 * @param polling_sleep_duration timeout for transform wait
	 *
	 * @return true if there is transform from odometry to the robot
	 */
	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const ros::Duration& timeout,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

	/**
	 * @brief  publish tf tree
	 *
	 */
	void publishTF();

   private:
	ros::NodeHandle n_;	 ///< Node handler
	double rawlog_play_delay;  ///< delay of replay from rawlog file
	double ellipse_scale_;	///< Scale of covariance ellipses
	bool rawlog_play_;	///< true if rawlog file exists
	// Subscribers
	std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics
	std::string rawlog_filename;  ///< name of rawlog file
	std::string ini_filename;  ///< name of ini file
	std::string global_frame_id;  ///< /map frame
	std::string odom_frame_id;	///< /odom frame
	std::string base_frame_id;	///< robot frame

	// Sensor source
	std::string sensor_source;	///< 2D laser scans

	std::map<std::string, mrpt::poses::CPose3D>
		landmark_poses_;  ///< landmark poses with respect to the map

	mrpt::system::CTicTac tictac;  ///< timer for SLAM performance evaluation
	float t_exec;  ///< the time which take one SLAM update execution

	ros::Publisher data_association_viz_pub_, state_viz_pub_;  ///< publishers

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listenerTF_{tf_buffer_};
	tf2_ros::TransformBroadcaster tf_broadcaster_;	///< transform broadcaster
};
