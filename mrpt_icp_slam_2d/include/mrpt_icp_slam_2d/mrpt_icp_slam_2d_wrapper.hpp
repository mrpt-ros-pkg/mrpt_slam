/*
 * File: mrpt_icp_slam_2d_wrapper.hpp
 * Author: Vladislav Tananaev
 *
 */

#pragma once

// MRPT libraries
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/gui/CDisplayWindow3D.h>

#include <stdint.h>
#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <string>
#include <memory>

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 messages
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_msgs/msg/map_meta_data.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// MRPT bridge libs (ROS2 versions)
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CRawlog.h>

using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

namespace mrpt_icp_slam_2d
{

/**
 * @brief The ICPslamWrapper class provides 2d icp based SLAM from MRPT
 * libraries.
 *
 */
class ICPslamWrapper : public rclcpp::Node
{
   public:
	/**
	 * @brief constructor
	 */
	ICPslamWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

	/**
	 * @brief destructor
	 */
	~ICPslamWrapper();

	/**
	 * @brief read ini file
	 *
	 * @param ini_filename the name of the ini file to read
	 */
	void read_iniFile(std::string ini_filename);
	/**
	 * @brief init 3D window from mrpt lib
	 */
	void init3Dwindow();
	/**
	 * @brief run 3D window update from mrpt lib
	 */
	void run3Dwindow();

	/**
	 * @brief read the parameters from launch file
	 */
	void get_param();
	/**
	 * @brief initialize publishers subscribers and icp slam
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
	 * @brief callback function for the laser scans
	 *
	 * Given the laser scans,
	 * implement one SLAM update,
	 * publish map and pose.
	 *
	 * @param msg  the laser scan message
	 */
	void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	/**
	 * @brief  publish tf tree
	 *
	 */
	void publishTF();
	/**
	 * @brief publish point and/or grid map and robot pose
	 *
	 */
	void publishMapPose();
	/**
	 * @brief  update the pose of the sensor with respect to the robot
	 *
	 *@param frame_id the frame of the sensors
	 */
	void updateSensorPose(std::string _frame_id);
	/**
	 * @brief  the callback for update trajectory
	 *
	 */
	void updateTrajectoryTimerCallback();
	/**
	 * @brief  the callback for publish trajectory
	 *
	 */
	void publishTrajectoryTimerCallback();

   protected:
	CMetricMapBuilderICP mapBuilder;  ///< icp slam class

	double rawlog_play_delay_;  ///< delay of replay from rawlog file
	bool rawlog_play_{false};  ///< true if rawlog file exists

	std::string rawlog_filename_;  ///< name of rawlog file
	std::string ini_filename_;  ///< name of ini file
	std::string global_frame_id_;  ///< /map frame
	std::string odom_frame_id_;  ///< /odom frame
	std::string base_frame_id_;  ///< robot frame
	geometry_msgs::msg::PoseStamped pose;  ///< the robot pose

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;  ///< trajectory publisher
	nav_msgs::msg::Path path;  ///< trajectory path

	rclcpp::TimerBase::SharedPtr update_trajectory_timer_;  ///< timer for update trajectory
	rclcpp::TimerBase::SharedPtr publish_trajectory_timer_;  ///< timer for publish trajectory

	double trajectory_update_rate_;  ///< trajectory update rate(Hz)
	double trajectory_publish_rate_;  ///< trajectory publish rate(Hz)

	// Sensor source
	std::string sensor_source_;  ///< 2D laser scans
	std::map<std::string, mrpt::poses::CPose3D>
		laser_poses_;  ///< laser scan poses with respect to the map

	// Subscribers
	std::vector<rclcpp::SubscriptionBase::SharedPtr> sensorSub_;  ///< list of sensors topics

	// receive map after iteration of SLAM to metric map
	CMultiMetricMap metric_map_;

	// Publishers
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
	rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pub_metadata_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;

	// TF2 infrastructure
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	CTicTac tictac;  ///< timer for SLAM performance evaluation
	float t_exec;  ///< the time which take one SLAM update execution
	CSensoryFrame::Ptr observations;
	CObservation::Ptr observation;
	mrpt::system::TTimeStamp
		timeLastUpdate_;  ///< last update of the pose and map

	mrpt::gui::CDisplayWindow3D::Ptr win3D_;  ///< MRPT window

	std::vector<CObservation2DRangeScan::Ptr> lst_current_laser_scans;
	bool isObsBasedRawlog;
	bool SHOW_PROGRESS_3D_REAL_TIME;
	int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS;
	bool SHOW_LASER_SCANS_3D;
	bool CAMERA_3DSCENE_FOLLOWS_ROBOT;
};

}  // namespace mrpt_icp_slam_2d
