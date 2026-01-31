/*
 * File: mrpt_icp_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.hpp"
#include <mrpt/serialization/CArchive.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfObjects.h>

using mrpt::maps::COccupancyGridMap2D;
using mrpt::maps::CSimplePointsMap;

// From:
// https://answers.ros.org/question/364561/tfcreatequaternionfromyaw-equivalent-in-ros2/
static inline auto createQuaternionMsgFromYaw(double yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	return tf2::toMsg(q);
}

namespace mrpt_icp_slam_2d
{

ICPslamWrapper::ICPslamWrapper(const rclcpp::NodeOptions& options)
	: Node("mrpt_icp_slam_2d", options)
{
	rawlog_play_ = false;
	// Default parameters for 3D window
	SHOW_PROGRESS_3D_REAL_TIME = false;
	SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;  // this parameter is not used
	SHOW_LASER_SCANS_3D = true;
	CAMERA_3DSCENE_FOLLOWS_ROBOT = true;
	isObsBasedRawlog = true;
	timeLastUpdate_ = mrpt::Clock::now();
}

ICPslamWrapper::~ICPslamWrapper()
{
	try
	{
		std::string sOutMap = "mrpt_icpslam_";
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::Clock::now(), parts, true);
		sOutMap += format(
			"%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
			(unsigned int)parts.month, (unsigned int)parts.day,
			(unsigned int)parts.hour, (unsigned int)parts.minute,
			(unsigned int)parts.second);
		sOutMap += ".simplemap";

		sOutMap = mrpt::system::fileNameStripInvalidChars(sOutMap);
		RCLCPP_INFO(this->get_logger(), "Saving built map to `%s`", sOutMap.c_str());
		mapBuilder.saveCurrentMapToFile(sOutMap);
	}
	catch (std::exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
	}
}

bool ICPslamWrapper::is_file_exists(const std::string& name)
{
	std::ifstream f(name.c_str());
	return f.good();
}

void ICPslamWrapper::read_iniFile(std::string ini_filename)
{
	mrpt::config::CConfigFile iniFile(ini_filename);

	mapBuilder.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile(iniFile, "ICP");
	mapBuilder.initialize();

	// Set up MRPT logging to use ROS2 logger
	mapBuilder.setVerbosityLevel(mrpt::system::LVL_INFO);
	mapBuilder.logging_enable_console_output = false;

	mapBuilder.logRegisterCallback(
		[this](std::string_view msg, const mrpt::system::VerbosityLevel level,
		       [[maybe_unused]] std::string_view loggerName,
		       [[maybe_unused]] const mrpt::Clock::time_point timestamp)
		{
			// Convert MRPT log to ROS2 log
			switch(level)
			{
				case mrpt::system::LVL_DEBUG:
					RCLCPP_DEBUG(this->get_logger(), "%s", std::string(msg).c_str());
					break;
				case mrpt::system::LVL_INFO:
					RCLCPP_INFO(this->get_logger(), "%s", std::string(msg).c_str());
					break;
				case mrpt::system::LVL_WARN:
					RCLCPP_WARN(this->get_logger(), "%s", std::string(msg).c_str());
					break;
				case mrpt::system::LVL_ERROR:
					RCLCPP_ERROR(this->get_logger(), "%s", std::string(msg).c_str());
					break;
				default:
					RCLCPP_INFO(this->get_logger(), "%s", std::string(msg).c_str());
			}
		});

	mapBuilder.options.alwaysInsertByClass.fromString(
		iniFile.read_string("MappingApplication", "alwaysInsertByClass", ""));

	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();

	// parameters for mrpt3D window
	CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool(
		"MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true,
		/*Force existence:*/ true);
	MRPT_LOAD_CONFIG_VAR(
		SHOW_PROGRESS_3D_REAL_TIME, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(
		SHOW_LASER_SCANS_3D, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(
		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile,
		"MappingApplication");
}

void ICPslamWrapper::get_param()
{
	RCLCPP_INFO(this->get_logger(), "READ PARAM FROM LAUNCH FILE");

	// Declare and get parameters
	this->declare_parameter<double>("rawlog_play_delay", 0.1);
	rawlog_play_delay_ = this->get_parameter("rawlog_play_delay").as_double();
	RCLCPP_INFO(this->get_logger(), "rawlog_play_delay: %f", rawlog_play_delay_);

	this->declare_parameter<std::string>("rawlog_filename", "");
	rawlog_filename_ = this->get_parameter("rawlog_filename").as_string();
	RCLCPP_INFO(this->get_logger(), "rawlog_filename: %s", rawlog_filename_.c_str());

	this->declare_parameter<std::string>("ini_filename", "");
	ini_filename_ = this->get_parameter("ini_filename").as_string();
	RCLCPP_INFO(this->get_logger(), "ini_filename: %s", ini_filename_.c_str());

	this->declare_parameter<std::string>("global_frame_id", "map");
	global_frame_id_ = this->get_parameter("global_frame_id").as_string();
	RCLCPP_INFO(this->get_logger(), "global_frame_id: %s", global_frame_id_.c_str());

	this->declare_parameter<std::string>("odom_frame_id", "odom");
	odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
	RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());

	this->declare_parameter<std::string>("base_frame_id", "base_link");
	base_frame_id_ = this->get_parameter("base_frame_id").as_string();
	RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());

	this->declare_parameter<std::string>("sensor_source", "scan");
	sensor_source_ = this->get_parameter("sensor_source").as_string();
	RCLCPP_INFO(this->get_logger(), "sensor_source: %s", sensor_source_.c_str());

	this->declare_parameter<double>("trajectory_update_rate", 10.0);
	trajectory_update_rate_ = this->get_parameter("trajectory_update_rate").as_double();
	RCLCPP_INFO(this->get_logger(), "trajectory_update_rate: %f", trajectory_update_rate_);

	this->declare_parameter<double>("trajectory_publish_rate", 5.0);
	trajectory_publish_rate_ = this->get_parameter("trajectory_publish_rate").as_double();
	RCLCPP_INFO(this->get_logger(), "trajectory_publish_rate: %f", trajectory_publish_rate_);
}

void ICPslamWrapper::init()
{
	// get parameters from ini file
	if (!is_file_exists(ini_filename_))
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "CAN'T READ INI FILE: " << ini_filename_);
		return;
	}
	read_iniFile(ini_filename_);

	// read rawlog file if it exists
	if (is_file_exists(rawlog_filename_))
	{
		RCLCPP_WARN_STREAM(this->get_logger(), "PLAY FROM RAWLOG FILE: " << rawlog_filename_.c_str());
		rawlog_play_ = true;
	}

	/// Create TF2 components ///
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	/// Create publishers ///
	pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
		"map", rclcpp::QoS(1).transient_local());
	pub_metadata_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
		"map_metadata", rclcpp::QoS(1).transient_local());
	pub_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		"PointCloudMap", rclcpp::QoS(1).transient_local());
	trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
		"trajectory", rclcpp::QoS(1).transient_local());
	pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"robot_pose", 1);

	// Create timers with std::chrono
	update_trajectory_timer_ = this->create_wall_timer(
		std::chrono::duration<double>(1.0 / trajectory_update_rate_),
		std::bind(&ICPslamWrapper::updateTrajectoryTimerCallback, this));

	publish_trajectory_timer_ = this->create_wall_timer(
		std::chrono::duration<double>(1.0 / trajectory_publish_rate_),
		std::bind(&ICPslamWrapper::publishTrajectoryTimerCallback, this));

	// read sensor topics
	std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
	if (lstSources.empty())
	{
		RCLCPP_FATAL(
			this->get_logger(),
			"*Fatal*: At least one sensor source must be provided in "
			"~sensor_sources (e.g. \"scan\" or \"beacon\")");
		return;
	}

	/// Create subscribers ///
	sensorSub_.resize(lstSources.size());
	for (size_t i = 0; i < lstSources.size(); i++)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to: " << lstSources[i]);
		sensorSub_[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
			lstSources[i], 1,
			std::bind(&ICPslamWrapper::laserCallback, this, std::placeholders::_1));
	}

	init3Dwindow();
}

void ICPslamWrapper::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	try
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "2D LIDAR rx: " << msg->header.frame_id);

		CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();
		if (laser_poses_.find(msg->header.frame_id) == laser_poses_.end())
		{
			updateSensorPose(msg->header.frame_id);
		}

		{
			mrpt::poses::CPose3D pose = laser_poses_[msg->header.frame_id];
			mrpt::ros2bridge::fromROS(
				*msg, laser_poses_[msg->header.frame_id], *laser);
			observation = CObservation::Ptr(laser);
			timeLastUpdate_ = laser->timestamp;
			tictac.Tic();
			mapBuilder.processObservation(observation);
			t_exec = tictac.Tac();
			RCLCPP_INFO(this->get_logger(), "Map building executed in %.03fms", 1000.0f * t_exec);

			run3Dwindow();
			publishTF();
			publishMapPose();
		}
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
	}
}

void ICPslamWrapper::publishMapPose()
{
	// get currently built map
	metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

	// publish map
	COccupancyGridMap2D* grid = nullptr;
	CSimplePointsMap* pm = nullptr;
	if (metric_map_.countMapsByClass<COccupancyGridMap2D>())
		grid = metric_map_.mapByClass<COccupancyGridMap2D>().get();
	if (metric_map_.countMapsByClass<CSimplePointsMap>())
		pm = metric_map_.mapByClass<CSimplePointsMap>().get();

	if (grid)
	{
		nav_msgs::msg::OccupancyGrid _msg;
		mrpt::ros2bridge::toROS(*grid, _msg);
		pub_map_->publish(_msg);
		pub_metadata_->publish(_msg.info);
	}
	if (pm)
	{
		sensor_msgs::msg::PointCloud2 _msg;
		std_msgs::msg::Header header;
		header.stamp = this->now();
		header.frame_id = global_frame_id_;
		mrpt::ros2bridge::toROS(*pm, header, _msg);
		pub_point_cloud_->publish(_msg);
	}

	CPose3D robotPose;
	mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

	// publish pose
	pose.header.frame_id = global_frame_id_;

	// the pose
	pose.pose.position.x = robotPose.x();
	pose.pose.position.y = robotPose.y();
	pose.pose.position.z = 0.0;
	pose.pose.orientation = createQuaternionMsgFromYaw(robotPose.yaw());

	pub_pose_->publish(pose);
}

void ICPslamWrapper::updateSensorPose(std::string _frame_id)
{
	geometry_msgs::msg::TransformStamped transformStmp;
	try
	{
		auto tf_timeout = std::chrono::seconds(1);
		transformStmp = tf_buffer_->lookupTransform(
			base_frame_id_, _frame_id, tf2::TimePointZero, tf_timeout);
	}
	catch (const tf2::TransformException& e)
	{
		RCLCPP_WARN(
			this->get_logger(),
			"Failed to get transform target_frame (%s) to source_frame (%s): %s",
			base_frame_id_.c_str(), _frame_id.c_str(), e.what());
		return;
	}
	tf2::Transform transform;
	tf2::fromMsg(transformStmp.transform, transform);
	const mrpt::poses::CPose3D pose = mrpt::ros2bridge::fromROS(transform);

	laser_poses_[_frame_id] = pose;
}

bool ICPslamWrapper::rawlogPlay()
{
	if (rawlog_play_ == false)
	{
		return false;
	}
	else
	{
		size_t rawlogEntry = 0;
		mrpt::io::CFileGZInputStream rawlog_stream(rawlog_filename_);
		auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);

		CActionCollection::Ptr action;

		for (;;)
		{
			if (rclcpp::ok())
			{
				if (!CRawlog::getActionObservationPairOrObservation(
						rawlogFile, action, observations, observation,
						rawlogEntry))
				{
					break;  // file EOF
				}
				isObsBasedRawlog = (bool)observation;

				tictac.Tic();
				if (isObsBasedRawlog)
					mapBuilder.processObservation(observation);
				else
					mapBuilder.processActionObservation(*action, *observations);
				t_exec = tictac.Tac();
				RCLCPP_INFO(this->get_logger(), "Map building executed in %.03fms", 1000.0f * t_exec);

				rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(rawlog_play_delay_ * 1e9)));

				metric_map_ = mapBuilder.getCurrentlyBuiltMetricMap();

				CPose3D robotPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

				// publish map
				COccupancyGridMap2D* grid = nullptr;
				CSimplePointsMap* pm = nullptr;
				if (metric_map_.countMapsByClass<COccupancyGridMap2D>())
					grid = metric_map_.mapByClass<COccupancyGridMap2D>().get();
				if (metric_map_.countMapsByClass<CSimplePointsMap>())
					pm = metric_map_.mapByClass<CSimplePointsMap>().get();

				if (grid)
				{
					nav_msgs::msg::OccupancyGrid _msg;
					mrpt::ros2bridge::toROS(*grid, _msg);
					pub_map_->publish(_msg);
					pub_metadata_->publish(_msg.info);
				}

				if (pm)
				{
					sensor_msgs::msg::PointCloud2 _msg;
					std_msgs::msg::Header header;
					header.stamp = this->now();
					header.frame_id = global_frame_id_;
					mrpt::ros2bridge::toROS(*pm, header, _msg);
					pub_point_cloud_->publish(_msg);
				}

				// publish pose
				pose.header.frame_id = global_frame_id_;
				pose.pose.position.x = robotPose.x();
				pose.pose.position.y = robotPose.y();
				pose.pose.position.z = 0.0;
				pose.pose.orientation = createQuaternionMsgFromYaw(robotPose.yaw());

				pub_pose_->publish(pose);
			}

			run3Dwindow();
			rclcpp::spin_some(this->shared_from_this());
		}

		if (win3D_) win3D_->waitForKey();
		return true;
	}
}

void ICPslamWrapper::publishTF()
{
	const mrpt::poses::CPose3D robotPoseTF =
		mapBuilder.getCurrentPoseEstimation()->getMeanVal();

	const rclcpp::Time stamp = mrpt::ros2bridge::toROS(timeLastUpdate_);

	geometry_msgs::msg::PoseStamped odom_to_map;

	try
	{
		tf2::Transform tmp_tf =
			mrpt::ros2bridge::toROS_tfTransform(robotPoseTF);

		geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = base_frame_id_;
		tmp_tf_stamped.header.stamp = stamp;
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

		tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
	}
	catch (const tf2::TransformException&)
	{
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
		auto transform_tolerance = rclcpp::Duration::from_seconds(0.1);

		rclcpp::Time transform_expiration = stamp + transform_tolerance;

		geometry_msgs::msg::TransformStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = global_frame_id_;
		tmp_tf_stamped.header.stamp = transform_expiration;
		tmp_tf_stamped.child_frame_id = odom_frame_id_;
		tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);

		tf_broadcaster_->sendTransform(tmp_tf_stamped);
	}
}

void ICPslamWrapper::updateTrajectoryTimerCallback()
{
	RCLCPP_DEBUG(this->get_logger(), "update trajectory");
	path.header.frame_id = global_frame_id_;
	path.header.stamp = this->now();
	path.poses.push_back(pose);
}

void ICPslamWrapper::publishTrajectoryTimerCallback()
{
	RCLCPP_DEBUG(this->get_logger(), "publish trajectory");
	trajectory_pub_->publish(path);
}

void ICPslamWrapper::init3Dwindow()
{
#if MRPT_HAS_WXWIDGETS
	if (!SHOW_PROGRESS_3D_REAL_TIME) return;

	RCLCPP_INFO(this->get_logger(), "[init3Dwindow] Creating 3D window...");

	if (win3D_)
	{
		RCLCPP_ERROR(
			this->get_logger(), "[init3Dwindow] 3D window already exists! Closing and "
			"reopening...");
		win3D_.reset();
	}

	win3D_ = mrpt::gui::CDisplayWindow3D::Create(
		"ICP-SLAM @ MRPT C++ Library", 600, 500);
	win3D_->setCameraZoom(20);
	win3D_->setCameraAzimuthDeg(-45);

	// Create the 3D scene and get the map only once, later we'll modify
	// only the necessary elements.
	mrpt::opengl::COpenGLScene::Ptr scene =
		mrpt::opengl::COpenGLScene::Create();

	// The ground:
	mrpt::opengl::CGridPlaneXY::Ptr groundPlane =
		mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
	groundPlane->setColor(0.4f, 0.4f, 0.4f);
	scene->insert(groundPlane);

	// The camera pointing to the current robot pose:
	if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
	{
		mrpt::opengl::CCamera::Ptr objCam =
			mrpt::opengl::CCamera::Create();
		objCam->setName("CameraAtRobotPose");
		scene->insert(objCam);
	}

	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::RobotPioneer();
		obj->setName("robot");
		scene->insert(obj);
	}

	{
		mrpt::opengl::CPointCloud::Ptr obj = mrpt::opengl::CPointCloud::Create();
		obj->setColor(1, 0, 0);
		obj->setPointSize(1.5);
		obj->setName("points");
		scene->insert(obj);
	}

	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::CSetOfObjects::Create();
		obj->setName("robot_poses");
		scene->insert(obj);
	}

	win3D_->get3DSceneAndLock() = scene;
	win3D_->unlockAccess3DScene();
	win3D_->repaint();

	RCLCPP_INFO(this->get_logger(), "[init3Dwindow] 3D window created.");

#endif
}

void ICPslamWrapper::run3Dwindow()
{
#if MRPT_HAS_WXWIDGETS
	if (!SHOW_PROGRESS_3D_REAL_TIME) return;
	if (!win3D_) return;

	const CPose3D currentRobotPose =
		mapBuilder.getCurrentPoseEstimation()->getMeanVal();

	mrpt::opengl::COpenGLScene::Ptr scene = win3D_->get3DSceneAndLock();

	// Update the 3D view:
	if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
	{
		mrpt::opengl::CCamera::Ptr cam =
			scene->getByClass<mrpt::opengl::CCamera>();
		if (cam)
		{
			const CPose3D camPose =
				currentRobotPose + CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);
			cam->setPose(camPose);
		}
	}

	// Draw latest robot pose:
	auto obj_robot_renderable = scene->getByName("robot");
	if (obj_robot_renderable)
	{
		auto obj_robot = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(obj_robot_renderable);
		if (obj_robot) obj_robot->setPose(currentRobotPose);
	}

	// Draw laser scan:
	if (SHOW_LASER_SCANS_3D && observation)
	{
		mrpt::opengl::CPointCloud::Ptr gl_points =
			scene->getByClass<mrpt::opengl::CPointCloud>();
		if (gl_points)
		{
			CSimplePointsMap pointsMap;
			pointsMap.insertObservation(*observation);

			gl_points->loadFromPointsMap(&pointsMap);
		}
	}

	win3D_->unlockAccess3DScene();
	win3D_->repaint();

	// Update at a limited rate:
	std::this_thread::sleep_for(
		std::chrono::milliseconds(SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS));

#endif
}

}  // namespace mrpt_icp_slam_2d
