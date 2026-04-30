/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */
#pragma once

#include <mrpt/ros2bridge/time.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/map.h>

namespace mrpt
{
namespace graphslam
{
namespace apps
{
// From:
// https://answers.ros.org/question/364561/tfcreatequaternionfromyaw-equivalent-in-ros2/
static inline auto createQuaternionMsgFromYaw(double yaw)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	return tf2::toMsg(q);
}

// static member variables
template <class GRAPH_T>
const std::string CGraphSlamHandler_ROS<GRAPH_T>::sep_header(40, '=');

template <class GRAPH_T>
const std::string CGraphSlamHandler_ROS<GRAPH_T>::sep_subheader(20, '-');

// Ctor
template <class GRAPH_T>
CGraphSlamHandler_ROS<GRAPH_T>::CGraphSlamHandler_ROS(
	mrpt::system::COutputLogger* logger,
	TUserOptionsChecker<GRAPH_T>* options_checker,
	const rclcpp::NodeOptions& options)
	: parent_t(logger, options_checker, /*enable_visuals=*/false),
	  rclcpp::Node("mrpt_graphslam_2d", options)
{
	using namespace mrpt::obs;

	m_queue_size = 1;

	// variables initialization/assignment
	m_pub_seq = 0;
	m_stats_pub_seq = 0;
	this->resetReceivedFlags();

	// measurements initialization
	m_mrpt_odom = CObservationOdometry::Create();
	m_mrpt_odom->hasEncodersInfo = false;
	m_mrpt_odom->hasVelocities = false;
	m_first_time_in_sniff_odom = true;

	m_measurement_cnt = 0;
	m_graph_nodes_last_size = 0;

	// TF2 infrastructure
	m_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
	m_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

template <class GRAPH_T>
CGraphSlamHandler_ROS<GRAPH_T>::~CGraphSlamHandler_ROS()
{
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readParams()
{
	this->readROSParameters();

	ASSERT_(!this->m_ini_fname.empty());
	parent_t::readConfigFname(this->m_ini_fname);
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readROSParameters()
{
	// misc
	{
		// enable/disable visuals
		this->declare_parameter<bool>("disable_MRPT_visuals", false);
		bool m_disable_MRPT_visuals = this->get_parameter("disable_MRPT_visuals").as_bool();
		this->m_enable_visuals = !m_disable_MRPT_visuals;

		// verbosity level
		this->declare_parameter<int>("verbosity", static_cast<int>(mrpt::system::LVL_INFO));
		int verbosity_lvl = this->get_parameter("verbosity").as_int();
		m_min_logging_level = static_cast<mrpt::system::VerbosityLevel>(verbosity_lvl);
		this->m_logger->setMinLoggingLevel(m_min_logging_level);
	}
	// deciders, optimizer
	{
		this->declare_parameter<std::string>("NRD", "CFixedIntervalsNRD");
		this->declare_parameter<std::string>("ERD", "CICPCriteriaERD");
		this->declare_parameter<std::string>("GSO", "CLevMarqGSO");
		m_node_reg = this->get_parameter("NRD").as_string();
		m_edge_reg = this->get_parameter("ERD").as_string();
		m_optimizer = this->get_parameter("GSO").as_string();
	}
	// filenames
	{
		// configuration file - mandatory
		this->declare_parameter<std::string>("config_file", "");
		this->m_ini_fname = this->get_parameter("config_file").as_string();
		ASSERTMSG_(
			!this->m_ini_fname.empty(),
			"Configuration file was not set. Set 'config_file' parameter and try "
			"again.\nExiting...");

		// ground-truth file
		this->declare_parameter<std::string>("ground_truth_file", "");
		this->m_gt_fname = this->get_parameter("ground_truth_file").as_string();
	}

	// TF Frame IDs
	{
		this->declare_parameter<std::string>("anchor_frame_id", "map");
		this->declare_parameter<std::string>("base_link_frame_id", "base_link");
		this->declare_parameter<std::string>("odom_frame_id", "odom");
		m_anchor_frame_id = this->get_parameter("anchor_frame_id").as_string();
		m_base_link_frame_id = this->get_parameter("base_link_frame_id").as_string();
		m_odom_frame_id = this->get_parameter("odom_frame_id").as_string();
	}

	// ASSERT that the given user options are valid
	// Fill the TuserOptionsChecker related structures
	this->m_options_checker->createDeciderOptimizerMappings();
	this->m_options_checker->populateDeciderOptimizerProperties();
	this->verifyUserInput();

	this->m_logger->logFmt(
		mrpt::system::LVL_DEBUG, "Successfully read parameters from ROS 2 Parameter Server");

	// Visuals initialization
	if (this->m_enable_visuals)
	{
		this->initVisualization();
	}
}  // end of readROSParameters

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readStaticTFs()
{
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::initEngine_ROS()
{
	this->m_logger->logFmt(
		mrpt::system::LVL_WARN, "Initializing CGraphSlamEngine_ROS instance...");
	this->m_engine = new CGraphSlamEngine_ROS<GRAPH_T>(
		this, this->m_ini_fname,
		/*rawlog_fname=*/"", this->m_gt_fname, this->m_win_manager,
		this->m_options_checker->node_regs_map[m_node_reg](),
		this->m_options_checker->edge_regs_map[m_edge_reg](),
		this->m_options_checker->optimizers_map[m_optimizer]());
	this->m_logger->logFmt(
		mrpt::system::LVL_WARN, "Successfully initialized CGraphSlamEngine_ROS instance.");
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::getROSParameters(std::string* str_out)
{
	using namespace std;

	ASSERT_(str_out);

	stringstream ss("");

	ss << "ROS Parameters: " << endl;
	ss << sep_header << endl;
	ss << endl;

	ss << "Deciders / Optimizers = " << endl;
	ss << sep_subheader << endl;
	ss << "Node Registration Decider = " << m_node_reg << endl;
	ss << "Edge Registration Decider = " << m_edge_reg << endl;
	ss << "GraphSLAM Optimizer       = " << m_optimizer << endl;
	ss << endl;

	ss << "Filenames: " << endl;
	ss << sep_subheader << endl;
	ss << "Configuration .ini file   = " << this->m_ini_fname << endl;
	ss << "Ground truth filename     = "
	   << (!this->m_gt_fname.empty() ? this->m_gt_fname : "NONE") << endl;
	ss << endl;

	ss << "Miscellaneous: " << endl;
	ss << sep_subheader << endl;
	ss << "Enable MRPT visuals?      = "
	   << (this->m_enable_visuals ? "TRUE" : "FALSE") << endl;
	ss << "Logging verbosity Level   = "
	   << mrpt::system::COutputLogger::logging_levels_to_names()[m_min_logging_level] << endl;

	ss << endl;

	*str_out = ss.str();
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::getParamsAsString(std::string* str_out)
{
	ASSERT_(str_out);

	// ros parameters
	std::string ros_params("");
	this->getROSParameters(&ros_params);
	std::string mrpt_params("");
	parent_t::getParamsAsString(&mrpt_params);

	*str_out += ros_params;
	*str_out += "\n\n";
	*str_out += mrpt_params;

	// various parameters
}

template <class GRAPH_T>
std::string CGraphSlamHandler_ROS<GRAPH_T>::getParamsAsString()
{
	std::string params;
	this->getParamsAsString(&params);
	return params;
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::printParams()
{
	using namespace std;
	parent_t::printParams();
	cout << this->getParamsAsString() << endl;
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::verifyUserInput()
{
	this->m_logger->logFmt(mrpt::system::LVL_DEBUG, "Verifying user input...");

	// verify the NRD, ERD, GSO parameters
	bool node_success, edge_success, optimizer_success;
	bool failed = false;

	node_success = this->m_options_checker->checkRegistrationDeciderExists(
		m_node_reg, "node");
	edge_success = this->m_options_checker->checkRegistrationDeciderExists(
		m_edge_reg, "edge");
	optimizer_success =
		this->m_options_checker->checkOptimizerExists(m_optimizer);

	if (!node_success)
	{
		this->m_logger->logFmt(
			mrpt::system::LVL_ERROR, "\nNode Registration Decider \"%s\" is not available",
			m_node_reg.c_str());
		this->m_options_checker->dumpRegistrarsToConsole("node");
		failed = true;
	}
	if (!edge_success)
	{
		this->m_logger->logFmt(
			mrpt::system::LVL_ERROR, "\nEdge Registration Decider \"%s\" is not available.",
			m_edge_reg.c_str());
		this->m_options_checker->dumpRegistrarsToConsole("edge");
		failed = true;
	}
	if (!optimizer_success)
	{
		this->m_logger->logFmt(
			mrpt::system::LVL_ERROR, "\ngraphSLAM Optimizser \"%s\" is not available.",
			m_optimizer.c_str());
		this->m_options_checker->dumpOptimizersToConsole();
		failed = true;
	}
	ASSERT_(!failed);

	// verify that the path to the files is correct
	// .ini file
	bool ini_exists = system::fileExists(this->m_ini_fname);
	ASSERTMSG_(
		ini_exists, format(
						"\n.ini configuration file \"%s\"doesn't exist. "
						"Please specify a valid pathname.\nExiting...\n",
						this->m_ini_fname.c_str()));
	// ground-truth file
	if (!this->m_gt_fname.empty())
	{
		bool gt_exists = system::fileExists(this->m_gt_fname);
		ASSERTMSG_(
			gt_exists, format(
						   "\nGround truth file \"%s\"doesn't exist."
						   "Either unset the corresponding ROS parameter or "
						   "specify a valid pathname.\n"
						   "Exiting...\n",
						   this->m_gt_fname.c_str()));
	}

}  // end of verifyUserInput

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupComm()
{
	this->m_logger->logFmt(
		mrpt::system::LVL_INFO,
		"Setting up ROS 2 subscribers, publishers, services...");

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

}  // end of setupComm

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupSubs()
{
	this->m_logger->logFmt(mrpt::system::LVL_INFO, "Setting up the subscribers...");

	// setup the names
	std::string ns = "input/";

	m_odom_topic = ns + "odom";
	m_laser_scan_topic = ns + "laser_scan";

	// odometry
	m_odom_sub = this->template create_subscription<nav_msgs::msg::Odometry>(
		m_odom_topic, m_queue_size,
		std::bind(&self_t::sniffOdom, this, std::placeholders::_1));

	// laser_scans
	m_laser_scan_sub = this->template create_subscription<sensor_msgs::msg::LaserScan>(
		m_laser_scan_topic, m_queue_size,
		std::bind(&self_t::sniffLaserScan, this, std::placeholders::_1));

}  // end of setupSubs

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupPubs()
{
	this->m_logger->logFmt(mrpt::system::LVL_INFO, "Setting up the publishers...");

	// setup the names
	std::string ns = "feedback/";

	m_curr_robot_pos_topic = ns + "robot_position";
	m_robot_trajectory_topic = ns + "robot_trajectory";
	m_robot_tr_poses_topic = ns + "robot_tr_poses";
	m_odom_trajectory_topic = ns + "odom_trajectory";
	m_gridmap_topic = ns + "gridmap";
	m_stats_topic = ns + "graphslam_stats";

	// setup the publishers

	// agent estimated position (latched via transient_local)
	m_curr_robot_pos_pub = this->template create_publisher<geometry_msgs::msg::PoseStamped>(
		m_curr_robot_pos_topic, rclcpp::QoS(m_queue_size).transient_local());
	m_robot_trajectory_pub = this->template create_publisher<nav_msgs::msg::Path>(
		m_robot_trajectory_topic, rclcpp::QoS(m_queue_size).transient_local());
	m_robot_tr_poses_pub = this->template create_publisher<geometry_msgs::msg::PoseArray>(
		m_robot_tr_poses_topic, rclcpp::QoS(m_queue_size).transient_local());

	// odometry nav_msgs::msg::Path
	m_odom_path.header.stamp = this->now();
	m_odom_path.header.frame_id = m_anchor_frame_id;

	m_odom_trajectory_pub = this->template create_publisher<nav_msgs::msg::Path>(
		m_odom_trajectory_topic, m_queue_size);

	// generated gridmap (latched)
	m_gridmap_pub = this->template create_publisher<nav_msgs::msg::OccupancyGrid>(
		m_gridmap_topic, rclcpp::QoS(m_queue_size).transient_local());

	m_stats_pub = this->template create_publisher<mrpt_msgs::msg::GraphSlamStats>(
		m_stats_topic, rclcpp::QoS(m_queue_size).transient_local());

}  // end of setupPubs

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupSrvs()
{
	this->m_logger->logFmt(mrpt::system::LVL_INFO, "Setting up the services...");

	// TODO Error statistics

}  // end of setupSrvs

template <class GRAPH_T>
bool CGraphSlamHandler_ROS<GRAPH_T>::usePublishersBroadcasters()
{
	using namespace std;

	MRPT_START;
	bool ret_val = true;

	rclcpp::Time timestamp = this->now();

	// current MRPT robot pose
	pose_t mrpt_pose = this->m_engine->getCurrentRobotPosEstimation();

	//
	// convert pose_t to corresponding geometry_msgs::msg::TransformStamped
	// anchor frame <=> base_link
	//
	geometry_msgs::msg::TransformStamped anchor_base_link_transform;
	anchor_base_link_transform.header.stamp = this->now();
	anchor_base_link_transform.header.frame_id = m_anchor_frame_id;
	anchor_base_link_transform.child_frame_id = m_base_link_frame_id;

	// translation
	anchor_base_link_transform.transform.translation.x = mrpt_pose.x();
	anchor_base_link_transform.transform.translation.y = mrpt_pose.y();
	anchor_base_link_transform.transform.translation.z = 0;

	// rotation
	anchor_base_link_transform.transform.rotation =
		createQuaternionMsgFromYaw(mrpt_pose.phi());

	m_broadcaster->sendTransform(anchor_base_link_transform);

	// anchor frame <=> odom frame
	//
	// make sure that we have received odometry information in the first
	// place...
	// the corresponding field would be initialized
	if (!m_anchor_odom_transform.child_frame_id.empty())
	{
		m_broadcaster->sendTransform(m_anchor_odom_transform);
	}

	// set an arrow indicating the current orientation of the robot
	{
		geometry_msgs::msg::PoseStamped geom_pose;
		geom_pose.header.stamp = timestamp;
		geom_pose.header.frame_id = m_anchor_frame_id;

		// position
		geom_pose.pose = mrpt::ros2bridge::toROS_Pose(mrpt_pose);
		m_curr_robot_pos_pub->publish(geom_pose);
	}

	// robot trajectory
	// publish the trajectory of the robot
	{
		this->m_logger->logFmt(
			mrpt::system::LVL_DEBUG, "Publishing the current robot trajectory");
		typename GRAPH_T::global_poses_t graph_poses;
		graph_poses = this->m_engine->getRobotEstimatedTrajectory();

		nav_msgs::msg::Path path;

		// set the header
		path.header.stamp = timestamp;
		path.header.frame_id = m_anchor_frame_id;

		//
		// fill in the pose as well as the nav_msgs::msg::Path at the same time.
		//

		geometry_msgs::msg::PoseArray geom_poses;
		geom_poses.header.stamp = timestamp;
		geom_poses.header.frame_id = m_anchor_frame_id;

		for (auto n_cit = graph_poses.begin(); n_cit != graph_poses.end();
			 ++n_cit)
		{
			geometry_msgs::msg::PoseStamped geom_pose_stamped;
			geometry_msgs::msg::Pose geom_pose;

			// grab the pose - convert to geometry_msgs::msg::Pose format
			const pose_t& mrpt_pose = n_cit->second;
			geom_pose = mrpt::ros2bridge::toROS_Pose(mrpt_pose);
			geom_poses.poses.push_back(geom_pose);
			geom_pose_stamped.pose = geom_pose;

			// edit the header
			geom_pose_stamped.header.stamp = timestamp;
			geom_pose_stamped.header.frame_id = m_anchor_frame_id;

			path.poses.push_back(geom_pose_stamped);
		}

		// publish only on new node addition
		if (this->m_engine->getGraph().nodeCount() > m_graph_nodes_last_size)
		{
			m_robot_tr_poses_pub->publish(geom_poses);
			m_robot_trajectory_pub->publish(path);
		}
	}

	// Odometry trajectory - nav_msgs::msg::Path
	m_odom_trajectory_pub->publish(m_odom_path);

	// generated gridmap
	// publish only on new node addition
	if (this->m_engine->getGraph().nodeCount() > m_graph_nodes_last_size)
	{
		std_msgs::msg::Header h;
		mrpt::system::TTimeStamp mrpt_time;
		auto mrpt_gridmap = mrpt::maps::COccupancyGridMap2D::Create();
		this->m_engine->getMap(mrpt_gridmap, &mrpt_time);

		// timestamp
		h.stamp = mrpt::ros2bridge::toROS(mrpt_time);
		h.frame_id = m_anchor_frame_id;

		// nav gridmap
		nav_msgs::msg::OccupancyGrid nav_gridmap;
		mrpt::ros2bridge::toROS(*mrpt_gridmap, nav_gridmap, h);
		m_gridmap_pub->publish(nav_gridmap);
	}

	// GraphSlamStats publishing
	{
		mrpt_msgs::msg::GraphSlamStats stats;
		stats.header.stamp = this->now();

		map<string, int> node_stats;
		map<string, int> edge_stats;
		vector<double> def_energy_vec;
		mrpt::system::TTimeStamp mrpt_time;

		this->m_engine->getGraphSlamStats(&node_stats, &edge_stats, &mrpt_time);

		// node/edge count
		stats.nodes_total = node_stats["nodes_total"];
		stats.edges_total = edge_stats["edges_total"];
		if (edge_stats.find("ICP2D") != edge_stats.end())
		{
			stats.edges_icp_2d = edge_stats["ICP2D"];
		}
		if (edge_stats.find("ICP3D") != edge_stats.end())
		{
			stats.edges_icp_3d = edge_stats["ICP3D"];
		}
		if (edge_stats.find("Odometry") != edge_stats.end())
		{
			stats.edges_odom = edge_stats["Odometry"];
		}
		stats.loop_closures = edge_stats["loop_closures"];

		// SLAM evaluation metric
		this->m_engine->getDeformationEnergyVector(
			&stats.slam_evaluation_metric);

		stats.header.stamp = mrpt::ros2bridge::toROS(mrpt_time);

		m_stats_pub->publish(stats);
	}

	// update the last known size
	m_graph_nodes_last_size = this->m_engine->getGraph().nodeCount();
	m_pub_seq++;

	if (this->m_enable_visuals)
	{
		ret_val = this->queryObserverForEvents();
	}

	return ret_val;
	MRPT_END;
}  // end of usePublishersBroadcasters

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffLaserScan(
	const sensor_msgs::msg::LaserScan::SharedPtr ros_laser_scan)
{
	using namespace std;
	using namespace mrpt::obs;

	this->m_logger->logFmt(
		mrpt::system::LVL_DEBUG,
		"sniffLaserScan: Received a LaserScan msg. Converting it to MRPT "
		"format...");

	// build the CObservation2DRangeScan
	m_mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;	// pose is 0.
	mrpt::ros2bridge::fromROS(*ros_laser_scan, rel_pose, *m_mrpt_laser_scan);

	m_received_laser_scan = true;
	CObservation::Ptr tmp =
		mrpt::ptr_cast<CObservation>::from(m_mrpt_laser_scan);
	this->processObservation(tmp);
}  // end of sniffLaserScan

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffOdom(
	const nav_msgs::msg::Odometry::SharedPtr ros_odom)
{
	using namespace std;
	using namespace mrpt::obs;
	using namespace mrpt::poses;
	using namespace mrpt::system;

	this->m_logger->logFmt(
		mrpt::system::LVL_DEBUG,
		"sniffOdom: Received an odometry msg. Converting it to MRPT format...");

	// update the odometry frame with regards to the anchor
	{
		// header
		m_anchor_odom_transform.header.frame_id = m_anchor_frame_id;
		m_anchor_odom_transform.header.stamp = ros_odom->header.stamp;

		m_anchor_odom_transform.child_frame_id = m_odom_frame_id;

		// translation
		m_anchor_odom_transform.transform.translation.x =
			ros_odom->pose.pose.position.x;
		m_anchor_odom_transform.transform.translation.y =
			ros_odom->pose.pose.position.y;
		m_anchor_odom_transform.transform.translation.z =
			ros_odom->pose.pose.position.z;

		// quaternion
		m_anchor_odom_transform.transform.rotation =
			ros_odom->pose.pose.orientation;
	}

	// build and fill an MRPT CObservationOdometry instance for manipulation
	// from the main algorithm
	m_mrpt_odom->timestamp = mrpt::ros2bridge::fromROS(ros_odom->header.stamp);
	m_mrpt_odom->odometry =
		mrpt::poses::CPose2D(mrpt::ros2bridge::fromROS(ros_odom->pose.pose));

	// if this is the first call odometry should be 0. Decrement by the
	// corresponding offset
	if (m_first_time_in_sniff_odom)
	{
		m_input_odometry_offset = m_mrpt_odom->odometry;
		m_first_time_in_sniff_odom = false;
	}
	// decrement by the (constant) offset
	m_mrpt_odom->odometry = m_mrpt_odom->odometry - m_input_odometry_offset;

	// add to the overall odometry path
	{
		geometry_msgs::msg::PoseStamped pose_stamped;
		pose_stamped.header = ros_odom->header;

		// just for convenience - convert the MRPT pose back to PoseStamped
		pose_stamped.pose = mrpt::ros2bridge::toROS_Pose(m_mrpt_odom->odometry);
		m_odom_path.poses.push_back(pose_stamped);
	}

	// print the odometry -  for debugging reasons...
	stringstream ss;
	ss << "Odometry - MRPT format:\t" << m_mrpt_odom->odometry << endl;
	this->m_logger->logFmt(mrpt::system::LVL_DEBUG, "%s", ss.str().c_str());

	m_received_odom = true;
	CObservation::Ptr tmp =
		mrpt::ptr_cast<mrpt::obs::CObservation>::from(m_mrpt_odom);
	this->processObservation(tmp);
}  // end of sniffOdom

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffCameraImage()
{
	THROW_EXCEPTION("Method is not implemented yet.");
}
template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniff3DPointCloud()
{
	THROW_EXCEPTION("Method is not implemented yet.");
}
template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::processObservation(
	mrpt::obs::CObservation::Ptr& observ)
{
	using namespace std;

	this->_process(observ);
	this->resetReceivedFlags();
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::_process(
	mrpt::obs::CObservation::Ptr& observ)
{
	if (!this->m_engine->isPaused())
	{
		this->m_engine->execGraphSlamStep(observ, m_measurement_cnt);
		m_measurement_cnt++;
	}
}

template <class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::resetReceivedFlags()
{
	m_received_odom = false;
	m_received_laser_scan = false;
	m_received_camera = false;
	m_received_point_cloud = false;
}

}  // namespace apps
}  // namespace graphslam
}  // namespace mrpt
