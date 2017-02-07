/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CGRAPHSLAMHANDLER_ROS_IMPL_H
#define CGRAPHSLAMHANDLER_ROS_IMPL_H

namespace mrpt { namespace graphslam { namespace apps {

using mrpt::utils::LVL_DEBUG;
using mrpt::utils::LVL_INFO;
using mrpt::utils::LVL_WARN;
using mrpt::utils::LVL_ERROR;

// static member variables
template<class GRAPH_T>
const std::string CGraphSlamHandler_ROS<GRAPH_T>::sep_header(40, '=');

template<class GRAPH_T>
const std::string CGraphSlamHandler_ROS<GRAPH_T>::sep_subheader(20, '-');

// Ctor
template<class GRAPH_T>
CGraphSlamHandler_ROS<GRAPH_T>::CGraphSlamHandler_ROS(
		mrpt::utils::COutputLogger* logger,
		TUserOptionsChecker<GRAPH_T>* options_checker,
		ros::NodeHandle* nh_in):
	m_nh(nh_in),
	parent_t(logger, options_checker, /*enable_visuals=*/ false)
{
	using namespace mrpt::obs;

	ASSERT_(m_nh);

	// TODO - does this affect?
	// Previous value = 0;
	m_queue_size = 1;

	// variables initialization/assignment
	m_has_read_config = false;
	m_pub_seq = 0;
	m_stats_pub_seq = 0;
	this->resetReceivedFlags();

	// measurements initialization
	m_mrpt_odom = CObservationOdometry::Create();
	m_mrpt_odom->hasEncodersInfo = false;
	m_mrpt_odom->hasVelocities = false;
	m_first_time_in_sniff_odom = true;

	m_measurement_cnt = 0;

	// Thu Nov 3 23:36:49 EET 2016, Nikos Koukis
	// WARNING: ROS Server Parameters have not been read yet. Make sure you know
	// what to initialize at this stage!
}

template<class GRAPH_T>
CGraphSlamHandler_ROS<GRAPH_T>::~CGraphSlamHandler_ROS() { }

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readParams() {
	this->readROSParameters();

	ASSERT_(!this->m_ini_fname.empty());
	parent_t::readConfigFname(this->m_ini_fname);

	m_has_read_config = true;
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readROSParameters() {
	using namespace mrpt::utils;

	// misc
	{
		std::string ns = "misc/";

		// enable/disable visuals
		bool m_disable_MRPT_visuals;
		m_nh->param<bool>(ns + "disable_MRPT_visuals", m_disable_MRPT_visuals, false);
		this->m_enable_visuals = !m_disable_MRPT_visuals;

		// verbosity level
		int lvl;
		m_nh->param<int>(ns + "verbosity",
				lvl,
				static_cast<int>(LVL_INFO));
		m_min_logging_level = static_cast<VerbosityLevel>(lvl);
		this->m_logger->setMinLoggingLevel(m_min_logging_level);
	}
	// deciders, optimizer
	{
		std::string ns = "deciders_optimizers/";
		m_nh->param<std::string>(ns + "NRD", m_node_reg, "CFixedIntervalsNRD");
		m_nh->param<std::string>(ns + "ERD", m_edge_reg, "CICPCriteriaERD");
		m_nh->param<std::string>(ns + "GSO", m_optimizer, "CLevMarqGSO");
	}
	// filenames
	{
		std::string ns = "files/";

		// configuration file - mandatory
		std::string config_param_path = ns + "config";
		bool found_config = m_nh->getParam(ns + "config", this->m_ini_fname);
		ASSERTMSG_(found_config,
				mrpt::format(
					"Configuration file was not set. Set %s and try again.\nExiting...",
					config_param_path.c_str()));

		// ground-truth file
		m_nh->getParam(ns + "ground_truth", this->m_gt_fname);
	}

	// TF Frame IDs
	// names of the frames of the corresponding robot parts
	{
		std::string ns = "frame_IDs/";

		m_nh->param<std::string>(ns + "anchor_frame" , m_anchor_frame_id, "map");
		m_nh->param<std::string>(ns + "base_link_frame" , m_base_link_frame_id, "base_link");
		m_nh->param<std::string>(ns + "odometry_frame" , m_odom_frame_id, "odom");
		m_nh->param<std::string>(ns + "laser_frame" , m_laser_frame_id, "laser");

		// take action based on the above frames
		//

	}

	// ASSERT that the given user options are valid
	// Fill the TuserOptionsChecker related structures
	this->m_options_checker->createDeciderOptimizerMappings();
	this->m_options_checker->populateDeciderOptimizerProperties();
	this->verifyUserInput();

	this->m_logger->logFmt(LVL_DEBUG,
			"Successfully read parameters from ROS Parameter Server");

	// Visuals initialization
	if (this->m_enable_visuals) {
		this->initVisualization();
	}
} // end of readROSParameters

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::readStaticTFs() {

	// base_link => laser
	this->m_logger->logFmt(LVL_WARN,
			"Looking up static transform...%s => %s",
			m_laser_frame_id.c_str(),
			m_base_link_frame_id.c_str());
	try {
		m_base_laser_transform = m_buffer.lookupTransform(
				/* target_fname = */ m_laser_frame_id,
				/* source_fname = */ m_base_link_frame_id,
				/* transform time = */ ros::Time(0));
		this->m_logger->logFmt(LVL_INFO, "OK.");
	}
	catch (tf2::LookupException &ex) {
		this->m_logger->logFmt(LVL_WARN,
				"Transformation not found, using default...");

		m_base_laser_transform.header.stamp = ros::Time::now();
		m_base_laser_transform.transform.translation.x = 0.0;
		m_base_laser_transform.transform.translation.y = 0.0;
		m_base_laser_transform.transform.translation.z = 0.0;
		m_base_laser_transform.transform.rotation.x = 0.0;
		m_base_laser_transform.transform.rotation.y = 0.0;
		m_base_laser_transform.transform.rotation.z = 0.0;
		m_base_laser_transform.transform.rotation.w = 1.0;

  	m_base_laser_transform.header.frame_id = m_base_link_frame_id;
  	m_base_laser_transform.child_frame_id = m_laser_frame_id;
	}

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::initEngine_ROS() { 

	this->m_logger->logFmt(LVL_WARN,
			"Initializing CGraphSlamEngine_ROS instance...");
	this->m_engine = new CGraphSlamEngine_ROS<GRAPH_T>(
			m_nh,
			this->m_ini_fname,
			/*rawlog_fname=*/ "",
			this->m_gt_fname,
			this->m_win_manager,
			this->m_options_checker->node_regs_map[m_node_reg](),
			this->m_options_checker->edge_regs_map[m_edge_reg](),
			this->m_options_checker->optimizers_map[m_optimizer]());
	this->m_logger->logFmt(LVL_WARN,
			"Successfully initialized CGraphSlamEngine_ROS instance.");

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::initEngine_CM() { 

	this->m_options_checker->node_regs_map[m_node_reg]();
	this->m_options_checker->edge_regs_map[m_edge_reg]();
	this->m_options_checker->optimizers_map[m_optimizer]();

	this->m_logger->logFmt(LVL_WARN,
			"Initializing CGraphSlamEngine_CM instance...");
	this->m_engine = new CGraphSlamEngine_CM<GRAPH_T>(
			m_nh,
			this->m_ini_fname,
			/*rawlog_fname=*/ "",
			this->m_gt_fname,
			this->m_win_manager,
			this->m_options_checker->node_regs_map[m_node_reg](),
			this->m_options_checker->edge_regs_map[m_edge_reg](),
			this->m_options_checker->optimizers_map[m_optimizer]());
	this->m_logger->logFmt(LVL_WARN,
			"Successfully initialized CGraphSlamEngine_CM instance.");

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::getROSParameters(std::string* str_out) {
	using namespace mrpt::utils;

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
	ss << "Ground truth filename     = " << (!this->m_gt_fname.empty()
			? this->m_gt_fname : "NONE")
		<< endl;
	ss << endl;

	ss << "Miscellaneous: " << endl;
	ss << sep_subheader << endl;
	ss << "Enable MRPT visuals?      = " <<
		(this->m_enable_visuals? "TRUE" : "FALSE")
		<< endl;
	ss << "Logging verbosity Level   = " << 
		COutputLogger::logging_levels_to_names[m_min_logging_level] << endl;;
	ss << endl;

	*str_out = ss.str();
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::getParamsAsString(std::string* str_out) {
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

template<class GRAPH_T>
std::string CGraphSlamHandler_ROS<GRAPH_T>::getParamsAsString() {
	std::string params;
	this->getParamsAsString(&params);
	return params;
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::printParams() {
	parent_t::printParams();
	cout << this->getParamsAsString() << endl;
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::verifyUserInput() {
	this->m_logger->logFmt(LVL_DEBUG, "Verifying user input...");


	// verify the NRD, ERD, GSO parameters
	bool node_success, edge_success, optimizer_success;
	bool failed = false;

	node_success =
		this->m_options_checker->checkRegistrationDeciderExists(
				m_node_reg, "node");
	edge_success =
		this->m_options_checker->checkRegistrationDeciderExists(
				m_edge_reg, "edge");
	optimizer_success =
		this->m_options_checker->checkOptimizerExists(
				m_optimizer);

	if (!node_success) {
		this->m_logger->logFmt(LVL_ERROR,
				"\nNode Registration Decider \"%s\" is not available",
				m_node_reg.c_str());
		this->m_options_checker->dumpRegistrarsToConsole("node");
		failed = true;
	}
	if (!edge_success) {
		this->m_logger->logFmt(LVL_ERROR,
				"\nEdge Registration Decider \"%s\" is not available.",
				m_edge_reg.c_str());
		this->m_options_checker->dumpRegistrarsToConsole("edge");
		failed = true;
	}
	if (!optimizer_success) {
		this->m_logger->logFmt(LVL_ERROR,
				"\ngraphSLAM Optimizser \"%s\" is not available.",
				m_optimizer.c_str());
		this->m_options_checker->dumpOptimizersToConsole();
		failed = true;
	}
	ASSERT_(!failed)

	// verify that the path to the files is correct
	// .ini file
	bool ini_exists = system::fileExists(this->m_ini_fname);
	ASSERTMSG_(ini_exists,
			format(
				"\n.ini configuration file \"%s\"doesn't exist. "
				"Please specify a valid pathname.\nExiting...\n",
				this->m_ini_fname.c_str()));
	// ground-truth file
	if (!this->m_gt_fname.empty()) {
		bool gt_exists = system::fileExists(this->m_gt_fname);
		ASSERTMSG_(gt_exists,
				format(
					"\nGround truth file \"%s\"doesn't exist."
					"Either unset the corresponding ROS parameter or specify a valid pathname.\n"
					"Exiting...\n",
					this->m_gt_fname.c_str()));
	}

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupComm() {

	this->m_logger->logFmt(LVL_INFO,
			"Setting up ROS-related subscribers, publishers, services...");

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	// fetch the static geometrical transformations
	//this->readStaticTFs();

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupSubs() {
	this->m_logger->logFmt(LVL_INFO, "Setting up the subscribers...");
	
	// setup the names
	std::string ns = "input/";

	m_odom_topic = ns + "odom";
	m_laser_scan_topic = ns + "laser_scan";

	// odometry
	m_odom_sub = m_nh->subscribe<nav_msgs::Odometry>(
			m_odom_topic,
			m_queue_size,
			&self_t::sniffOdom, this);

	// laser_scans
	m_laser_scan_sub = m_nh->subscribe<sensor_msgs::LaserScan>(
			m_laser_scan_topic,
			m_queue_size,
			&self_t::sniffLaserScan, this);
	
	// camera
	// TODO
	
	// 3D point clouds
	// TODO

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupPubs() {
	this->m_logger->logFmt(LVL_INFO, "Setting up the publishers...");

	// setup the names
	std::string ns = "feedback/";

	m_curr_robot_pos_topic = ns + "robot_position";
	m_robot_trajectory_topic = ns + "robot_trajectory";
	m_robot_tr_poses_topic = ns + "robot_tr_poses";
	m_odom_trajectory_topic = ns + "odom_trajectory";
	m_gridmap_topic = ns + "gridmap";
	m_stats_topic = ns + "graphslam_stats";

	// setup the publishers

	// agent estimated position
	m_curr_robot_pos_pub = m_nh->advertise<geometry_msgs::PoseStamped>(
			m_curr_robot_pos_topic,
			m_queue_size);
	 m_robot_trajectory_pub = m_nh->advertise<nav_msgs::Path>(
	 		 m_robot_trajectory_topic,
	 		 m_queue_size);
	 m_robot_tr_poses_pub = m_nh->advertise<geometry_msgs::PoseArray>(
	 		 m_robot_tr_poses_topic,
	 		 m_queue_size);

	 // odometry nav_msgs::Path
	 m_odom_path.header.seq = 0;
	 m_odom_path.header.stamp = ros::Time::now();
	 m_odom_path.header.frame_id = m_anchor_frame_id;

	 m_odom_trajectory_pub = m_nh->advertise<nav_msgs::Path>(
	 		 m_odom_trajectory_topic,
	 		 m_queue_size);

	 // generated gridmap
	 m_gridmap_pub = m_nh->advertise<nav_msgs::OccupancyGrid>(
	 		 m_gridmap_topic,
	 		 m_queue_size,
	 		 /*latch=*/true);

	 m_stats_pub = m_nh->advertise<mrpt_msgs::GraphSlamStats>(
	 		 m_stats_topic,
	 		 m_queue_size,
	 		 /*latch=*/true);

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::setupSrvs() {
	this->m_logger->logFmt(LVL_INFO, "Setting up the services...");

	// SLAM statistics
	// Error statistics

}

template<class GRAPH_T>
bool CGraphSlamHandler_ROS<GRAPH_T>::usePublishersBroadcasters() {
	using namespace mrpt::utils;


this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 29);
	ros::Time timestamp = ros::Time::now();

	// current MRPT robot pose
	pose_t mrpt_pose = this->m_engine->getCurrentRobotPosEstimation();

	// anchor frame <=> base_link
  {
		// fill the geometry_msgs::TransformStamped object
		geometry_msgs::TransformStamped transform_stamped;
		transform_stamped.header.stamp = ros::Time::now();
		transform_stamped.header.frame_id = m_anchor_frame_id;
		transform_stamped.child_frame_id = m_base_link_frame_id;

		transform_stamped.transform.translation.x = mrpt_pose.x();
		transform_stamped.transform.translation.y = mrpt_pose.y();
		transform_stamped.transform.translation.z = 0;

		tf2::Quaternion q;
		q.setRPY(0, 0, mrpt_pose.phi());
		tf2::Vector3 axis = q.getAxis();
		tf2Scalar w = q.getW();
		transform_stamped.transform.rotation.x = axis.getX();
		transform_stamped.transform.rotation.y = axis.getY();
		transform_stamped.transform.rotation.z = axis.getZ();
		transform_stamped.transform.rotation.w = w;

		m_broadcaster.sendTransform(transform_stamped);
  }

this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 30);
  // anchor frame <=> odom frame
  //
  // make sure that we have received odometry information in the first
  // place...
  // the corresponding field would be initialized
  if (!m_anchor_odom_transform.child_frame_id.empty()) {
  	m_broadcaster.sendTransform(m_anchor_odom_transform);
  }
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 31);

  // set an arrow indicating clearly the current orientation of the robot
	{
		geometry_msgs::PoseStamped geom_pose;

		geom_pose.header.stamp = timestamp;
		geom_pose.header.seq = m_pub_seq;
		geom_pose.header.frame_id = m_base_link_frame_id; // with regards to base_link...

		geometry_msgs::Point point;
		point.x = 0;
		point.y = 0;
		point.z = 0;
		geometry_msgs::Quaternion quat;
		quat.x = 0;
		quat.y = 0;
		quat.z = 0;
		quat.w = 1;

		geom_pose.pose.position = point;
		geom_pose.pose.orientation = quat;
		m_curr_robot_pos_pub.publish(geom_pose);
	}
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 32);

	// robot trajectory
	// publish the trajectory of the robot
	{
		this->m_logger->logFmt(LVL_DEBUG,
				"Publishing the current robot trajectory");
		typename GRAPH_T::global_poses_t graph_poses;
		this->m_engine->getRobotEstimatedTrajectory(&graph_poses);

		nav_msgs::Path path;

		// set the header
		path.header.stamp = timestamp;
		path.header.seq = m_pub_seq;
		path.header.frame_id = m_anchor_frame_id;

		//
		// fill in the pose as well as the nav_msgs::Path at the same time.
		//

		geometry_msgs::PoseArray geom_poses;
		geom_poses.header.stamp = timestamp;
		geom_poses.header.frame_id = m_anchor_frame_id;

		for (int i = 0; i != graph_poses.size(); ++i) {
			geometry_msgs::PoseStamped geom_pose_stamped;
			geometry_msgs::Pose geom_pose;

			// grab the pose - convert to geometry_msgs::Pose format
			pose_t mrpt_pose = graph_poses.at(i);
			mrpt_bridge::convert(mrpt_pose, geom_pose);
			geom_poses.poses.push_back(geom_pose);
			geom_pose_stamped.pose = geom_pose;

			// edit the header
			geom_pose_stamped.header.stamp = timestamp;
			geom_pose_stamped.header.seq = m_pub_seq;
			geom_pose_stamped.header.frame_id = m_anchor_frame_id;

			path.poses.push_back(geom_pose_stamped);
		}

		m_robot_tr_poses_pub.publish(geom_poses);
		m_robot_trajectory_pub.publish(path);
	}

	// Odometry trajectory - nav_msgs::Path
	m_odom_trajectory_pub.publish(m_odom_path);
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 33);

	// generated gridmap
	{
		std_msgs::Header h;
		mrpt::system::TTimeStamp mrpt_time;
		mrpt::maps::COccupancyGridMap2DPtr mrpt_gridmap =
			mrpt::maps::COccupancyGridMap2D::Create();
		this->m_engine->getMap(mrpt_gridmap, &mrpt_time);

		// timestamp
		mrpt_bridge::convert(mrpt_time, h.stamp);
		h.seq = m_pub_seq;
		h.frame_id = m_anchor_frame_id;

		// nav gridmap
		nav_msgs::OccupancyGrid nav_gridmap;
		mrpt_bridge::convert(*mrpt_gridmap, nav_gridmap, h);
		m_gridmap_pub.publish(nav_gridmap);
	}

	// GraphSlamStats publishing
	{
		mrpt_msgs::GraphSlamStats stats;
		stats.header.seq = m_stats_pub_seq++;

		map<string, int>  node_stats;
		map<string, int>  edge_stats;
		vector<double> def_energy_vec;
		mrpt::system::TTimeStamp mrpt_time;

		bool ret = this->m_engine->getGraphSlamStats(&node_stats,
				&edge_stats,
				&mrpt_time);

		if (ret) {
			// node/edge count
			stats.nodes_total = node_stats["nodes_total"];
			stats.edges_total = edge_stats["edges_total"];
			if (edge_stats.find("ICP2D") != edge_stats.end()) {
				stats.edges_ICP2D = edge_stats["ICP2D"];
			}
			if (edge_stats.find("ICP3D") != edge_stats.end()) {
				stats.edges_ICP3D = edge_stats["ICP3D"];
			}
			if (edge_stats.find("Odometry") != edge_stats.end()) {
				stats.edges_odom = edge_stats["Odometry"];
			}
			stats.loop_closures = edge_stats["loop_closures"];

			// SLAM evaluation metric
			this->m_engine->getDeformationEnergyVector(
					&stats.slam_evaluation_metric);

			mrpt_bridge::convert(mrpt_time, stats.header.stamp);

			m_stats_pub.publish(stats);
		}
		else {
			this->m_logger->logFmt(LVL_ERROR, "Answer is False");
		}
		
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 34);
		

	}

	m_pub_seq++;
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 35);

	if (this->m_enable_visuals) {
		return this->queryObserverForEvents();
	}
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 36);
} // USEPUBLISHERSBROADCASTERS

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffLaserScan(
		const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	this->m_logger->logFmt(
			LVL_DEBUG,
			"sniffLaserScan: Received a LaserScan msg. Converting it to MRPT format...");

	// build the CObservation2DRangeScan
	m_mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;
	mrpt_bridge::convert(*ros_laser_scan, rel_pose, *m_mrpt_laser_scan);

	m_received_laser_scan = true;
	this->processObservation(m_mrpt_laser_scan);
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 40);
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffOdom(
		const nav_msgs::Odometry::ConstPtr& ros_odom) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::poses;

	this->m_logger->logFmt(
			LVL_DEBUG,
			"sniffOdom: Received an odometry msg. Converting it to MRPT format...");

	// update the odometry frame with regards to the anchor
	{
		// header
		m_anchor_odom_transform.header.frame_id = m_anchor_frame_id;
		m_anchor_odom_transform.header.stamp = ros_odom->header.stamp;
		m_anchor_odom_transform.header.seq = ros_odom->header.seq;

		m_anchor_odom_transform.child_frame_id = m_odom_frame_id;

		// translation
		m_anchor_odom_transform.transform.translation.x =
			ros_odom->pose.pose.position.x;
		m_anchor_odom_transform.transform.translation.y =
			ros_odom->pose.pose.position.y;
		m_anchor_odom_transform.transform.translation.z =
			ros_odom->pose.pose.position.z;

		// quaternion
		m_anchor_odom_transform.transform.rotation.x =
			ros_odom->pose.pose.orientation.x;
		m_anchor_odom_transform.transform.rotation.y =
			ros_odom->pose.pose.orientation.y;
		m_anchor_odom_transform.transform.rotation.z =
			ros_odom->pose.pose.orientation.z;
		m_anchor_odom_transform.transform.rotation.w =
			ros_odom->pose.pose.orientation.w;
	}

	// build and fill an MRPT CObservationOdometry instance for manipulation from
	// the main algorithm
	mrpt_bridge::convert(
			/* src = */ ros_odom->header.stamp,
			/* dst = */ m_mrpt_odom->timestamp);
	mrpt_bridge::convert(
			/* src = */ ros_odom->pose.pose,
			/* dst = */ m_mrpt_odom->odometry);

	// if this is the first call odometry should be 0. Decrement by the
	// corresponding offset
	if (m_first_time_in_sniff_odom) {
		m_input_odometry_offset = m_mrpt_odom->odometry;
		m_first_time_in_sniff_odom = false;
	}
	// decrement by the (constant) offset
	m_mrpt_odom->odometry =
		m_mrpt_odom->odometry - m_input_odometry_offset;
	
  // add to the overall odometry path
  {
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header = ros_odom->header;

  	// just for convenience - convert the MRPT pose back to PoseStamped
		mrpt_bridge::convert(
				/* src = */ m_mrpt_odom->odometry,
				/* des = */ pose_stamped.pose);
  	m_odom_path.poses.push_back(pose_stamped);
  }

	// print the odometry -  for debugging reasons...
	stringstream ss;
	ss << "Odometry - MRPT format:\t" << m_mrpt_odom->odometry << endl;
	this->m_logger->logFmt(LVL_DEBUG, "%s", ss.str().c_str());

	m_received_odom = true;
	this->processObservation(m_mrpt_odom);
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 41);
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniffCameraImage() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::sniff3DPointCloud() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::processObservation(
		mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	this->_process(observ);
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 42);
	this->resetReceivedFlags();
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 43);

}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::_process(
		mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 37);

	//this->m_logger->logFmt(LVL_DEBUG, "Calling execGraphSlamStep...");

	if (!this->m_engine->isPaused()) {
		this->m_engine->execGraphSlamStep(observ, m_measurement_cnt);
		m_measurement_cnt++;
	}
this->m_logger->logFmt(mrpt::utils::LVL_INFO, "TODO - Remove me. Kalimera %d", 38);
}

template<class GRAPH_T>
void CGraphSlamHandler_ROS<GRAPH_T>::resetReceivedFlags() {
	m_received_odom = false;
	m_received_laser_scan = false;
	m_received_camera = false;
	m_received_point_cloud = false;
}

} } } // end of namespaces

#endif /* end of include guard: CGRAPHSLAMHANDLER_ROS_IMPL_H */
