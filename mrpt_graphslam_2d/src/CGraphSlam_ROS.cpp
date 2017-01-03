/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "mrpt_graphslam_2d/CGraphSlam_ROS.h"

// static member variables
const std::string CGraphSlam_ROS::sep_header(40, '=');
const std::string CGraphSlam_ROS::sep_subheader(20, '-');

// Ctor
CGraphSlam_ROS::CGraphSlam_ROS(
		mrpt::utils::COutputLogger* logger_in,
		ros::NodeHandle* nh_in):
	m_logger(logger_in),
	m_nh(nh_in)
{
	using namespace std;
	using namespace mrpt::obs;
	ASSERT_(m_logger);
	ASSERT_(m_nh);

	m_graphslam_handler = new CGraphSlamHandler();
	m_graphslam_handler->setOutputLoggerPtr(m_logger);

	// variables initialization/assignment
	m_has_read_config = false;
	m_pub_seq = 0;
	m_stats_pub_seq = 0;
	this->resetReceivedFlags();

	// measurements initialization
	m_mrpt_odom = CObservationOdometry::Create();
	m_mrpt_odom->hasEncodersInfo = false;
	m_mrpt_odom->hasVelocities = false;

	m_graphslam_engine = NULL;

	m_measurement_cnt = 0;

	// Thu Nov 3 23:36:49 EET 2016, Nikos Koukis
	// WARNING: ROS Server Parameters have not been read yet. Make sure you know
	// what to initialize at this stage!
}
CGraphSlam_ROS::~CGraphSlam_ROS() {
	using namespace mrpt::utils;


	// cleaning heap...
	m_logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamEngine instance...");
	delete m_graphslam_engine;
	m_logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamHandler instance...");
	delete m_graphslam_handler;

}

void CGraphSlam_ROS::readParams() {
	this->readROSParameters();
	m_graphslam_handler->readConfigFname(m_ini_fname);


	m_has_read_config = true;
}
void CGraphSlam_ROS::readROSParameters() {
	using namespace mrpt::utils;

	// misc
	{
		std::string ns = "misc/";

		// enable/disable visuals
		m_nh->param<bool>(ns + "disable_MRPT_visuals", m_disable_MRPT_visuals, false);

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
		bool found_config = m_nh->getParam(ns + "config", m_ini_fname);
		std::cout << "KALIMERA : " << m_ini_fname << std::endl;
		ASSERTMSG_(found_config,
				mrpt::format("Configuration file was not set. Set %s and try again.\nExiting...",
					config_param_path.c_str()));

		// ground-truth file
		m_nh->getParam(ns + "ground_truth", m_gt_fname);
	}

	// TF Frame IDs
	// names of the frames of the corresponding robot parts
	{
		std::string ns = "frame_IDs/";

		m_nh->param<std::string>(ns + "anchor_frame"    , m_anchor_frame_id    , "map");
		m_nh->param<std::string>(ns + "base_link_frame" , m_base_link_frame_id , "base_link");
		m_nh->param<std::string>(ns + "odometry_frame"  , m_odom_frame_id      , "odom");
		m_nh->param<std::string>(ns + "laser_frame"     , m_laser_frame_id     , "laser");

		// take action based on the above frames
		//

	}

	// ASSERT that the given user options are valid
	// Fill the TuserOptionsChecker related structures
	m_options_checker.createDeciderOptimizerMappings();
	m_options_checker.populateDeciderOptimizerProperties();
	this->verifyUserInput();

	m_logger->logFmt(LVL_DEBUG,
			"Successfully read parameters from ROS Parameter Server");

	this->initGraphSLAM();
}
void CGraphSlam_ROS::readStaticTFs() {
	using namespace mrpt::utils;

	// base_link => laser
	m_logger->logFmt(LVL_WARN, "Looking up static transform...%s => %s",
			m_laser_frame_id.c_str(),
			m_base_link_frame_id.c_str());
	try {
		m_base_laser_transform = m_buffer.lookupTransform(
				/* target_fname = */ m_laser_frame_id,
				/* source_fname = */ m_base_link_frame_id,
				/* transform time = */ ros::Time(0));
		m_logger->logFmt(LVL_INFO, "OK.");
	}
	catch (tf2::LookupException &ex) {
		m_logger->logFmt(LVL_WARN, "Transformation not found, using default...");

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
void CGraphSlam_ROS::initGraphSLAM() {
	using namespace mrpt::graphs;
	using namespace mrpt::graphslam;
	using namespace mrpt::utils;
	
	m_logger->logFmt(LVL_DEBUG, "Initializing CGraphSlamEngine instance...");


	// TODO - quickfix for calling CGraphSlamEngine...
	std::string rawlog_fname("");
	m_graphslam_handler->setRawlogFname(rawlog_fname);

	// Visuals initialization
	if (!m_disable_MRPT_visuals) {
		m_graphslam_handler->initVisualization();
	}

	m_graphslam_engine = new CGraphSlamEngine<CNetworkOfPoses2DInf>(
			m_ini_fname,
			rawlog_fname,
			m_gt_fname,
			m_graphslam_handler->win_manager,
			m_options_checker.node_regs_map[m_node_reg](),
			m_options_checker.edge_regs_map[m_edge_reg](),
			m_options_checker.optimizers_map[m_optimizer]());

	m_logger->logFmt(LVL_DEBUG, "Successfully initialized CGraphSlamEngine instance.");
}
void CGraphSlam_ROS::getROSParameters(std::string* str_out) {
	using namespace std;
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
	ss << "Configuration .ini file   = " << m_ini_fname << endl;
	ss << "Ground truth filename     = " << (!m_gt_fname.empty() ? m_gt_fname : "NONE")
		<< endl;
	ss << endl;

	ss << "Miscellaneous: " << endl;
	ss << sep_subheader << endl;
	ss << "Enable MRPT visuals?      = " << (m_disable_MRPT_visuals? "FALSE" : "TRUE")
		<< endl;
	ss << "Logging verbosity Level   = " << 
		COutputLogger::logging_levels_to_names[m_min_logging_level] << endl;;
	ss << endl;

	*str_out = ss.str();
}

void CGraphSlam_ROS::getParamsAsString(std::string* str_out) {
	ASSERT_(str_out);

	// ros parameters
	std::string ros_params("");
	this->getROSParameters(&ros_params);
	*str_out += ros_params;

	// various parameters
}

std::string CGraphSlam_ROS::getParamsAsString() {
	std::string params;
	this->getParamsAsString(&params);
	return params;
}

void CGraphSlam_ROS::printParams() {
	using namespace std;
	// print the problem parameters
	cout << this->getParamsAsString() << endl;

	m_graphslam_handler->printParams();
	m_graphslam_engine->printParams();


}
void CGraphSlam_ROS::verifyUserInput() {
	using namespace mrpt;
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_DEBUG, "Verifying user input...");


	// verify the NRD, ERD, GSO parameters
	bool node_success, edge_success, optimizer_success;
	bool failed = false;

	node_success = m_options_checker.checkRegistrationDeciderExists(m_node_reg, "node");
	edge_success = m_options_checker.checkRegistrationDeciderExists(m_edge_reg, "edge");
	optimizer_success = m_options_checker.checkOptimizerExists(m_optimizer);

	if (!node_success) {
		m_logger->logFmt(LVL_ERROR,
				"\nNode Registration Decider \"%s\" is not available",
				m_node_reg.c_str());
		m_options_checker.dumpRegistrarsToConsole("node");
		failed = true;
	}
	if (!edge_success) {
		m_logger->logFmt(LVL_ERROR,
				"\nEdge Registration Decider \"%s\" is not available.",
				m_edge_reg.c_str());
		m_options_checker.dumpRegistrarsToConsole("edge");
		failed = true;
	}
	if (!optimizer_success) {
		m_logger->logFmt(LVL_ERROR,
				"\ngraphSLAM Optimizser \"%s\" is not available.",
				m_optimizer.c_str());
		m_options_checker.dumpOptimizersToConsole();
		failed = true;
	}
	ASSERT_(!failed)

	// verify that the path to the files is correct
	// .ini file
	bool ini_exists = system::fileExists(m_ini_fname);
	ASSERTMSG_(ini_exists,
			format(
				"\n.ini configuration file \"%s\"doesn't exist. Please specify a valid pathname.\nExiting...\n",
				m_ini_fname.c_str()));
	// ground-truth file
	if (!m_gt_fname.empty()) {
		bool gt_exists = system::fileExists(m_gt_fname);
		ASSERTMSG_(gt_exists,
				format(
					"\nGround truth file \"%s\"doesn't exist. Either unset the corresponding ROS parameter or specify a valid pathname.\nExiting...\n",
					m_gt_fname.c_str()));
	}

}

void CGraphSlam_ROS::setupComm() {
	using namespace mrpt::utils;

	m_logger->logFmt(LVL_INFO,
			"Setting up ROS-related subscribers, publishers, services...");

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	// fetch the static geometrical transformations
	this->readStaticTFs();

}

void CGraphSlam_ROS::setupSubs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the subscribers...");
	
	// setup the names
	std::string ns = "input/";

	m_odom_topic = ns + "odom";
	m_laser_scan_topic = ns + "laser_scan";

	// odometry
	m_odom_sub = m_nh->subscribe<nav_msgs::Odometry>(
			m_odom_topic,
			m_queue_size,
			&CGraphSlam_ROS::sniffOdom, this);

	// laser_scans
	m_laser_scan_sub = m_nh->subscribe<sensor_msgs::LaserScan>(
			m_laser_scan_topic,
			m_queue_size,
			&CGraphSlam_ROS::sniffLaserScan, this);
	
	// camera
	// TODO
	
	// 3D point clouds
	// TODO

}
void CGraphSlam_ROS::setupPubs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the publishers...");

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

void CGraphSlam_ROS::setupSrvs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the services...");

	// SLAM statistics
	// Error statistics
	// Graph statistics

	// TODO - Implement this...
}

bool CGraphSlam_ROS::usePublishersBroadcasters() {
	using namespace mrpt::poses;
	using namespace mrpt::utils;
	using namespace std;


	ros::Time timestamp = ros::Time::now();

	// current MRPT robot pose
	pose_t mrpt_pose = m_graphslam_engine->getCurrentRobotPosEstimation();

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

  // anchor frame <=> odom frame
  //
  // make sure that we have received odometry information in the first place...
  // the corresponding field would be initialized
  if (!m_anchor_odom_transform.child_frame_id.empty()) {
  	m_broadcaster.sendTransform(m_anchor_odom_transform);
  }


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

	// robot trajectory
	// publish the trajectory of the robot
	{
		m_logger->logFmt(LVL_DEBUG, "Publishing the current robot trajectory");
		mrpt::graphs::CNetworkOfPoses2DInf::global_poses_t graph_poses;
		m_graphslam_engine->getRobotEstimatedTrajectory(&graph_poses);

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

	// generated gridmap
	{
		std_msgs::Header h;
		mrpt::system::TTimeStamp mrpt_time;
		mrpt::maps::COccupancyGridMap2DPtr mrpt_gridmap = mrpt::maps::COccupancyGridMap2D::Create();
		m_graphslam_engine->getMap(mrpt_gridmap, &mrpt_time);

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

		bool ret = m_graphslam_engine->getGraphSlamStats(&node_stats,
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
			m_graphslam_engine->getDeformationEnergyVector(&stats.slam_evaluation_metric);

			mrpt_bridge::convert(mrpt_time, stats.header.stamp);

			m_stats_pub.publish(stats);
		}
		else {
			m_logger->logFmt(LVL_ERROR, "Answer is False");
		}
		
		

	}


	m_pub_seq++;

	if (!m_disable_MRPT_visuals) {
		return this->continueExec();
	}
} // USEPUBLISHERSBROADCASTERS

void CGraphSlam_ROS::sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;

	m_logger->logFmt(LVL_DEBUG, "sniffLaserScan: Received a LaserScan msg. Converting it to MRPT format...");

	// build the CObservation2DRangeScan
	m_mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;
	mrpt_bridge::convert(*ros_laser_scan, rel_pose, *m_mrpt_laser_scan);

	m_received_laser_scan = true;
	this->processObservation(m_mrpt_laser_scan);
}

void CGraphSlam_ROS::sniffOdom(const nav_msgs::Odometry::ConstPtr& ros_odom) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::poses;

	m_logger->logFmt(LVL_DEBUG, "sniffOdom: Received an odometry msg. Converting it to MRPT format...");

	// update the odometry frame with regards to the anchor
	{
		// header
		m_anchor_odom_transform.header.frame_id = m_anchor_frame_id;
		m_anchor_odom_transform.header.stamp = ros_odom->header.stamp;
		m_anchor_odom_transform.header.seq = ros_odom->header.seq;

		m_anchor_odom_transform.child_frame_id = m_odom_frame_id;

		// translation
		m_anchor_odom_transform.transform.translation.x = ros_odom->pose.pose.position.x;
		m_anchor_odom_transform.transform.translation.y = ros_odom->pose.pose.position.y;
		m_anchor_odom_transform.transform.translation.z = ros_odom->pose.pose.position.z;

		// quaternion
		m_anchor_odom_transform.transform.rotation.x = ros_odom->pose.pose.orientation.x;
		m_anchor_odom_transform.transform.rotation.y = ros_odom->pose.pose.orientation.y;
		m_anchor_odom_transform.transform.rotation.z = ros_odom->pose.pose.orientation.z;
		m_anchor_odom_transform.transform.rotation.w = ros_odom->pose.pose.orientation.w;
	}

  // add to the overall odometry path
  {
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header = ros_odom->header;
		pose_stamped.pose = ros_odom->pose.pose;
  	m_odom_path.poses.push_back(pose_stamped);
  }

	// build and fill an MRPT CObservationOdometry instance for manipulation from
	// the main algorithm
	mrpt_bridge::convert(
			/* src = */ ros_odom->header.stamp,
			/* dst = */ m_mrpt_odom->timestamp);
	mrpt_bridge::convert(
			/* src = */ ros_odom->pose.pose,
			/* dst = */ m_mrpt_odom->odometry);

	// print the odometry -  for debugging reasons...
	stringstream ss;
	ss << "Odometry - MRPT format:\t" << m_mrpt_odom->odometry << endl;
	m_logger->logFmt(LVL_DEBUG, "%s", ss.str().c_str());

	m_received_odom = true;
	this->processObservation(m_mrpt_odom);
}

void CGraphSlam_ROS::sniffCameraImage() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
void CGraphSlam_ROS::sniff3DPointCloud() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
void CGraphSlam_ROS::processObservation(mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	this->_process(observ);
	this->resetReceivedFlags();

}
void CGraphSlam_ROS::generateReport() {
	using namespace std;
	using namespace mrpt::utils;

	m_logger->logFmt(LVL_INFO, "Generating overall report...");
	m_graphslam_engine->generateReportFiles(m_graphslam_handler->output_dir_fname);
	// save the graph and the 3DScene 
	if (m_graphslam_handler->save_graph) {
		m_logger->logFmt(LVL_INFO, "Saving the graph...");
		std::string save_graph_fname = 
			m_graphslam_handler->output_dir_fname +
			"/" +
			m_graphslam_handler->save_graph_fname;
		m_graphslam_engine->saveGraph(&save_graph_fname);
	}
	if (!m_disable_MRPT_visuals && m_graphslam_handler->save_3DScene) {
	m_logger->logFmt(LVL_INFO, "Saving the 3DScene object...");
		std::string save_3DScene_fname = 
			m_graphslam_handler->output_dir_fname +
			"/" +
			m_graphslam_handler->save_3DScene_fname;

		m_graphslam_engine->save3DScene(&save_3DScene_fname);
	}
	// get the occupancy gridmap that was built
	if (m_graphslam_handler->save_map) {
		COccupancyGridMap2DPtr gridmap;
		m_graphslam_engine->getMap(gridmap);
		gridmap->saveMetricMapRepresentationToFile(
				m_graphslam_handler->output_dir_fname +
				"/" +
				m_graphslam_handler->save_map_fname);
	}


}

bool CGraphSlam_ROS::continueExec() {
	using namespace std;
	using namespace mrpt::utils;

	m_logger->logFmt(LVL_DEBUG, "In continueExec check method");
	return m_graphslam_handler->queryObserverForEvents();
}

void CGraphSlam_ROS::_process(mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	m_logger->logFmt(LVL_DEBUG, "Calling execGraphSlamStep...");

	m_graphslam_engine->execGraphSlamStep(observ, m_measurement_cnt);
	m_measurement_cnt++;
}

void CGraphSlam_ROS::resetReceivedFlags() {
	m_received_odom = false;
	m_received_laser_scan = false;
	m_received_camera = false;
	m_received_point_cloud = false;
}


