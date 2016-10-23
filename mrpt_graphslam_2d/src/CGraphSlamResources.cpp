/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "mrpt_graphslam_2d/CGraphSlamResources.h"

// supplementary functions - TODO - where to put these?
template<class T>
std::string getVectorAsString(const T& t) {
	using namespace std;
	stringstream ss;
	for (typename T::const_iterator it = t.begin(); it != t.end(); ++it) {
		ss << *it << ", ";
	}
	return ss.str();
}
template<class T>
void printVector(const T& t) {
	std::cout << getVectorAsString(t) << std::endl;
}
template<class T>
void printVectorOfVectors(const T& t) {
	int i = 0;
	for (typename T::const_iterator it = t.begin(); it  != t.end(); ++i, ++it) {
		printf("Vector %d/%lu:\n\t", i, t.size());
		printVector(*it);
	}
}


// static member variables
const std::string CGraphSlamResources::sep_header(40, '=');
const std::string CGraphSlamResources::sep_subheader(20, '-');

//////////////////////////////////////////////////////////////////////////////
// Ctor
CGraphSlamResources::CGraphSlamResources(
		mrpt::utils::COutputLogger* logger_in,
		ros::NodeHandle* nh_in):
	m_logger(logger_in),
	nh(nh_in)
{
	using namespace std;
	using namespace mrpt::obs;

	ASSERT_(m_logger);
	ASSERT_(nh_in);

	m_graphslam_handler = new CGraphSlamHandler();
	m_graphslam_handler->setOutputLoggerPtr(m_logger);

	// variables initialization/assignment
	m_has_read_config = false;
	m_pub_seq = 0;
	this->resetReceivedFlags();

	// measurements initialization
	// TODO - put them  in the corresponding function?
	m_mrpt_odom = CObservationOdometry::Create();
	m_mrpt_odom->hasEncodersInfo = false;
	m_mrpt_odom->hasVelocities = false;

	m_graphslam_engine = NULL;

	m_measurement_cnt = 0;
}
//////////////////////////////////////////////////////////////////////////////
CGraphSlamResources::~CGraphSlamResources() {
	using namespace mrpt::utils;


	// cleaning heap...
	m_logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamEngine instance...");
	delete m_graphslam_engine;
	m_logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamHandler instance...");
	delete m_graphslam_handler;

}
//////////////////////////////////////////////////////////////////////////////

void CGraphSlamResources::readParams() {
	this->readROSParameters();
	m_graphslam_handler->readConfigFname(m_ini_fname);


	m_has_read_config = true;
}
void CGraphSlamResources::readROSParameters() {
	using namespace mrpt::utils;

	// misc
	{
		std::string ns = "misc/";

		// enable/disable visuals
		nh->param<bool>(ns + "disable_MRPT_visuals", m_disable_MRPT_visuals, false);
		nh->param<bool>(ns + "disable_ROS_visuals", m_disable_ROS_visuals, false);

		// verbosity level
		int lvl;
		nh->param<int>(ns + "verbosity",
				lvl,
				static_cast<int>(LVL_INFO));
		m_min_logging_level = static_cast<VerbosityLevel>(lvl);
		this->m_logger->setMinLoggingLevel(m_min_logging_level);
	}
	// deciders, optimizer
	{
		std::string ns = "deciders_optimizers/";
		nh->param<std::string>(ns + "NRD", m_node_reg, "CFixedIntervalsNRD");
		nh->param<std::string>(ns + "ERD", m_edge_reg, "CICPCriteriaERD");
		nh->param<std::string>(ns + "GSO", m_optimizer, "CLevMarqGSO");
	}
	// filenames
	{
		std::string ns = "files/";

		// configuration file - mandatory
		std::string config_param_path = ns + "config";
		bool found_config = nh->getParam(ns + "config", m_ini_fname);
		ASSERTMSG_(found_config,
				mrpt::format("Configuration file was not set. Set %s and try again.\nExiting...",
					config_param_path.c_str()));

		// ground-truth file
		nh->getParam(ns + "ground_truth", m_gt_fname);
	}

	// TF Frame IDs
	// names of the frames of the corresponding robot parts
	{
		std::string ns = "frame_IDs/";

		nh->param<std::string>(ns + "global_frame"    , m_global_frame_id    , "map");
		nh->param<std::string>(ns + "base_link_frame" , m_base_link_frame_id , "base_link");
		nh->param<std::string>(ns + "odometry_frame"  , m_odom_frame_id      , "odom");
		nh->param<std::string>(ns + "laser_frame"     , m_laser_frame_id     , "laser");
	}

	// ASSERT that the given user options are valid
	this->verifyUserInput();

	m_logger->logFmt(LVL_DEBUG,
			"Successfully read parameters from ROS Parameter Server");

	this->initGraphSLAM();
}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::readStaticTFs() {
	using namespace mrpt::utils;

	// base_link => laser
	m_logger->logFmt(LVL_WARN, "Looking up static transform...%s => %s",
			m_laser_frame_id.c_str(),
			m_base_link_frame_id.c_str());
	try {
		m_listener.lookupTransform(
				/* target_fname = */ m_laser_frame_id,
				/* source_fname = */ m_base_link_frame_id,
				/* transform time = */ ros::Time(0),
				/* transform = */ m_base_laser_transform);
		m_logger->logFmt(LVL_INFO, "OK.");
	}
	catch (tf::TransformException &ex) {
		m_logger->logFmt(LVL_WARN, "Transformation not found, using default...");

  	m_base_laser_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  	m_base_laser_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  	m_base_laser_transform.stamp_ = ros::Time::now();
  	m_base_laser_transform.frame_id_ = m_base_link_frame_id;
  	m_base_laser_transform.child_frame_id_ = m_laser_frame_id;
	}


}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::initGraphSLAM() {
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
			m_graphslam_opts.node_regs_map[m_node_reg](),
			m_graphslam_opts.edge_regs_map[m_edge_reg](),
			m_graphslam_opts.optimizers_map[m_optimizer]());

	m_logger->logFmt(LVL_DEBUG, "Successfully initialized CGraphSlamEngine instance.");
}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::getROSParameters(std::string* str_out) {
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

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::getParamsAsString(std::string* str_out) {
	ASSERT_(str_out);

	// ros parameters
	std::string ros_params("");
	this->getROSParameters(&ros_params);
	*str_out += ros_params;

	// various parameters
}

//////////////////////////////////////////////////////////////////////////////
std::string CGraphSlamResources::getParamsAsString() {
	std::string params;
	this->getParamsAsString(&params);
	return params;
}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::printParams() {
	using namespace std;
	// print the problem parameters
	cout << this->getParamsAsString() << endl;

	m_graphslam_handler->printParams();
	m_graphslam_engine->printParams();


}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::verifyUserInput() {
	using namespace mrpt;
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_DEBUG, "Verifying user input...");


	// verify the NRD, ERD, GSO parameters
	bool node_success, edge_success, optimizer_success;
	bool failed = false;

	node_success = m_graphslam_opts.checkRegistrationDeciderExists(m_node_reg, "node");
	edge_success = m_graphslam_opts.checkRegistrationDeciderExists(m_edge_reg, "edge");
	optimizer_success = m_graphslam_opts.checkOptimizerExists(m_optimizer);

	if (!node_success) {
		m_logger->logFmt(LVL_ERROR,
				"\nNode Registration Decider \"%s\" is not available",
				m_node_reg.c_str());
		m_graphslam_opts.dumpRegistrarsToConsole("node");
		failed = true;
	}
	if (!edge_success) {
		m_logger->logFmt(LVL_ERROR,
				"\nEdge Registration Decider \"%s\" is not available.",
				m_edge_reg.c_str());
		m_graphslam_opts.dumpRegistrarsToConsole("edge");
		failed = true;
	}
	if (!optimizer_success) {
		m_logger->logFmt(LVL_ERROR,
				"\ngraphSLAM Optimizser \"%s\" is not available.",
				m_optimizer.c_str());
		m_graphslam_opts.dumpOptimizersToConsole();
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

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::setupCommunication() {

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	// fetch the static geometrical transformations
	this->readStaticTFs();

}

void CGraphSlamResources::setupSubs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the subscribers...");
	
	// setup the names
	std::string ns = "input/";

	m_odom_topic = ns + "odom";
	m_laser_scan_topic = ns + "laser_scan";

	// odometry
	m_odom_sub = nh->subscribe<mrpt_msgs::Pose2DStamped>(
			m_odom_topic,
			m_queue_size,
			&CGraphSlamResources::sniffOdom, this);

	// laser_scans
	m_laser_scan_sub = nh->subscribe<sensor_msgs::LaserScan>(
			m_laser_scan_topic,
			m_queue_size,
			&CGraphSlamResources::sniffLaserScan, this);
	
	// camera
	// TODO
	
	// 3D point clouds
	// TODO

}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::setupPubs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the publishers...");

	// setup the names
	std::string ns = "feedback/";

	m_curr_robot_pos_topic = ns + "robot_position";
	m_robot_trajectory_topic = ns + "robot_trajectory";
	m_robot_tr_poses_topic = ns + "robot_tr_poses";
	m_odom_tr_poses_topic = ns + "odom_tr_poses";
	m_SLAM_eval_metric_topic = ns + "evaluation_metric";
	m_gridmap_topic = ns + "gridmap";

	// agent estimated position
	m_curr_robot_pos_pub = nh->advertise<geometry_msgs::PoseStamped>(
			m_curr_robot_pos_topic,
			m_queue_size);
	 m_robot_trajectory_pub = nh->advertise<nav_msgs::Path>(
	 		 m_robot_trajectory_topic,
	 		 m_queue_size);
	 m_robot_tr_poses_pub = nh->advertise<geometry_msgs::PoseArray>(
	 		 m_robot_tr_poses_topic,
	 		 m_queue_size);

	 // odometry nav_msgs::Path
	 m_odom_path.header.seq = 0;
	 m_odom_path.header.stamp = ros::Time::now();
	 m_odom_path.header.frame_id = m_global_frame_id;

	 m_odom_tr_poses_pub = nh->advertise<nav_msgs::Path>(
	 		 m_odom_tr_poses_topic,
	 		 m_queue_size);

	 // generated gridmap
	 m_gridmap_pub = nh->advertise<nav_msgs::OccupancyGrid>(
	 		 m_gridmap_topic,
	 		 m_queue_size);


}

void CGraphSlamResources::setupSrvs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the services...");

	// SLAM statistics
	// Error statistics
	// Graph statistics

	// TODO - Implement this...
}

bool CGraphSlamResources::usePublishersBroadcasters() {
	using namespace mrpt::poses;
	using namespace mrpt::utils;
	using namespace std;


	ros::Time timestamp = ros::Time::now();

	// global frame <=> base_link
  {
		// current robot pose
		pose_t mrpt_pose = m_graphslam_engine->getCurrentRobotPosEstimation();

  	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(mrpt_pose.x(), mrpt_pose.y(), 0.0));
  	tf::Quaternion q;
  	q.setRPY(0, 0, mrpt_pose.phi());
  	transform.setRotation(q);

  	m_broadcaster.sendTransform(
  			tf::StampedTransform(transform, ros::Time::now(),
  				m_global_frame_id,
  				m_base_link_frame_id));
  }
  // global frame <=> odom frame
  // make sure that we have received odometry information in the first place...
  if (!m_global_odom_transform.frame_id_.empty()) {
  	m_broadcaster.sendTransform(m_global_odom_transform);
  }


  // set an arrow indicating clearly the current orientation of the robot
	{
		geometry_msgs::PoseStamped geom_pose;

		geom_pose.header.stamp = timestamp;
		geom_pose.header.seq = m_pub_seq;
		geom_pose.header.frame_id = m_base_link_frame_id; // with regards to base_link...

		geometry_msgs::Point point;
		point.x = 0; point.y = 0; point.z = 0;
		geometry_msgs::Quaternion quat;
		quat.x = 0; quat.y = 0; quat.z = 0; quat.w = 1;

		geom_pose.pose.position = point;
		geom_pose.pose.orientation = quat;
		m_curr_robot_pos_pub.publish(geom_pose);
	}

	// robot trajectory
	// publish the trajectory of the robot
	// TODO - have it cached until an LC is detected and reported
	{
		m_logger->logFmt(LVL_DEBUG, "Publishing the current robot trajectory");
		mrpt::graphs::CNetworkOfPoses2DInf::global_poses_t graph_poses;
		m_graphslam_engine->getRobotEstimatedTrajectory(&graph_poses);

		nav_msgs::Path path;

		// set the header
		path.header.stamp = timestamp;
		path.header.seq = m_pub_seq;
		path.header.frame_id = m_global_frame_id;

		//
		// fill in the pose as well as the nav_msgs::Path at the same time.
		//

		geometry_msgs::PoseArray geom_poses;
		geom_poses.header.stamp = timestamp;
		geom_poses.header.frame_id = m_global_frame_id;

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
			geom_pose_stamped.header.frame_id = m_global_frame_id;

			path.poses.push_back(geom_pose_stamped);
		}

		m_robot_tr_poses_pub.publish(geom_poses);
		m_robot_trajectory_pub.publish(path);
	}

	// SLAM Evaluation Metric
	{
	}

	// Odometry trajectory - nav_msgs::Path
	m_odom_tr_poses_pub.publish(m_odom_path);

	// generated gridmap
	{
		std_msgs::Header h;
		mrpt::system::TTimeStamp mrpt_time;
		mrpt::maps::COccupancyGridMap2D mrpt_gridmap;
		m_graphslam_engine->getOccupancyGridMap2D(&mrpt_gridmap, &mrpt_time);

		// timestamp
		mrpt_bridge::convert(mrpt_time, h.stamp);
		h.seq = m_pub_seq;
		h.frame_id = m_global_frame_id;

		// nav gridmap
		nav_msgs::OccupancyGrid nav_gridmap;
		mrpt_bridge::convert(mrpt_gridmap, nav_gridmap, h);
		m_gridmap_pub.publish(nav_gridmap);
	}

	m_pub_seq++;

	if (!m_disable_MRPT_visuals) {
		return this->continueExec();
	}
} // USEPUBLISHERSBROADCASTERS

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan) {
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

void CGraphSlamResources::sniffOdom(const mrpt_msgs::Pose2DStamped::ConstPtr& ros_odom) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::poses;

	m_logger->logFmt(LVL_DEBUG, "sniffOdom: Received an odometry msg. Converting it to MRPT format...");

	// update the odometry frame
  m_global_odom_transform.setOrigin(tf::Vector3(
  			ros_odom->pose.x,
  			ros_odom->pose.y,
  			0));
  tf::Quaternion quaternion;
  quaternion.setRPY(0, 0, ros_odom->pose.theta);
  quaternion.normalize();
  m_global_odom_transform.setRotation(quaternion);
  m_global_odom_transform.stamp_ = ros_odom->header.stamp;
  m_global_odom_transform.frame_id_ = m_global_frame_id;
  m_global_odom_transform.child_frame_id_ = m_odom_frame_id;
  
  // add to the overall odometry path
	geometry_msgs::PoseStamped geom_pose_stamped;
	{
		geom_pose_stamped.header.frame_id = m_global_frame_id;
		geom_pose_stamped.header.stamp = ros::Time::now();

		geometry_msgs::Point p;
		p.x = ros_odom->pose.x;
		p.y = ros_odom->pose.y;
		p.z = 0;
		geom_pose_stamped.pose.position = p;

		tf::quaternionTFToMsg(quaternion, geom_pose_stamped.pose.orientation);
	}
  m_odom_path.poses.push_back(geom_pose_stamped);

	// build and fill an CObservationOdometry instance
	mrpt_bridge::convert(
			/* src = */ ros_odom->header.stamp,
			/* dst = */m_mrpt_odom->timestamp);

	m_mrpt_odom->odometry.x(ros_odom->pose.x);
	m_mrpt_odom->odometry.y(ros_odom->pose.y);
	m_mrpt_odom->odometry.phi(ros_odom->pose.theta);

	// print the odometry -  for debugging reasons...
	stringstream ss;
	ss << "Odometry - MRPT format:\t" << m_mrpt_odom->odometry << endl;
	m_logger->logFmt(LVL_DEBUG, "%s", ss.str().c_str());

	m_received_odom = true;
	this->processObservation(m_mrpt_odom);

}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::sniffCameraImage() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::sniff3DPointCloud() {
	THROW_EXCEPTION("Method is not implemented yet.");

}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::processObservation(mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	this->_process(observ);
	this->resetReceivedFlags();

}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::generateReport() {
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
	if (m_graphslam_handler->save_gridmap) {
		COccupancyGridMap2D gridmap;
		m_graphslam_engine->getOccupancyGridMap2D(&gridmap);
		gridmap.saveMetricMapRepresentationToFile(
				m_graphslam_handler->output_dir_fname +
				"/" +
				m_graphslam_handler->save_gridmap_fname);
	}


}

bool CGraphSlamResources::continueExec() {
	using namespace std;
	using namespace mrpt::utils;

	m_logger->logFmt(LVL_DEBUG, "In continueExec check method");
	return m_graphslam_handler->queryObserverForEvents();
}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::_process(mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	m_logger->logFmt(LVL_DEBUG, "Calling execGraphSlamStep...");

	m_graphslam_engine->execGraphSlamStep(observ, m_measurement_cnt);
	m_measurement_cnt++;
}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::resetReceivedFlags() {
	m_received_odom = false;
	m_received_laser_scan = false;
	m_received_camera = false;
	m_received_point_cloud = false;
}


