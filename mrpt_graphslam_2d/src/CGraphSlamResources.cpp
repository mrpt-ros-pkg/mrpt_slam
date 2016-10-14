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

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	m_graphslam_handler = new CGraphSlamHandler();
	m_graphslam_handler->setOutputLoggerPtr(m_logger);

	// variables initialization/assignment
	m_has_read_config = false;
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
		nh->param<bool>(ns + "disable_visuals", m_disable_visuals, false);

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

	// ASSERT that the given user options are valid
	this->verifyUserInput();

	m_logger->logFmt(LVL_DEBUG,
			"Successfully read parameters from ROS Parameter Server");

	this->initGraphSLAM();
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
	if (!m_disable_visuals) {
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

	ss << "Deciders / Optimizers: " << endl;
	ss << sep_subheader << endl;
	ss << "Node Registration Decider : " << m_node_reg << endl;
	ss << "Edge Registration Decider : " << m_edge_reg << endl;
	ss << "GraphSLAM Optimizer       : " << m_optimizer << endl;
	ss << endl;

	ss << "Filenames: " << endl;
	ss << sep_subheader << endl;
	ss << "Configuration .ini file   : " << m_ini_fname << endl;
	ss << "Ground truth filename     : " << (!m_gt_fname.empty() ? m_gt_fname : "NONE")
		<< endl;
	ss << endl;

	ss << "Miscellaneous: " << endl;
	ss << sep_subheader << endl;
	ss << "Enable visuals?           : " << (!m_disable_visuals? "TRUE" : "FALSE")
		<< endl;
	ss << "Logging verbosity Level   : " << 
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
					"\n.Ground truth file \"%s\"doesn't exist. Either unset the corresponding ROS parameter or specify a valid pathname.\nExiting...\n",
					m_gt_fname.c_str()));
	}

}

//////////////////////////////////////////////////////////////////////////////

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
			1000,
			&CGraphSlamResources::sniffOdom, this);

	// laser_scans
	m_laser_scan_sub = nh->subscribe<sensor_msgs::LaserScan>(
			m_laser_scan_topic,
			1000,
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

	std::string ns = "feedback/";
	// TODO - Implement this...

	// agent estimated position

}

void CGraphSlamResources::setupSrvs() {
	using namespace mrpt::utils;
	m_logger->logFmt(LVL_INFO, "Setting up the services...");

	// SLAM statistics
	// Error statistics
	// Graph statistics

	// TODO - Implement this...
}
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
	if (!m_disable_visuals && m_graphslam_handler->save_3DScene) {
	m_logger->logFmt(LVL_INFO, "Saving the 3DScene object...");
		std::string save_3DScene_fname = 
			m_graphslam_handler->output_dir_fname +
			"/" +
			m_graphslam_handler->save_3DScene_fname;

		m_graphslam_engine->save3DScene(&save_3DScene_fname);
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


