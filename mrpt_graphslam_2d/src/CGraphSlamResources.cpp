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
	logger(logger_in),
	nh(nh_in)
{
	using namespace std;
	using namespace mrpt::obs;

	ASSERT_(logger);
	ASSERT_(nh_in);

	// setup subscribers, publishers, services...
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	graphslam_handler = new CGraphSlamHandler();
	graphslam_handler->setOutputLoggerPtr(logger);

	// variables initialization/assignment
	has_read_config = false;
	this->resetReceivedFlags();

	// measurements initialization


	mrpt_odom = CObservationOdometry::Create();
	mrpt_odom->hasEncodersInfo = false;
	mrpt_odom->hasVelocities = false;

	graphslam_engine = NULL;

	measurement_cnt = 0;
}
//////////////////////////////////////////////////////////////////////////////
CGraphSlamResources::~CGraphSlamResources() {
	using namespace mrpt::utils;


	// cleaning heap...
	logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamEngine instance...");
	delete graphslam_engine;
	logger->logFmt(LVL_DEBUG, "Releasing CGraphSlamHandler instance...");
	delete graphslam_handler;

}
//////////////////////////////////////////////////////////////////////////////

void CGraphSlamResources::readParams() {
	this->readROSParameters();
	graphslam_handler->readConfigFname(ini_fname);

	has_read_config = true;
}
void CGraphSlamResources::readROSParameters() {
	using namespace mrpt::utils;

	// misc
	{
		std::string ns = "misc/";

		// enable/disable visuals
		nh->param<bool>(ns + "disable_visuals", disable_visuals, false);

		// verbosity level
		int lvl;
		nh->param<int>(ns + "verbosity",
				lvl,
				static_cast<int>(LVL_INFO));
		min_logging_level = static_cast<VerbosityLevel>(lvl);
		this->logger->setMinLoggingLevel(min_logging_level);
	}
	// deciders, optimizer
	{
		std::string ns = "deciders_optimizers/";
		nh->param<std::string>(ns + "NRD", node_reg, "CFixedIntervalsNRD");
		nh->param<std::string>(ns + "ERD", edge_reg, "CICPCriteriaERD");
		nh->param<std::string>(ns + "GSO", optimizer, "CLevMarqGSO");
	}
	// filenames
	{
		std::string ns = "files/";

		// configuration file - mandatory
		std::string config_param_path = ns + "config";
		bool found_config = nh->getParam(ns + "config", ini_fname);
		ASSERTMSG_(found_config,
				mrpt::format("Configuration file was not set. Set %s and try again.\nExiting...",
					config_param_path.c_str()));

		// ground-truth file
		nh->getParam(ns + "ground_truth", gt_fname);
	}

	// ASSERT that the given user options are valid
	this->verifyUserInput();

	logger->logFmt(LVL_DEBUG,
			"Successfully read parameters from ROS Parameter Server");

	this->initGraphSLAM();
}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::initGraphSLAM() {
	using namespace mrpt::graphs;
	using namespace mrpt::graphslam;
	using namespace mrpt::utils;
	
	logger->logFmt(LVL_DEBUG, "Initializing CGraphSlamEngine instance...");


	// TODO - quickfix for calling CGraphSlamEngine...
	std::string rawlog_fname("");
	graphslam_handler->setRawlogFname(rawlog_fname);

	// Visuals initialization
	if (!disable_visuals) {
		graphslam_handler->initVisualization();
	}

	graphslam_engine = new CGraphSlamEngine<CNetworkOfPoses2DInf>(
			ini_fname,
			rawlog_fname,
			gt_fname,
			graphslam_handler->win_manager,
			graphslam_opts.node_regs_map[node_reg](),
			graphslam_opts.edge_regs_map[edge_reg](),
			graphslam_opts.optimizers_map[optimizer]());

	logger->logFmt(LVL_DEBUG, "Successfully initialized CGraphSlamEngine instance.");
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
	ss << "Node Registration Decider : " << node_reg << endl;
	ss << "Edge Registration Decider : " << edge_reg << endl;
	ss << "GraphSLAM Optimizer       : " << optimizer << endl;
	ss << endl;

	ss << "Filenames: " << endl;
	ss << sep_subheader << endl;
	ss << "Configuration .ini file   : " << ini_fname << endl;
	ss << "Ground truth filename     : " << (!gt_fname.empty() ? gt_fname : "NONE")
		<< endl;
	ss << endl;

	ss << "Miscellaneous: " << endl;
	ss << sep_subheader << endl;
	ss << "Enable visuals?           : " << (!disable_visuals? "TRUE" : "FALSE")
		<< endl;
	ss << "Logging verbosity Level   : " << 
		COutputLogger::logging_levels_to_names[min_logging_level] << endl;;
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

	graphslam_handler->printParams();
	graphslam_engine->printParams();


}
//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::verifyUserInput() {
	using namespace mrpt;
	using namespace mrpt::utils;
	logger->logFmt(LVL_DEBUG, "Verifying user input...");


	// verify the NRD, ERD, GSO parameters
	bool node_success, edge_success, optimizer_success;
	bool failed = false;

	node_success = graphslam_opts.checkRegistrationDeciderExists(node_reg, "node");
	edge_success = graphslam_opts.checkRegistrationDeciderExists(edge_reg, "edge");
	optimizer_success = graphslam_opts.checkOptimizerExists(optimizer);

	if (!node_success) {
		logger->logFmt(LVL_ERROR,
				"\nNode Registration Decider \"%s\" is not available",
				node_reg.c_str());
		graphslam_opts.dumpRegistrarsToConsole("node");
		failed = true;
	}
	if (!edge_success) {
		logger->logFmt(LVL_ERROR,
				"\nEdge Registration Decider \"%s\" is not available.",
				edge_reg.c_str());
		graphslam_opts.dumpRegistrarsToConsole("edge");
		failed = true;
	}
	if (!optimizer_success) {
		logger->logFmt(LVL_ERROR,
				"\ngraphSLAM Optimizser \"%s\" is not available.",
				optimizer.c_str());
		graphslam_opts.dumpOptimizersToConsole();
		failed = true;
	}
	ASSERT_(!failed)

	// verify that the path to the files is correct
	// .ini file
	bool ini_exists = system::fileExists(ini_fname);
	ASSERTMSG_(ini_exists,
			format(
				"\n.ini configuration file \"%s\"doesn't exist. Please specify a valid pathname.\nExiting...\n",
				ini_fname.c_str()));
	// ground-truth file
	if (!gt_fname.empty()) {
		bool gt_exists = system::fileExists(gt_fname);
		ASSERTMSG_(gt_exists,
				format(
					"\n.Ground truth file \"%s\"doesn't exist. Either unset the corresponding ROS parameter or specify a valid pathname.\nExiting...\n",
					gt_fname.c_str()));
	}

}

//////////////////////////////////////////////////////////////////////////////

void CGraphSlamResources::setupSubs() {
	using namespace mrpt::utils;
	logger->logFmt(LVL_INFO, "Setting up the subscribers...");
	
	// setup the names
	std::string ns = "input/";

	odom_topic = ns + "odom";
	laser_scan_topic = ns + "laser_scan";

	// odometry
	odom_sub = nh->subscribe<mrpt_msgs::Pose2DStamped>(
			odom_topic,
			1000,
			&CGraphSlamResources::sniffOdom, this);

	// laser_scans
	laser_scan_sub = nh->subscribe<sensor_msgs::LaserScan>(
			laser_scan_topic,
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
	logger->logFmt(LVL_INFO, "Setting up the publishers...");

	std::string ns = "feedback/";
	// TODO - Implement this...

	// agent estimated position

}

void CGraphSlamResources::setupSrvs() {
	using namespace mrpt::utils;
	logger->logFmt(LVL_INFO, "Setting up the services...");

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

	logger->logFmt(LVL_DEBUG, "sniffLaserScan: Received a LaserScan msg. Converting it to MRPT format...");

	// build the CObservation2DRangeScan
	mrpt_laser_scan = CObservation2DRangeScan::Create();
	mrpt::poses::CPose3D rel_pose;
	mrpt_bridge::convert(*ros_laser_scan, rel_pose, *mrpt_laser_scan);

	received_laser_scan = true;
	this->processObservation(mrpt_laser_scan);
}

void CGraphSlamResources::sniffOdom(const mrpt_msgs::Pose2DStamped::ConstPtr& ros_odom) {
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::obs;
	using namespace mrpt::poses;

	logger->logFmt(LVL_DEBUG, "sniffOdom: Received an odometry msg. Converting it to MRPT format...");

	// build and fill an CObservationOdometry instance

	mrpt_bridge::convert(
			/* src = */ ros_odom->header.stamp,
			/* dst = */mrpt_odom->timestamp);

	mrpt_odom->odometry.x(ros_odom->pose.x);
	mrpt_odom->odometry.y(ros_odom->pose.y);
	mrpt_odom->odometry.phi(ros_odom->pose.theta);

	// print the odometry -  for debugging reasons...
	stringstream ss;
	ss << "Odometry - MRPT format:\t" << mrpt_odom->odometry << endl;
	logger->logFmt(LVL_DEBUG, "%s", ss.str().c_str());

	received_odom = true;
	this->processObservation(mrpt_odom);


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
void CGraphSlamResources::_process(mrpt::obs::CObservationPtr& observ) {
	using namespace mrpt::utils;
	using namespace std;

	logger->logFmt(LVL_DEBUG, "Calling execGraphSlamStep...");

	graphslam_engine->execGraphSlamStep(observ, measurement_cnt);
	measurement_cnt++;
}

//////////////////////////////////////////////////////////////////////////////
void CGraphSlamResources::resetReceivedFlags() {
	received_odom = false;
	received_laser_scan = false;
	received_camera = false;
	received_point_cloud = false;
}


