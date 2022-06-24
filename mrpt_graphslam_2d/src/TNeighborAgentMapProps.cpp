/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "mrpt_graphslam_2d/TNeighborAgentMapProps.h"

using namespace mrpt::system;
using namespace mrpt_msgs;
using namespace mrpt::maps;
using namespace mrpt::graphslam;
using namespace ros;
using namespace nav_msgs;
using namespace std;

TNeighborAgentMapProps::TNeighborAgentMapProps(
	mrpt::system::COutputLogger* logger_in,
	const mrpt_msgs::GraphSlamAgent& agent_in, ros::NodeHandle* nh_in)
	: m_logger(logger_in),
	  nh(nh_in),
	  agent(agent_in),
	  queue_size(1),
	  has_init_class(false),
	  has_setup_comm(false)
{
	ASSERT_(nh);
	ASSERT_(m_logger);
	m_logger->logFmt(LVL_WARN, "In TNeighborAgentMapProps constructor");

	this->map_topic =
		"/" + agent.topic_namespace.data + "/" + "feedback" + "/" + "gridmap";
	this->robot_trajectory_topic = "/" + agent.topic_namespace.data + "/" +
								   "feedback" + "/" + "robot_trajectory";

	cout << "Map topic: " << this->map_topic << endl;
	cout << "Trajectory topic: " << this->robot_trajectory_topic << endl;

	has_init_class = true;
	readROSParameters();
}

void TNeighborAgentMapProps::readROSParameters()
{
	// std::map<std::sttring, std::string> agent_pose;
	// bool res = nh->getParam("/" + agent.topic_namespace.data + "/" +
	// "global_pos", agent_pose);

	// if (res) {
	// m_logger->logFmt(LVL_INFO, "%s", getMapAsString(agent_pose).c_str());

	// global_init_pos = pose_t(
	// atof(pos_x),
	// atof(pos_y),
	// atof(rot_z));
	// m_logger->logFmt(LVL_INFO, "global_init_pos: %s\n", global_init_pos);
	//}
	// else {
	// cout << "parameter fetch was not successful" << endl;
	//}
}

void TNeighborAgentMapProps::setupComm()
{
	ASSERT_(has_init_class);
	m_logger->logFmt(LVL_WARN, "In TNeighborAgentMapProps::setupComm");
	this->setupSubs();
	has_setup_comm = true;
}

void TNeighborAgentMapProps::setupSubs()
{
	m_logger->logFmt(LVL_WARN, "In TNeighborAgentMapProps::setupSubs");
	this->map_sub = nh->subscribe<nav_msgs::OccupancyGrid>(
		this->map_topic, this->queue_size,
		&TNeighborAgentMapProps::updateGridMap, this);

	this->robot_trajectory_sub = nh->subscribe<nav_msgs::Path>(
		this->robot_trajectory_topic, this->queue_size,
		&TNeighborAgentMapProps::updateRobotTrajectory, this);
}  // end of setupSubs

void TNeighborAgentMapProps::updateGridMap(
	const nav_msgs::OccupancyGrid::ConstPtr& nav_gridmap)
{
	nav_map = nav_gridmap;
}  // end of updateGridMap

void TNeighborAgentMapProps::updateRobotTrajectory(
	const nav_msgs::Path::ConstPtr& nav_robot_traj)
{
	nav_robot_trajectory = nav_robot_traj;
}
