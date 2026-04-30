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
using namespace mrpt::maps;
using namespace mrpt::graphslam;
using namespace std;

TNeighborAgentMapProps::TNeighborAgentMapProps(
	mrpt::system::COutputLogger* logger_in,
	const mrpt_msgs::msg::GraphSlamAgent& agent_in,
	rclcpp::Node* node_in)
	: m_logger(logger_in),
	  nh(node_in),
	  agent(agent_in),
	  queue_size(1),
	  has_init_class(false),
	  has_setup_comm(false)
{
	ASSERT_(nh);
	ASSERT_(m_logger);
	m_logger->logFmt(LVL_WARN, "In TNeighborAgentMapProps constructor");

	this->map_topic =
		"/" + agent.topic_namespace.data + "/feedback/gridmap";
	this->robot_trajectory_topic =
		"/" + agent.topic_namespace.data + "/feedback/robot_trajectory";

	cout << "Map topic: " << this->map_topic << endl;
	cout << "Trajectory topic: " << this->robot_trajectory_topic << endl;

	has_init_class = true;
	readROSParameters();
}

void TNeighborAgentMapProps::readROSParameters() {}

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
	map_sub = nh->create_subscription<nav_msgs::msg::OccupancyGrid>(
		this->map_topic, this->queue_size,
		[this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
			this->updateGridMap(msg);
		});
	robot_trajectory_sub = nh->create_subscription<nav_msgs::msg::Path>(
		this->robot_trajectory_topic, this->queue_size,
		[this](nav_msgs::msg::Path::SharedPtr msg) {
			this->updateRobotTrajectory(msg);
		});
}  // end of setupSubs

void TNeighborAgentMapProps::updateGridMap(
	nav_msgs::msg::OccupancyGrid::SharedPtr nav_gridmap)
{
	nav_map = nav_gridmap;
}  // end of updateGridMap

void TNeighborAgentMapProps::updateRobotTrajectory(
	nav_msgs::msg::Path::SharedPtr nav_robot_traj)
{
	nav_robot_trajectory = nav_robot_traj;
}
