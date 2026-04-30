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

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <mrpt_msgs/msg/graph_slam_agent.hpp>
#include <mrpt_msgs/msg/graph_slam_agents.hpp>

#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/math/utils.h>

#include <algorithm>
#include <iterator>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

#include <cstdlib>

namespace mrpt
{
namespace graphslam
{
namespace detail
{
/**\brief Class responsible of handling the network communication between SLAM
 * agents in the Multi-Robot graphSLAM algorithm.
 *
 * ROS 2 port: fkie_multimaster discovery replaced by heartbeat topic
 * "/mrpt_graphslam/agent_heartbeat" (see docs/architecture/multirobot-ros2-design.md).
 */
class CConnectionManager
{
   public:
	typedef mrpt_msgs::msg::GraphSlamAgents::_list_type::iterator agents_it;
	typedef mrpt_msgs::msg::GraphSlamAgents::_list_type::const_iterator
		agents_cit;

	/**\brief Constructor */
	CConnectionManager(
		mrpt::system::COutputLogger* logger, rclcpp::Node* node);
	/**\brief Destructor */
	~CConnectionManager();
	/**\brief Fill the given vector with the SLAM Agents that the current
	 * manager can see and communicate with
	 *
	 * \param[in] ignore_self If true the GraphSlamAgent instance that is under
	 * the same  namespace as the CConnectionManager is not going to be inserted
	 * in the agents_vec
	 *
	 * \sa updateNearbySlamAgents
	 */
	void getNearbySlamAgents(
		mrpt_msgs::msg::GraphSlamAgents* agents_vec, bool ignore_self = true);
	/**\brief Read-only method for accessing list of nearby agents
	 */
	const mrpt_msgs::msg::GraphSlamAgents& getNearbySlamAgents();
	/**\brief Read-only method for accessing list of nearby agents.
	 * This <b>doesn't update</b> the internal list of GraphSlamAgents but just
	 * the returns its latest cached version
	 */
	const mrpt_msgs::msg::GraphSlamAgents& getNearbySlamAgentsCached() const;

	/**\brief Wrapper method around the private setup* class methods.
	 *
	 * Handy for setting up publishers, subscribers, services, TF-related stuff
	 * all at once from the user application
	 *
	 */
	void setupComm();
	/**\brief Get the agent ROS namespace */
	const std::string& getTrimmedNs() const;

   private:
	/**\brief Namespace under which we are running. Corresponds to the
	 * agent_ID_str with which the nodes are going to be registered in the graph
	 */
	std::string own_ns;
	/**\brief Update the internal list of nearby SLAM agents from cached data.
	 *
	 * \sa getNearbySlamAgents
	 */
	void updateNearbySlamAgents();
	/**\brief Heartbeat callback: called when another agent publishes its info */
	void onAgentHeartbeat(
		const mrpt_msgs::msg::GraphSlamAgent::SharedPtr agent_msg);
	/**\brief Prune agents that have not sent heartbeats recently */
	void pruneStaleAgents();
	/**\name setup* ROS-related methods
	 *\brief Methods for setting up topic subscribers, publishers, and
	 * corresponding services
	 *
	 * \sa setupComm
	 */
	/**\{*/
	void setupSubs();
	void setupPubs();
	void setupSrvs();
	/**\}*/

	/**\brief Pointer to the logging instance */
	mrpt::system::COutputLogger* m_logger;
	/**\brief Pointer to the ROS 2 node */
	rclcpp::Node* m_node;

	/**\brief Heartbeat publisher — publishes own agent info */
	rclcpp::Publisher<mrpt_msgs::msg::GraphSlamAgent>::SharedPtr m_agent_pub;
	/**\brief Heartbeat subscriber — receives other agents' info */
	rclcpp::Subscription<mrpt_msgs::msg::GraphSlamAgent>::SharedPtr m_agent_sub;
	/**\brief Timer for periodic heartbeat publishing */
	rclcpp::TimerBase::SharedPtr m_heartbeat_timer;
	/**\brief Timer for pruning stale agents */
	rclcpp::TimerBase::SharedPtr m_prune_timer;

	/**\brief Track last-seen time for each agent (by topic_namespace) */
	std::map<std::string, rclcpp::Time> m_agent_last_seen;
	/**\brief Mutex protecting m_nearby_slam_agents and m_agent_last_seen */
	mutable std::mutex m_agents_mutex;

	/**\brief List of slam agents in the current agent's neighborhood
	 *
	 * \note vector includes the GraphSlamAgent that is at the same namespace as
	 * the current CConnectionManager instance
	 */
	mrpt_msgs::msg::GraphSlamAgents m_nearby_slam_agents;

	bool has_setup_comm;
};

}  // namespace detail
}  // namespace graphslam
}  // namespace mrpt

/**\brief GraphSlamAgent ordering (for use in std::map/std::set).
 * ROS2 messages already provide operator== and operator!= as members.
 */
bool operator<(
	const mrpt_msgs::msg::GraphSlamAgent& agent1,
	const mrpt_msgs::msg::GraphSlamAgent& agent2);
