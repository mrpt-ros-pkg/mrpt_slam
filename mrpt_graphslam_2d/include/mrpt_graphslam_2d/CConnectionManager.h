/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) | |
   http://www.mrpt.org/                             | | | | Copyright (c)
   2005-2016, Individual contributors, see AUTHORS file        | | See:
   http://www.mrpt.org/Authors - All rights reserved.                   | |
   Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+
 */

#pragma once

#include <ros/ros.h>
#include <fkie_multimaster_msgs/DiscoverMasters.h>
#include <mrpt_msgs/GraphSlamAgent.h>
#include <mrpt_msgs/GraphSlamAgents.h>

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

#include <cstdlib>

namespace mrpt
{
namespace graphslam
{
namespace detail
{
/**\brief Class responsible of handling the network communication between SLAM
 * agents in the Multi-Robot graphSLAM algorithm.
 */
class CConnectionManager
{
   public:
	// typedef std::vector<mrpt_msgs::GraphSlamAgent>::iterator agents_it;
	// typedef std::vector<mrpt_msgs::GraphSlamAgent>::const_iterator
	// agents_cit;
	typedef mrpt_msgs::GraphSlamAgents::_list_type::iterator agents_it;
	typedef mrpt_msgs::GraphSlamAgents::_list_type::const_iterator agents_cit;

	/**\brief Constructor */
	CConnectionManager(
		mrpt::system::COutputLogger* logger, ros::NodeHandle* nh_in);
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
		mrpt_msgs::GraphSlamAgents* agents_vec, bool ignore_self = true);
	/**\brief Read-only method for accessing list of nearby agents
	 */
	const mrpt_msgs::GraphSlamAgents& getNearbySlamAgents();
	/**\brief Read-only method for accessing list of nearby agents.
	 * This <b>doesn't update</b> the internal list of GraphSlamAgents but just
	 * the returns its latest cached version
	 */
	const mrpt_msgs::GraphSlamAgents& getNearbySlamAgentsCached() const;

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
	/**\brief Update the internal list of nearby SLAM agents
	 *
	 * \sa getNearbySlamAgents
	 */
	void updateNearbySlamAgents();
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

	/**\brief ROSMaster ==> mrpt_msgs::GraphSlamAgent
	 *
	 * Assumption is that each ROSMaster instance holds exactly one
	 * mrpt_graphslam_2d node which publshes at a specific toic namespace ->
	 * /<hostname>_<last_IP_field>/...
	 *
	 * \return False if the ros_master specified doesn't correspond to a valid
	 * GraphSlamAgent node. Should at least have a \b feedback topic
	 * namespace under its main topic namespace
	 */
	static bool convert(
		const fkie_multimaster_msgs::ROSMaster& ros_master,
		mrpt_msgs::GraphSlamAgent* slam_agent);
	/**\brief GraphSlamAgent ==> ROSMaster. */
	static void convert(
		const mrpt_msgs::GraphSlamAgent& slam_agent,
		fkie_multimaster_msgs::ROSMaster* ros_master);
	/**\brief Remove http:// prefix and port suffix from the string and return
	 * result
	 *
	 * \param[out] agent_port Port that the agent is running on. Extracted from
	 * the overall string
	 */
	static std::string extractHostnameOrIP(
		const std::string& str, unsigned short* agent_port = NULL);

	/**\brief Pointer to the logging instance */
	mrpt::system::COutputLogger* m_logger;
	/**\brief Pointer to the Ros NodeHanle instance */
	ros::NodeHandle* m_nh;

	ros::ServiceClient m_DiscoverMasters_client;
	/**\brief List of slam agents in the current agent's neighborhood
	 *
	 * \note vector includes the GraphSlamAgent that is at the same namespace as
	 * the current CConnectionManager instance
	 */
	mrpt_msgs::GraphSlamAgents m_nearby_slam_agents;

	bool has_setup_comm;
};

}  // namespace detail
}  // namespace graphslam
}  // namespace mrpt

/**\brief ROSMaster instances are considered the same if the "uri" field is the
 * same
 */
/**\{*/
bool operator==(
	const fkie_multimaster_msgs::ROSMaster& master1,
	const fkie_multimaster_msgs::ROSMaster& master2);
bool operator!=(
	const fkie_multimaster_msgs::ROSMaster& master1,
	const fkie_multimaster_msgs::ROSMaster& master2);
/**\{*/

/**\brief GraphSlamAgent instances are considered the same if the "agent_id"
 * field is the same and the topic_namespace is the same
 */
/**\{*/
bool operator==(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2);
bool operator!=(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2);
bool operator<(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2);
/**\}*/

/**\brief GraphSlamAgent and ROSMaster instances are considered the same if the
 * corresponding "name" fields coincede
 */
/**\{*/
bool operator==(
	const fkie_multimaster_msgs::ROSMaster& master,
	const mrpt_msgs::GraphSlamAgent& agent);
bool operator==(
	const mrpt_msgs::GraphSlamAgent& agent,
	const fkie_multimaster_msgs::ROSMaster& master);
bool operator!=(
	const fkie_multimaster_msgs::ROSMaster& master,
	const mrpt_msgs::GraphSlamAgent& agent);
bool operator!=(
	const mrpt_msgs::GraphSlamAgent& agent,
	const fkie_multimaster_msgs::ROSMaster& master);

/**\}*/
