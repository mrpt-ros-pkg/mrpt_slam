/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) | |
   http://www.mrpt.org/                             | | | | Copyright (c)
   2005-2016, Individual contributors, see AUTHORS file        | | See:
   http://www.mrpt.org/Authors - All rights reserved.                   | |
   Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+
 */

#include "mrpt_graphslam_2d/CConnectionManager.h"

using namespace mrpt::graphslam::detail;
using namespace std;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace fkie_multimaster_msgs;

bool operator==(
	const fkie_multimaster_msgs::ROSMaster& master1,
	const fkie_multimaster_msgs::ROSMaster& master2)
{
	return master1.uri == master2.uri;
}
bool operator!=(
	const fkie_multimaster_msgs::ROSMaster& master1,
	const fkie_multimaster_msgs::ROSMaster& master2)
{
	return !(master1 == master2);
}

bool operator==(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2)
{
	return (
		agent1.agent_id == agent2.agent_id &&
		agent1.topic_namespace.data == agent2.topic_namespace.data);
}
////////////////////////////////////////////////////////////

bool operator!=(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2)
{
	return !(agent1 == agent2);
}

bool operator<(
	const mrpt_msgs::GraphSlamAgent& agent1,
	const mrpt_msgs::GraphSlamAgent& agent2)
{
	return agent1.agent_id < agent2.agent_id;
}
////////////////////////////////////////////////////////////

bool operator==(
	const fkie_multimaster_msgs::ROSMaster& master,
	const mrpt_msgs::GraphSlamAgent& agent)
{
	return (master.name == agent.name.data);
}

bool operator!=(
	const fkie_multimaster_msgs::ROSMaster& master,
	const mrpt_msgs::GraphSlamAgent& agent)
{
	return !(master == agent);
}

bool operator==(
	const mrpt_msgs::GraphSlamAgent& agent,
	const fkie_multimaster_msgs::ROSMaster& master)
{
	return (master == agent);
}
bool operator!=(
	const mrpt_msgs::GraphSlamAgent& agent,
	const fkie_multimaster_msgs::ROSMaster& master)
{
	return (master != agent);
}

////////////////////////////////////////////////////////////

CConnectionManager::CConnectionManager(
	mrpt::system::COutputLogger* logger, ros::NodeHandle* nh_in)
	: m_logger(logger), m_nh(nh_in), has_setup_comm(false)
{
	ASSERT_(m_logger);
	ASSERT_(m_nh);

	{
		std::string own_ns_tmp = m_nh->getNamespace();
		// ignore starting "/" characters
		own_ns = std::string(
			own_ns_tmp.begin() + own_ns_tmp.find_first_not_of(" /"),
			own_ns_tmp.end());
	}

	// keep this call below the topic names initializations
	this->setupComm();
}

CConnectionManager::~CConnectionManager() {}

const std::string& CConnectionManager::getTrimmedNs() const { return own_ns; }

void CConnectionManager::getNearbySlamAgents(
	mrpt_msgs::GraphSlamAgents* agents_vec, bool ignore_self /*= true */)
{
	ASSERTMSG_(agents_vec, "Invalid pointer to vector of GraphSlam Agents.");
	this->updateNearbySlamAgents();
	*agents_vec = m_nearby_slam_agents;

	if (ignore_self)
	{
		// remove the GraphSlamAgent instance whose topic namespace coincedes
		// with the namespace that the CConnectionManager instance is running
		// under.
		auto search = [this](const mrpt_msgs::GraphSlamAgent& agent) {
			return (agent.topic_namespace.data == this->own_ns);
		};
		agents_it it =
			find_if(agents_vec->list.begin(), agents_vec->list.end(), search);

		// TODO - fix the following
		// this agent should always exist
		// TODO - well, sometimes it doesn't, investigate this
		// UPDATE: Even when /master_discovery node is up the agents vector
		// might be empty.
		// ASSERT_(it != agents_vec->list.end());
		if (it != agents_vec->list.end())
		{
			agents_vec->list.erase(it);
		}
		else
		{
		}
	}
}

const mrpt_msgs::GraphSlamAgents&
	CConnectionManager::getNearbySlamAgentsCached() const
{
	return m_nearby_slam_agents;
}

const mrpt_msgs::GraphSlamAgents& CConnectionManager::getNearbySlamAgents()
{
	this->updateNearbySlamAgents();
	return this->getNearbySlamAgentsCached();
}  // end of getNearbySlamAgents

void CConnectionManager::updateNearbySlamAgents()
{
	using ::operator==;
	using namespace mrpt::math;
	ASSERT_(has_setup_comm);

	DiscoverMasters srv;

	// ask for the agents in the neighborhood
	m_DiscoverMasters_client.call(srv);
	std::vector<ROSMaster>* masters = &(srv.response.masters);

	// convert RosMaster(s) to mrpt_msgs::GraphSlamAgent(s)
	for (std::vector<ROSMaster>::const_iterator masters_it = masters->begin();
		 masters_it != masters->end(); ++masters_it)
	{
		// 3 cases:
		// In RosMasters     AND     In mrpt_msgs::GraphSlamAgents => update
		// relevant fields In RosMasters     AND NOT In
		// mrpt_msgs::GraphSlamAgents => add it to mrpt_msgs::GraphSlamAgents
		// NOT In RosMasters AND     In mrpt_msgs::GraphSlamAgents => Do
		// nothing.

		// have I already registered the current agent?
		auto search = [masters_it](const mrpt_msgs::GraphSlamAgent& agent) {
			return agent == *masters_it;
		};
		agents_it it = find_if(
			m_nearby_slam_agents.list.begin(), m_nearby_slam_agents.list.end(),
			search);

		if (it != m_nearby_slam_agents.list.end())
		{  // found, update relevant fields
			// update timestamp
			it->last_seen_time.data = ros::Time((*masters_it).last_change);
		}
		else
		{  // not found, try to insert it.
			mrpt_msgs::GraphSlamAgent new_agent;
			bool is_agent = this->convert(*masters_it, &new_agent);
			if (is_agent)
			{
				m_nearby_slam_agents.list.push_back(new_agent);
			};
		}

	}  // for all ROSMaster(s)

}  // end of updateNearbySlamAgents

void CConnectionManager::setupComm()
{
	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	has_setup_comm = true;
}  // end of setupComm

void CConnectionManager::setupSubs() {}
void CConnectionManager::setupPubs() {}
void CConnectionManager::setupSrvs()
{
	// call to the querier should be made after the
	// fkie_multimaster_msgs::DiscoverMaster service is up and running
	m_DiscoverMasters_client =
		m_nh->serviceClient<fkie_multimaster_msgs::DiscoverMasters>(
			"/master_discovery/list_masters");

	// ASSERT_(m_DiscoverMasters_client.isValid());
}

bool CConnectionManager::convert(
	const fkie_multimaster_msgs::ROSMaster& ros_master,
	mrpt_msgs::GraphSlamAgent* slam_agent)
{
	ASSERT_(slam_agent);
	bool agent_namespace_found = false;

	slam_agent->name.data = ros_master.name;
	slam_agent->is_online.data = static_cast<bool>(ros_master.online);

	// ip_address, hostname, port
	std::string ip_addr =
		CConnectionManager::extractHostnameOrIP(ros_master.monitoruri);
	slam_agent->ip_addr.data = ip_addr;
	std::string hostname = CConnectionManager::extractHostnameOrIP(
		ros_master.uri, &slam_agent->port);
	slam_agent->hostname.data = hostname;

	// agent_id - last field of the IP address
	vector<string> tokens;
	mrpt::system::tokenize(ip_addr, ".", tokens);
	slam_agent->agent_id = atoi(tokens.rbegin()->c_str());

	// robot topic namespace
	{
		// stringstream ss("");
		// ss << slam_agent->name.data  << "_" << slam_agent->agent_id;
		// slam_agent->topic_namespace.data = ss.str().c_str();
		slam_agent->topic_namespace.data = slam_agent->name.data;

		// assert that there exists a subtopic namespace named feedback under
		// this.
		ros::master::V_TopicInfo topics;
		bool got_topics = ros::master::getTopics(topics);
		ASSERTMSG_(got_topics, "Unable to fetch topics. Exiting.");

		// get the namespaces under the current topic_namespace
		const std::string& topic_ns = "/" + slam_agent->topic_namespace.data;
		// TODO - What if this topic changes? from the configuration file
		const std::string& feedback_ns =
			"/" + slam_agent->topic_namespace.data + "/" + "feedback";

		auto search = [&feedback_ns](const ros::master::TopicInfo& topic) {
			return (strStarts(topic.name, feedback_ns));
		};
		ros::master::V_TopicInfo::const_iterator cit =
			find_if(topics.begin(), topics.end(), search);
		if (cit != topics.end())
		{
			agent_namespace_found = true;
		}
	}

	// timestamp
	slam_agent->last_seen_time.data = ros::Time(ros_master.last_change);
	return agent_namespace_found;

}  // end of convert

void CConnectionManager::convert(
	const mrpt_msgs::GraphSlamAgent& slam_agent,
	fkie_multimaster_msgs::ROSMaster* ros_master)
{
	ASSERT_(ros_master);

	ros_master->name = slam_agent.name.data;
	{
		stringstream ss("");
		ss << "http://" << slam_agent.ip_addr << ":" << slam_agent.port;
		ros_master->uri = ss.str();
	}
	ros_master->online = slam_agent.is_online.data;
	ros_master->discoverer_name = "/master_discovery";

	// TODO - timestamp
}

std::string CConnectionManager::extractHostnameOrIP(
	const std::string& str, unsigned short* agent_port /*=NULL*/)
{
	// example for monitoruri: http://nickkouk-ubuntu:11311/
	std::string s = std::string(str.begin() + 7, str.end());

	vector<string> tokens;
	mrpt::system::tokenize(s, ":", tokens);

	if (agent_port)
	{
		*agent_port = static_cast<unsigned short>(atoi(tokens[1].c_str()));
	}

	return tokens[0];
}
