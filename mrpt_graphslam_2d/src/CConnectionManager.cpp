/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#include "mrpt_graphslam_2d/CConnectionManager.h"

using namespace mrpt::graphslam::detail;
using namespace std;
using namespace mrpt::utils;
using namespace multimaster_msgs_fkie;

bool operator==(
		const multimaster_msgs_fkie::ROSMaster& master1,
		const multimaster_msgs_fkie::ROSMaster& master2) {
	return master1.uri == master2.uri;
}
bool operator!=(
		const multimaster_msgs_fkie::ROSMaster& master1,
		const multimaster_msgs_fkie::ROSMaster& master2) {
	return !(master1 == master2);
}

bool operator==(
		const mrpt_msgs::GraphSlamAgent& agent1,
		const mrpt_msgs::GraphSlamAgent& agent2) {

	return (
			agent1.agent_ID == agent2.agent_ID &&
			agent1.topic_namespace.data == agent2.topic_namespace.data);
}

bool operator!=(
		const mrpt_msgs::GraphSlamAgent& agent1,
		const mrpt_msgs::GraphSlamAgent& agent2) {
	return !(agent1 == agent2);
}

bool operator<(
		const mrpt_msgs::GraphSlamAgent& agent1,
		const mrpt_msgs::GraphSlamAgent& agent2) {
	return agent1.agent_ID < agent2.agent_ID;
}

CConnectionManager::CConnectionManager(
		mrpt::utils::COutputLogger* logger,
		ros::NodeHandle* nh_in):
	m_logger(logger),
	m_nh(nh_in),
	has_setup_comm(false)
{

	using namespace mrpt::utils;
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

CConnectionManager::~CConnectionManager() { }

const std::string& CConnectionManager::getTrimmedNs() const {
	return own_ns;
}

void CConnectionManager::getNearbySlamAgents(
		mrpt_msgs::GraphSlamAgents* agents_vec,
		bool ignore_self/*= true */) {

	using namespace mrpt::math;
	using namespace std;

	ASSERTMSG_(agents_vec, "Invalid pointer to vector of GraphSlam Agents.");
	this->updateNearbySlamAgents();
	*agents_vec = m_nearby_slam_agents;

	if (ignore_self) {
		// remove the GraphSlamAgent instance whose topic namespace coincedes with
		// the namespace that the CConnectionManager instance is running under.

		auto search = [this](const mrpt_msgs::GraphSlamAgent& agent) {
			return (agent.topic_namespace.data == this->own_ns);
		};
		agents_it it = find_if(
				agents_vec->list.begin(),
				agents_vec->list.end(), search);

		// this agent should always exist
		// TODO - well, sometimes it doesn't, investigate this
		// UPDATE: Even when /master_discovery node is up the agents vector might
		// be empty.
		//ASSERT_(it != agents_vec->list.end());
		if (it != agents_vec->list.end()) {
			// TODO - remove these
			//cout << "agents_vec prior to erasing own agent" << *agents_vec << endl;
			agents_vec->list.erase(it);
			//cout << "agents_vec after erasing own agent" << *agents_vec << endl;
		}
		else {
			//cout << "Own agent not found! " << endl;
			//cout << getSTLContainerAsString(agents_vec->list) << endl;
			//cout << "Own namespace: " << m_nh->getNamespace() << endl;
		}
			//cout << "Nodes that are up: " << endl;
			//std::vector<string> nodes_up;
			//ros::master::getNodes(nodes_up);
			//printSTLContainer(nodes_up);
			//mrpt::system::pause();
	}

}

const mrpt_msgs::GraphSlamAgents&  CConnectionManager::getNearbySlamAgents() {

	this->updateNearbySlamAgents();
	return m_nearby_slam_agents;
}

void CConnectionManager::updateNearbySlamAgents() {
	ASSERT_(has_setup_comm);

	DiscoverMasters srv;

	// ask for the agents in the neighborhood
	m_DiscoverMasters_client.call(srv);
	std::vector<ROSMaster>* masters = &(srv.response.masters);

	// convert RosMaster(s) to mrpt_msgs::GraphSlamAgent(s)
	for (std::vector<ROSMaster>::const_iterator masters_it = masters->begin();
			masters_it != masters->end(); ++masters_it) {

		// resize the m_nearby_slam_agents list

		//// TODO - remove these.
		//cout << "name: " << masters_it->name << endl;
		//cout << "uri: " << masters_it->uri << endl;
		//cout << "ROS timestamp: " << masters_it->timestamp << endl;
		//cout << "online? : " << masters_it->online << endl;
		//cout << "monitoruri: " << masters_it->monitoruri << endl;
		//cout << "---------------" << endl << endl;

		// 3 cases:
		// In RosMasters     AND     In mrpt_msgs::GraphSlamAgents => update relevant fields
		// In RosMasters     BUT NOT In mrpt_msgs::GraphSlamAgents => add it to mrpt_msgs::GraphSlamAgents
		// NOT In RosMasters BUT     In mrpt_msgs::GraphSlamAgents => remove it from mrpt_msgs::GraphSlamAgents

		// have I already registered the current agent?
		auto search = [&masters_it](const mrpt_msgs::GraphSlamAgent& agent) {
			return agent.name.data == masters_it->name;
		};
		agents_it it = find_if(
				m_nearby_slam_agents.list.begin(),
				m_nearby_slam_agents.list.end(), search);

		if (it != m_nearby_slam_agents.list.end()) { // found, update fields

			// update the timestamp
			it->last_seen_time.data = ros::Time(masters_it->timestamp);

		}
		else { // not found, insert it.
			mrpt_msgs::GraphSlamAgent new_agent;
			this->convert(*masters_it, &new_agent);
			m_nearby_slam_agents.list.push_back(new_agent);
		}

	}

	// if not found in RosMasters but found in mrpt_msgs::GraphSlamAgents, remove it.
	for (agents_cit cit = m_nearby_slam_agents.list.begin(); cit !=
			m_nearby_slam_agents.list.end(); ++cit) {


		// TODO

	}

}


void CConnectionManager::setupComm() { 

	this->setupSubs();
	this->setupPubs();
	this->setupSrvs();

	has_setup_comm = true;
}

void CConnectionManager::setupSubs() { }
void CConnectionManager::setupPubs() { }
void CConnectionManager::setupSrvs() {
	// call to the querier should be made after the
	// multimaster_msgs_fkie::DiscoverMaster service is up and running
	m_DiscoverMasters_client =
		m_nh->serviceClient<multimaster_msgs_fkie::DiscoverMasters>(
				"/master_discovery/list_masters");

	//ASSERT_(m_DiscoverMasters_client.isValid());
}

void CConnectionManager::convert(const multimaster_msgs_fkie::ROSMaster& ros_master,
		mrpt_msgs::GraphSlamAgent* slam_agent) {
	ASSERT_(slam_agent);

	slam_agent->name.data = ros_master.name;
	slam_agent->is_online.data = static_cast<bool>(ros_master.online);

	// ip_address, hostname, port
	std::string ip_addr = CConnectionManager::extractHostnameOrIP(ros_master.monitoruri);
	slam_agent->ip_addr.data = ip_addr;
	std::string hostname = CConnectionManager::extractHostnameOrIP(
			ros_master.uri, &slam_agent->port);
	slam_agent->hostname.data = hostname;

	// agent_ID
	vector<string> tokens;
	mrpt::system::tokenize(ip_addr, ".", tokens);
	slam_agent->agent_ID = atoi(tokens.rbegin()->c_str());

	// robot topic namespace
	{
		stringstream ss("");
		ss << slam_agent->name.data  << "_" << slam_agent->agent_ID;
		slam_agent->topic_namespace.data = ss.str().c_str();
	}

	// timestamp
	slam_agent->last_seen_time.data = ros::Time(ros_master.timestamp);

}

void CConnectionManager::convert(
		const mrpt_msgs::GraphSlamAgent& slam_agent,
		multimaster_msgs_fkie::ROSMaster* ros_master) {

	using namespace std;
	ASSERT_(ros_master);

	ros_master->name = slam_agent.name.data;
	{
		stringstream ss("");
		ss << "http://" << slam_agent.ip_addr <<  ":" << slam_agent.port;
		ros_master->uri = ss.str();
	}
	ros_master->online = slam_agent.is_online.data;
	ros_master->discoverer_name = "/master_discovery";

	// TODO - timestamp

}

std::string CConnectionManager::extractHostnameOrIP(const std::string& str,
		unsigned short* agent_port/*=NULL*/) {
	// example for monitoruri: http://nickkouk-ubuntu:11311/
	std::string s = std::string(str.begin()+7, str.end());

	vector<string> tokens;
	mrpt::system::tokenize(s, ":", tokens);

	if (agent_port) {
		*agent_port = static_cast<unsigned short>(atoi(tokens[1].c_str()));
	}

	return tokens[0];
}

