#include "mrpt_graphslam_2d/CConnectionManager.h"

using namespace mrpt::graphslam::detail;
using namespace std;
using namespace mrpt::utils;
using namespace multimaster_msgs_fkie;

// this should be on top...
std::ostream& operator<<(
		std::ostream& os, 
		const mrpt::graphslam::detail::TSlamAgent& agent) {
	os << agent.getAsString() << std::endl;
	return os;
}

CConnectionManager::CConnectionManager(
		mrpt::utils::COutputLogger* logger,
		ros::NodeHandle* nh_in):
	m_logger(logger),
	m_nh(nh_in),
	m_nearby_slam_agents_is_cached(false),
	has_setup_comm(false)
{

	using namespace mrpt::utils;
	ASSERT_(m_logger);
	ASSERT_(m_nh);

}

CConnectionManager::~CConnectionManager() { }

void CConnectionManager::getNearbySlamAgents(
		std::vector<TSlamAgent>* agents_vec) {

	ASSERTMSG_(agents_vec, "Invalid pointer to vector of SLAM Agents.");
	ASSERT_(has_setup_comm);

	// TODO - when should I use the cached version and when I shouldn't?
	if (!m_nearby_slam_agents_is_cached) {
		this->updateNearbySlamAgents();
	}

	*agents_vec = m_nearby_slam_agents;


}

const std::vector<mrpt::graphslam::detail::TSlamAgent>&  CConnectionManager::getNearbySlamAgents() {
	if (!m_nearby_slam_agents_is_cached) {
		this->updateNearbySlamAgents();
	}

	return m_nearby_slam_agents;
}

void CConnectionManager::updateNearbySlamAgents() {

		DiscoverMasters srv;

		// ask for the agents in the neighborhood
		m_DiscoverMasters_querier.call(srv);
		std::vector<ROSMaster>* masters = &(srv.response.masters);

		cout << "Found [" << masters->size() << "] slam agents nearby" << endl;

		// convert RosMaster(s) to TSlamAgent(s)
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
			// In RosMasters - In TSlamAgents => update relevant fields
			// In RosMasters - NOT In TSlamAgents => add it to TSlamAgents
			// NOT In RosMasters - In TSlamAgents => remove it from TSlamAgents

			// have I already registered the current node?
			auto search = [&masters_it](const TSlamAgent& agent) {
				return agent.name == masters_it->name;
			};
			vector<TSlamAgent>::iterator agents_it = find_if(
					m_nearby_slam_agents.begin(),
					m_nearby_slam_agents.end(), search);

			if (agents_it != m_nearby_slam_agents.end()) { // found, update fields
				//ASSERTMSG_(agents_it->hostname == hostname,
						//mrpt::format("TSlamAgent hostname [%s] doesn't match [%s]",
							//agents_it->hostname.c_str(), hostname.c_str()));
				//ASSERTMSG_(agents_it->ip_addr == ip_addr,
						//mrpt::format("TSlamAgent IP address [%s] doesn't match [%s]",
							//agents_it->ip_addr.c_str(), ip_addr.c_str()));

				// update the timestamp
				mrpt_bridge::convert(ros::Time(masters_it->timestamp),
						agents_it->last_seen_time);
						
			}
			else { // not found, insert it.
				TSlamAgent new_agent;
				this->convert(*masters_it, &new_agent);

				cout << new_agent << endl;
				m_nearby_slam_agents.push_back(new_agent);
			}

		}

		// if not found in RosMasters but found in TSlamAgents, remove it.
		for (agents_cit cit = m_nearby_slam_agents.begin(); cit !=
				m_nearby_slam_agents.end(); ++cit) {


			// TODO

		}

		//m_nearby_slam_agents = true;

}

void CConnectionManager::setupComm() { 

	m_logger->logFmt(LVL_INFO,
			"Setting up ROS-related subscribers, publishers, services...");

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
	m_DiscoverMasters_querier =
		m_nh->serviceClient<multimaster_msgs_fkie::DiscoverMasters>(
				"master_discovery/list_masters");

	//ASSERT_(m_DiscoverMasters_querier.isValid());
}

void CConnectionManager::convert(const multimaster_msgs_fkie::ROSMaster& ros_master,
		TSlamAgent* slam_agent) {
	ASSERT_(slam_agent);

	slam_agent->name = ros_master.name;
	slam_agent->is_online = ros_master.online;

	// ip_address, hostname, port
	std::string ip_addr = CConnectionManager::extractHostnameOrIP(ros_master.monitoruri);
	slam_agent->ip_addr = ip_addr;
	std::string hostname = CConnectionManager::extractHostnameOrIP(
			ros_master.uri, &slam_agent->port);
	slam_agent->hostname = hostname;

	// agent_ID
	vector<string> tokens;
	mrpt::system::tokenize(ip_addr, ".", tokens);
	slam_agent->agent_ID = atoi(tokens.rbegin()->c_str());

	// robot topic namespace
	{
		stringstream ss;
		ss << slam_agent->name  << "_" + slam_agent->agent_ID;
		slam_agent->topic_ns = ss.str();
	}

	// timestamp
	mrpt_bridge::convert(ros::Time(ros_master.timestamp),
			slam_agent->last_seen_time);

}

void CConnectionManager::convert(
		const TSlamAgent& slam_agent,
		multimaster_msgs_fkie::ROSMaster* ros_master) {
	using namespace std;
	ASSERT_(ros_master);

	ros_master->name = slam_agent.name;
	{
		stringstream ss;
		ss << "http://" << slam_agent.ip_addr <<  ":" << slam_agent.port;
		ros_master->uri = ss.str();
	}
	ros_master->online = slam_agent.is_online;
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

