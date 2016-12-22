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
	has_setup_comm(false) {

		using namespace mrpt::utils;
		ASSERT_(m_logger);
		ASSERT_(m_nh);

	}

CConnectionManager::~CConnectionManager() { }

void CConnectionManager::getNearbySlamAgents(std::vector<TSlamAgent>* agents_vec) {

	ASSERTMSG_(agents_vec, "Invalid pointer to vector of SLAM Agents.");
	ASSERT_(has_setup_comm);

	// TODO - when should I used the cached version and when I shouldn't?
	if (!m_nearby_slam_agents_is_cached) {

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

			// hostname
			std::string hostname = this->extractHostnameOrIP(masters_it->uri);
			std::string ip_addr = this->extractHostnameOrIP(masters_it->monitoruri);

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

			// if found update the relevant fields
			if (agents_it != m_nearby_slam_agents.end()) {
				ASSERTMSG_(agents_it->hostname == hostname,
						mrpt::format("TSlamAgent hostname [%s] doesn't match [%s]",
							agents_it->hostname.c_str(), hostname.c_str()));
				ASSERTMSG_(agents_it->ip_addr == ip_addr,
						mrpt::format("TSlamAgent IP address [%s] doesn't match [%s]",
							agents_it->ip_addr.c_str(), ip_addr.c_str()));

				// update the timestamp
				mrpt_bridge::convert(ros::Time(masters_it->timestamp), agents_it->last_seen_time);
						
			}
			else { // not found, insert it.
				TSlamAgent new_agent;
				new_agent.name = masters_it->name;
				new_agent.hostname = hostname;
				new_agent.ip_addr = ip_addr;

				vector<string> tokens;
				mrpt::system::tokenize(ip_addr, ".", tokens);
				new_agent.agent_ID = atoi(tokens.rbegin()->c_str());

				cout << new_agent << endl;
				m_nearby_slam_agents.push_back(new_agent);
			}

			// if not found in RosMasters but found in TSlamAgents, remove it.
			// TODO
		}

		//m_nearby_slam_agents = true;
	}

	
	*agents_vec = m_nearby_slam_agents;


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
	// TODO - What if the "master_discovery" changes (don't know how would that happen yet..
	m_DiscoverMasters_querier =
		m_nh->serviceClient<multimaster_msgs_fkie::DiscoverMasters>("master_discovery/list_masters");

	//ASSERT_(m_DiscoverMasters_querier.isValid());
}

void convert(const multimaster_msgs_fkie::ROSMaster& ros_master, TSlamAgent* slam_agent) { }

std::string CConnectionManager::extractHostnameOrIP(const std::string& str) {
	// example: http://nickkouk-ubuntu:11311/
	std::string s = std::string(str.begin()+7, str.end());

	vector<string> tokens;
	mrpt::system::tokenize(s, ":", tokens);

	return tokens[0];
}

