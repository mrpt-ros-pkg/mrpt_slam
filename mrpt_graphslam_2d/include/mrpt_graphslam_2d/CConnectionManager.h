#ifndef CCONNECTIONMANAGER_H
#define CCONNECTIONMANAGER_H

#include <ros/ros.h>
#include <multimaster_msgs_fkie/DiscoverMasters.h>
#include <mrpt_bridge/mrpt_bridge.h>

#include <mrpt/utils/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <cstdlib>


#define INVALID_SLAM_AGENT_ID static_cast<size_t>(-1)

namespace mrpt { namespace graphslam { namespace detail {


/**\brief Struct that holds the properties of a SLAM agent, capable of
 * communicating with the current connection manager
 */
struct TSlamAgent {
	TSlamAgent():
		agent_ID(INVALID_SLAM_AGENT_ID),
		last_seen_time(INVALID_TIMESTAMP) { }
	~TSlamAgent() { }

	bool operator==(const TSlamAgent& o) const {
		return this->agent_ID == o.agent_ID;
	}
	bool operator!=(const TSlamAgent& o) const {
		return !(*this == o);
	}

	// TODO - Is \name needed here?
	/**\brief Get a string representation of the current Slam Agent
	 *
	 * \param[in] is_oneline If true then the returned string is going to be a one
	 * liner. Use it for a more compact representation
	 */
	/**\{*/
	std::string getAsString(const bool is_oneline=false) const {
		std::string str;
		this->getAsString(&str, is_oneline);
		return str;
	}
	/**\param[out] str_out String object to be filled */
	void getAsString(std::string* str_out, const bool is_oneline=false) const {
		using namespace std;
		using namespace mrpt;
		ASSERT_(str_out);

		string header('=', 20);
		stringstream ss;

		if (!is_oneline) {
			ss << "Slam Agent - " << name << endl;
			ss << header << endl;
		}
		else {
			ss << "Slam Agent: ";
		}

		// fields to print...
		vector<string> substrings;
		substrings.push_back(format("Name: %s", name.c_str()));
		substrings.push_back(format("Hostname: %s", hostname.c_str()));
		substrings.push_back(format("IP Addr: %s", ip_addr.c_str()));
		{
			stringstream tmp;
			tmp << "Agent ID: " << agent_ID;
			substrings.push_back(format("%s", tmp.str().c_str()));
		}
		{
			stringstream tmp;
			tmp << "Last Seen: " << last_seen_time;
			substrings.push_back(format("%s", tmp.str().c_str()));
		}

		for (vector<string>::iterator str_it = substrings.begin();
				str_it != substrings.end(); ++str_it) {

			if (is_oneline) {
				*str_it = *str_it + "|\t";
			}
			else {
				*str_it = *str_it + "\n";
			}

			ss << *str_it;
		}

		*str_out = ss.str();
		mrpt::system::trim(*str_out); // trim end tab
	}
	/**\}*/

	/**\brief Agent name - This is a concatenation of the hostname and the port
	 * that the corresponding ROS Master runs on
	 */
	std::string name;
	std::string hostname;
	std::string ip_addr;
	// TODO - store the ROS topics namespace
	/**\brief SLAM Agent ID
	 *
	 * It is advised that this is set to the last part of the IPv4 string
	 * e.g. IP: 192.168.1.15 => agent_ID: 15
	 */
	size_t agent_ID;
	mrpt::system::TTimeStamp last_seen_time;


};

/**\brief Class responsible of handling the network communication between SLAM
 * agents in the Multi-Robot Condensed Measurements graphSLAM algorithm.
 */
class CConnectionManager
{
public:
	/**\brief Constructor */
	CConnectionManager(
			mrpt::utils::COutputLogger* logger,
			ros::NodeHandle* nh_in);
	/**\brief Destructor */
	~CConnectionManager();
	/**\brief Fill the given vector with the SLAM Agents that the current manager
	 * can see and communicate with
	 *
	 * \sa updateNearbySlamAgents
	 */
	void getNearbySlamAgents(std::vector<TSlamAgent>* agents_vec);
	/**\brief Read-only method for accessing list of nearby agents
	 */
	const std::vector<TSlamAgent>&  getNearbySlamAgents();

	/**\brief Wrapper method around the private setup* class methods.
	 *
	 * Handy for setting up publishers, subscribers, services, TF-related stuff
	 * all at once from the user application
	 *
	 */
	void setupComm();

private:
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

	/**\brief Convert the ROSMaster instance to the corresponding TSlamAgent. */
	void convert(const multimaster_msgs_fkie::ROSMaster& ros_master, TSlamAgent* slam_agent);

	/**\brief Pointer to the logging instance */
	mrpt::utils::COutputLogger* m_logger;
	/**\brief Pointer to the Ros NodeHanle instance */
	ros::NodeHandle* m_nh;

	ros::ServiceClient m_DiscoverMasters_querier;
	/**\brief List of slam agents in the current agent's neighborhood */
	std::vector<TSlamAgent> m_nearby_slam_agents;
	/**\brief Indicates whether the list of slam agents is up-to-date. */
	bool m_nearby_slam_agents_is_cached;

	bool has_setup_comm;

	static std::string extractHostnameOrIP(const std::string& str);


};

} } } // end of namespaces

std::ostream& operator<<(std::ostream& os, const mrpt::graphslam::detail::TSlamAgent& agent);

#endif /* end of include guard: CCONNECTIONMANAGER_H */
