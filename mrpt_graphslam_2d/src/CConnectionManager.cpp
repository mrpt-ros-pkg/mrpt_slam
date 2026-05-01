// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "mrpt_graphslam_2d/CConnectionManager.h"

using namespace mrpt::graphslam::detail;
using namespace std;
using namespace mrpt::system;
using namespace mrpt::math;

// ─── operator overloads ───────────────────────────────────────────────────────
// Note: ROS2 generated messages already provide operator== and operator!=
// as member functions. Only operator< is defined here for ordering in
// std::map / std::set.
bool operator<(
  const mrpt_msgs::msg::GraphSlamAgent & agent1,
  const mrpt_msgs::msg::GraphSlamAgent & agent2)
{
  return agent1.agent_id < agent2.agent_id;
}

// ─── CConnectionManager ───────────────────────────────────────────────────────

CConnectionManager::CConnectionManager(
  mrpt::system::COutputLogger * logger, rclcpp::Node * node)
: m_logger(logger), m_node(node), has_setup_comm(false)
{
  ASSERT_(m_logger);
  ASSERT_(m_node);

        // derive own namespace from the ROS 2 node's namespace
  {
    std::string ns_tmp = m_node->get_namespace();
                // strip leading '/' characters
    const auto first = ns_tmp.find_first_not_of(" /");
    own_ns = (first == std::string::npos) ?
      "robot" :
      std::string(ns_tmp.begin() + first, ns_tmp.end());
  }

  this->setupComm();
}

CConnectionManager::~CConnectionManager() {}

const std::string & CConnectionManager::getTrimmedNs() const {return own_ns;}

void CConnectionManager::getNearbySlamAgents(
  mrpt_msgs::msg::GraphSlamAgents * agents_vec, bool ignore_self /*= true */)
{
  ASSERTMSG_(agents_vec, "Invalid pointer to vector of GraphSlam Agents.");
  this->updateNearbySlamAgents();
  std::lock_guard<std::mutex> lock(m_agents_mutex);
  *agents_vec = m_nearby_slam_agents;

  if (ignore_self) {
    auto search = [this](const mrpt_msgs::msg::GraphSlamAgent & agent) {
        return  agent.topic_namespace.data == this->own_ns;
      };
    agents_it it =
      find_if(agents_vec->list.begin(), agents_vec->list.end(), search);

    if (it != agents_vec->list.end()) {
      agents_vec->list.erase(it);
    }
  }
}

const mrpt_msgs::msg::GraphSlamAgents &
CConnectionManager::getNearbySlamAgentsCached() const
{
  return m_nearby_slam_agents;
}

const mrpt_msgs::msg::GraphSlamAgents & CConnectionManager::getNearbySlamAgents()
{
  this->updateNearbySlamAgents();
  return this->getNearbySlamAgentsCached();
}

void CConnectionManager::updateNearbySlamAgents()
{
        // In ROS 2 the list is updated reactively via onAgentHeartbeat() callback.
        // This method only prunes stale entries (also done by prune timer).
  pruneStaleAgents();
}

void CConnectionManager::onAgentHeartbeat(
  const mrpt_msgs::msg::GraphSlamAgent::SharedPtr agent_msg)
{
  std::lock_guard<std::mutex> lock(m_agents_mutex);

  const std::string & ns = agent_msg->topic_namespace.data;

        // Update last-seen timestamp
  m_agent_last_seen[ns] = m_node->now();

        // Check if this agent is already registered
  auto search = [&agent_msg](const mrpt_msgs::msg::GraphSlamAgent & agent) {
      return  agent_msg->agent_id == agent.agent_id &&
             agent_msg->topic_namespace.data == agent.topic_namespace.data;
    };
  agents_it it = find_if(
                m_nearby_slam_agents.list.begin(), m_nearby_slam_agents.list.end(),
                search);

  if (it != m_nearby_slam_agents.list.end()) {
                // Found — update timestamp field
    it->last_seen_time = m_node->now();
    it->is_online.data = true;
  } else {
                // New agent — check that it has the expected feedback topic before
                // adding, so we know it's a valid graphSLAM agent.
    const std::string feedback_ns =
      "/" + ns + "/feedback";
    auto topic_map = m_node->get_topic_names_and_types();
    bool agent_ns_found = false;
    for (const auto & [topic, _] : topic_map) {
      if (topic.rfind(feedback_ns, 0) == 0) {
        agent_ns_found = true;
        break;
      }
    }

                // If the feedback topic is not yet visible, we still register the
                // agent (it may not have published yet), but flag it.
    mrpt_msgs::msg::GraphSlamAgent new_agent = *agent_msg;
    new_agent.is_online.data = true;
    new_agent.last_seen_time = m_node->now();
    m_nearby_slam_agents.list.push_back(new_agent);
    m_logger->logFmt(
                        LVL_INFO, "CConnectionManager: discovered new agent [%s]%s",
                        ns.c_str(),
                        agent_ns_found ? "" : " (feedback topic not yet visible)");
  }
}

void CConnectionManager::pruneStaleAgents()
{
  std::lock_guard<std::mutex> lock(m_agents_mutex);
  const double stale_threshold_s = 5.0;
  rclcpp::Time now = m_node->now();

  for (auto it = m_nearby_slam_agents.list.begin();
    it != m_nearby_slam_agents.list.end(); )
  {
    const std::string & ns = it->topic_namespace.data;
    auto seen_it = m_agent_last_seen.find(ns);
    if (seen_it != m_agent_last_seen.end()) {
      double age = (now - seen_it->second).seconds();
      if (age > stale_threshold_s) {
        m_logger->logFmt(
                                        LVL_WARN,
                                        "CConnectionManager: pruning stale agent [%s] (age %.1f s)",
                                        ns.c_str(), age);
        m_agent_last_seen.erase(seen_it);
        it = m_nearby_slam_agents.list.erase(it);
        continue;
      }
    }
    ++it;
  }
}

void CConnectionManager::setupComm()
{
  this->setupSubs();
  this->setupPubs();
  this->setupSrvs();

  has_setup_comm = true;
}

void CConnectionManager::setupSubs()
{
  m_agent_sub =
    m_node->create_subscription<mrpt_msgs::msg::GraphSlamAgent>(
                        "/mrpt_graphslam/agent_heartbeat",
                        rclcpp::QoS(10).reliable().transient_local(),
    [this](const mrpt_msgs::msg::GraphSlamAgent::SharedPtr msg) {
      this->onAgentHeartbeat(msg);
                        });
}

void CConnectionManager::setupPubs()
{
  m_agent_pub =
    m_node->create_publisher<mrpt_msgs::msg::GraphSlamAgent>(
                        "/mrpt_graphslam/agent_heartbeat",
                        rclcpp::QoS(10).reliable().transient_local());

        // Publish own agent info on a 1 Hz timer
  m_heartbeat_timer = m_node->create_wall_timer(
                std::chrono::seconds(1),
    [this]() {
      mrpt_msgs::msg::GraphSlamAgent self;
      self.name.data = own_ns;
      self.hostname.data = own_ns;
      self.topic_namespace.data = own_ns;
      self.is_online.data = true;
      self.agent_id =
      static_cast<int32_t>(std::hash<std::string>{}(own_ns) & 0x7FFFFFFF);
      self.last_seen_time = m_node->now();
      m_agent_pub->publish(self);
                });

        // Prune timer — runs every 2 s
  m_prune_timer = m_node->create_wall_timer(
                std::chrono::seconds(2),
    [this]() {this->pruneStaleAgents();});
}

void CConnectionManager::setupSrvs() {}
