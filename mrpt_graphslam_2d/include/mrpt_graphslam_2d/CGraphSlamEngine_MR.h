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

#pragma once

// ROS 2
#include <rclcpp/rclcpp.hpp>

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_MR.h"
#include "mrpt_graphslam_2d/interfaces/CEdgeRegistrationDecider_MR.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"
#include "mrpt_graphslam_2d/misc/common.h"

#include <mrpt_msgs/msg/node_id_with_pose_vec.hpp>
#include <mrpt_msgs/msg/node_id_with_laser_scan.hpp>
#include <mrpt_msgs/msg/network_of_poses.hpp>
#include <mrpt_msgs/srv/get_cm_graph.hpp>
#include <mrpt_msgs_bridge/network_of_poses.hpp>
#include <mrpt/ros2bridge/laser_scan.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/os.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/graphs/TMRSlamNodeAnnotations.h>
#include <mrpt/graphslam/misc/TUncertaintyPath.h>
#include <mrpt/graphslam/misc/TNodeProps.h>
#include <mrpt/img/TColorManager.h>
#include <mrpt/img/TColor.h>
#include <mrpt/graphs/TNodeID.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/containers/stl_containers_utils.h>

#include <set>
#include <iterator>
#include <algorithm>

using namespace mrpt::img;
using namespace mrpt::graphs;
using namespace mrpt::config;
using namespace mrpt::system;
using namespace mrpt::containers;

namespace mrpt
{
namespace graphslam
{
/** \brief mrpt::graphslam::CGraphSlamEngine derived class for executing
 * multi-robot graphSLAM
 */
template<class GRAPH_T>
class CGraphSlamEngine_MR : public CGraphSlamEngine_ROS<GRAPH_T>
{
public:
  typedef CGraphSlamEngine_ROS<GRAPH_T> parent_t;
  typedef CGraphSlamEngine_MR<GRAPH_T> self_t;
  typedef typename GRAPH_T::constraint_t constraint_t;
  typedef typename constraint_t::type_value pose_t;
  typedef std::pair<TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>
    MRPT_NodeIDWithLaserScan;
  typedef std::map<TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>
    nodes_to_scans2D_t;
  typedef std::vector<std::vector<TNodeID>> partitions_t;
  typedef typename mrpt::graphs::detail::THypothesis<GRAPH_T> hypot_t;
  typedef std::vector<hypot_t> hypots_t;
  typedef std::vector<hypot_t *> hypotsp_t;
  typedef typename GRAPH_T::global_pose_t global_pose_t;
  typedef typename mrpt::graphslam::detail::TNodeProps<GRAPH_T> node_props_t;
  typedef mrpt::graphslam::TUncertaintyPath<GRAPH_T> path_t;
  typedef std::vector<path_t> paths_t;
  typedef mrpt::graphslam::deciders::CEdgeRegistrationDecider_MR<GRAPH_T>
    edge_reg_mr_t;

  CGraphSlamEngine_MR(
    rclcpp::Node * node, const std::string & config_file,
    const std::string & rawlog_fname = "", const std::string & fname_GT = "",
    mrpt::graphslam::CWindowManager * win_manager = NULL,
    mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T> * node_reg =
    NULL,
    mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T> * edge_reg =
    NULL,
    mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T> * optimizer =
    NULL);

  ~CGraphSlamEngine_MR();

  bool _execGraphSlamStep(
    mrpt::obs::CActionCollection::Ptr & action,
    mrpt::obs::CSensoryFrame::Ptr & observations,
    mrpt::obs::CObservation::Ptr & observation, size_t & rawlog_entry);

  void initClass();

        /**\brief Struct responsible for holding properties (nodeIDs, node
         * positions, LaserScans) that have been registered by a nearby
         * GraphSlamAgent.
         */
  struct TNeighborAgentProps
  {
                /**\brief Constructor */
    TNeighborAgentProps(
      CGraphSlamEngine_MR<GRAPH_T> & engine_in,
      const mrpt_msgs::msg::GraphSlamAgent & agent_in);
                /**\brief Destructor */
    ~TNeighborAgentProps();

                /**\brief Wrapper for calling setupSubs, setupSrvs
                 */
    void setupComm();
                /**\brief Setup the necessary subscribers for fetching nodes, laserScans
                 * for the current neighbor
                 */
    void setupSubs();
                /**\brief Setup necessary services for neighbor.
                 */
    void setupSrvs();
                /**\name Subscriber callback methods
                 * Methods to be called when data is received on the subscribed topics
                 */
                /**\{ */
                /**\brief Update nodeIDs + corresponding estimated poses */
    void fetchUpdatedNodesList(
      const mrpt_msgs::msg::NodeIDWithPoseVec::SharedPtr nodes);

                /**\brief Fill the LaserScan of the last registered nodeID */
    void fetchLastRegdIDScan(
      const mrpt_msgs::msg::NodeIDWithLaserScan::SharedPtr last_regd_id_scan);
                /**\} */

                /**\brief Return cached list of nodeIDs (with their corresponding poses,
                 * LaserScans)
                 *
                 * \param[in] only_unused Include only the nodes that have not already
                 * been used in the current CGraphSlamEngine's graph \param[out] nodeIDs
                 * Pointer to vector of nodeIDs that are actually returned. This
                 * argument is redundant but may be convinient in case that just the
                 * nodeIDs are required \param[out] node_params Pointer to the map of
                 * nodeIDs \rightarrow Corresponding properties that is to be filled by
                 * the method
                 *
                 * \note Method also calls resetFlags
                 * \sa resetFlags
                 */
    void getCachedNodes(
      std::vector<TNodeID> * nodeIDs = NULL,
      std::map<TNodeID, node_props_t> * nodes_params = NULL,
      bool only_unused = true) const;
                /**\brief Fill the optimal paths for each combination of the given
                 * nodeIDs.
                 */
    void fillOptPaths(
      const std::set<TNodeID> & nodeIDs, paths_t * opt_paths) const;
                /**\brief Using the fetched LaserScans and nodeID positions, compute the
                 * occupancy gridmap of the given neighbor
                 */
    void computeGridMap() const;
    const mrpt::maps::COccupancyGridMap2D::Ptr & getGridMap() const;
                /**\brief Fill the given occupancy gridmap object with the current
                 * neighbor's grdmap.
                 */
    void getGridMap(mrpt::maps::COccupancyGridMap2D::Ptr & map) const;
                /**\brief Return True if there are new data (node positions and
                 * corresponding LaserScans available)
                 */
    bool hasNewData() const;
    std::string getAgentNs() const
    {
      return this->agent.topic_namespace.data;
    }
    void resetFlags() const;
    bool operator==(const TNeighborAgentProps & other) const
    {
      return  this->agent.agent_id == other.agent.agent_id &&
             this->agent.topic_namespace.data ==
             other.agent.topic_namespace.data;
    }
    bool operator<(const TNeighborAgentProps & other) const
    {
      return  this->agent < other.agent;
    }
                /** Utility method for fetching the ROS LaserScan that corresponds to a
                 * nodeID.
                 */
    const sensor_msgs::msg::LaserScan * getLaserScanByNodeID(
      const TNodeID nodeID) const;

                /** Ref to the outer class.  */
    CGraphSlamEngine_MR<GRAPH_T> & engine;

                /** GraphSlamAgent instance of the neighbor. */
    const mrpt_msgs::msg::GraphSlamAgent agent;

    bool hasNewNodesBatch(int new_batch_size);

    void setTColor(const TColor & color_in) {color = color_in;}
    TColor color;

                /**\name Neighbor cached properties */
                /**\{ */
    std::set<TNodeID> nodeIDs_set;
    typename GRAPH_T::global_poses_t poses;
    std::vector<mrpt_msgs::msg::NodeIDWithLaserScan> ros_scans;
    std::map<TNodeID, bool> nodeID_to_is_integrated;
                /**\} */

                /**\name Subscriber/Service Instances */
                /**\{ */
    rclcpp::Subscription<mrpt_msgs::msg::NodeIDWithPoseVec>::SharedPtr
      last_regd_nodes_sub;
    rclcpp::Subscription<mrpt_msgs::msg::NodeIDWithLaserScan>::SharedPtr
      last_regd_id_scan_sub;

    rclcpp::Client<mrpt_msgs::srv::GetCMGraph>::SharedPtr cm_graph_srvclient;
                /**\brief Callback group for the service client (Reentrant, for
                 * blocking calls within MultiThreadedExecutor)
                 */
    rclcpp::CallbackGroup::SharedPtr cm_graph_cb_group;
                /**\} */

                /**\name Full topic names / service names */
                /**\{ */
    std::string last_regd_nodes_topic;
    std::string last_regd_id_scan_topic;
    std::string cm_graph_service;
                /**\} */

    mutable mrpt::maps::COccupancyGridMap2D::Ptr gridmap_cached;

    mutable bool has_new_nodes;
    mutable bool has_new_scans;

    int m_queue_size;
                /**\brief Pointer to the ROS 2 node (from the outer engine) */
    rclcpp::Node * nh;
    bool has_setup_comm;

    mrpt::poses::CPose2D tf_self_to_neighbor_first_integrated_pose;
    std::pair<TNodeID, mrpt::poses::CPose2D>
    last_integrated_pair_neighbor_frame;
  };
  typedef std::vector<TNeighborAgentProps *> neighbors_t;

  const neighbors_t & getVecOfNeighborAgentProps() const
  {
    return m_neighbors;
  }
  bool isOwnNodeID(
    const TNodeID nodeID, const global_pose_t * pose_out = NULL) const;

private:
  bool addNodeBatchesFromAllNeighbors();
  bool addNodeBatchFromNeighbor(TNeighborAgentProps * neighbor);
  bool findTFsWithAllNeighbors();
  bool findTFWithNeighbor(TNeighborAgentProps * neighbor);
  bool getNeighborByAgentID(
    const std::string & agent_ID_str, TNeighborAgentProps *& neighbor) const;
  bool pubUpdatedNodesList();
  bool pubLastRegdIDScan();

  void usePublishersBroadcasters();

  void setupSubs();
  void setupPubs();
  void setupSrvs();

  bool getCMGraph(
    const mrpt_msgs::srv::GetCMGraph::Request::SharedPtr req,
    mrpt_msgs::srv::GetCMGraph::Response::SharedPtr res);

  void readParams();
  void readROSParameters();
  void printParams() const;
  mrpt::poses::CPose3D getLSPoseForGridMapVisualization(
    const TNodeID nodeID) const;
  void setObjectPropsFromNodeID(
    const TNodeID nodeID, mrpt::opengl::CSetOfObjects::Ptr & viz_object);
  void monitorNodeRegistration(
    bool registered = false, std::string class_name = "Class");
  void getAllOwnNodes(std::set<TNodeID> * nodes_set) const;
  void getNodeIDsOfEstimatedTrajectory(std::set<TNodeID> * nodes_set) const;
  void getRobotEstimatedTrajectory(
    typename GRAPH_T::global_poses_t * graph_poses) const;

  neighbors_t m_neighbors;
  std::map<TNeighborAgentProps *, bool> m_neighbor_to_found_initial_tf;

        /**\name ROS 2 Publishers */
        /**\{*/
  rclcpp::Publisher<mrpt_msgs::msg::GraphSlamAgents>::SharedPtr
    m_list_neighbors_pub;
  rclcpp::Publisher<mrpt_msgs::msg::NodeIDWithLaserScan>::SharedPtr
    m_last_regd_id_scan_pub;
  rclcpp::Publisher<mrpt_msgs::msg::NodeIDWithPoseVec>::SharedPtr
    m_last_regd_nodes_pub;

  rclcpp::Service<mrpt_msgs::srv::GetCMGraph>::SharedPtr m_cm_graph_srvserver;
        /**\}*/

        /**\name Topic Names */
        /**\{*/
  std::string m_mr_ns;
  std::string m_list_neighbors_topic;
  std::string m_last_regd_id_scan_topic;
  std::string m_last_regd_nodes_topic;
  std::string m_cm_graph_service;
        /**\}*/

  size_t m_nodes_to_laser_scans2D_last_size;

  mrpt::graphslam::detail::CConnectionManager m_conn_manager;

        /**\brief Last known size of the m_nodes map */
  size_t m_graph_nodes_last_size;

  double m_offset_y_nrd;
  double m_offset_y_erd;
  double m_offset_y_gso;
  double m_offset_y_namespace;

  int m_text_index_nrd;
  int m_text_index_erd;
  int m_text_index_gso;
  int m_text_index_namespace;

  bool m_registered_multiple_nodes;

  mrpt::slam::CGridMapAligner::TConfigParams m_alignment_options;

  TColorManager m_neighbor_colors_manager;

  std::string m_sec_alignment_params;
  std::string m_sec_mr_slam_params;

  struct TOptions : CLoadableOptions
  {
    typedef self_t engine_mr_t;

    TOptions(const engine_mr_t & engine_in);
    ~TOptions();
    void loadFromConfigFile(
      const CConfigFileBase & source, const std::string & section);
    void dumpToTextStream(std::ostream & out) const;

    bool conservative_find_initial_tfs_to_neighbors;
    int nodes_integration_batch_size;
    int num_last_regd_nodes;
    size_t inter_group_node_count_thresh;
    size_t inter_group_node_count_thresh_minadv;
    const engine_mr_t & engine;
  } m_opts;
};

}  // namespace graphslam
}  // namespace mrpt

#include "mrpt_graphslam_2d/CGraphSlamEngine_MR_impl.h"
