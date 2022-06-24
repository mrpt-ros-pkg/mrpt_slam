/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) | |
   http://www.mrpt.org/                             | | | | Copyright (c)
   2005-2016, Individual contributors, see AUTHORS file        | | See:
   http://www.mrpt.org/Authors - All rights reserved.                   | |
   Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+
 */

#pragma once

#include <ros/callback_queue.h>

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_MR.h"
#include "mrpt_graphslam_2d/interfaces/CEdgeRegistrationDecider_MR.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"
#include "mrpt_graphslam_2d/misc/common.h"

#include <mrpt_msgs/NodeIDWithPoseVec.h>
#include <mrpt_msgs/NodeIDWithLaserScan.h>
#include <mrpt_msgs/NetworkOfPoses.h>
#include <mrpt_msgs/GetCMGraph.h>  // service
#include <mrpt_msgs_bridge/network_of_poses.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
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
template <class GRAPH_T>
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
	typedef std::vector<hypot_t*> hypotsp_t;
	typedef typename GRAPH_T::global_pose_t global_pose_t;
	typedef typename mrpt::graphslam::detail::TNodeProps<GRAPH_T> node_props_t;
	typedef mrpt::graphslam::TUncertaintyPath<GRAPH_T> path_t;
	typedef std::vector<path_t> paths_t;
	typedef mrpt::graphslam::deciders::CEdgeRegistrationDecider_MR<GRAPH_T>
		edge_reg_mr_t;

	CGraphSlamEngine_MR(
		ros::NodeHandle* nh, const std::string& config_file,
		const std::string& rawlog_fname = "", const std::string& fname_GT = "",
		mrpt::graphslam::CWindowManager* win_manager = NULL,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* node_reg =
			NULL,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* edge_reg =
			NULL,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* optimizer =
			NULL);

	~CGraphSlamEngine_MR();

	bool _execGraphSlamStep(
		mrpt::obs::CActionCollection::Ptr& action,
		mrpt::obs::CSensoryFrame::Ptr& observations,
		mrpt::obs::CObservation::Ptr& observation, size_t& rawlog_entry);

	void initClass();

	/**\brief Struct responsible for holding properties (nodeIDs, node
	 * positions, LaserScans) that have been registered by a nearby
	 * GraphSlamAgent.
	 */
	struct TNeighborAgentProps
	{
		/**\brief Constructor */
		TNeighborAgentProps(
			CGraphSlamEngine_MR<GRAPH_T>& engine_in,
			const mrpt_msgs::GraphSlamAgent& agent_in);
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
			const mrpt_msgs::NodeIDWithPoseVec::ConstPtr& nodes);

		/**\brief Fill the LaserScan of the last registered nodeID */
		void fetchLastRegdIDScan(
			const mrpt_msgs::NodeIDWithLaserScan::ConstPtr& last_regd_id_scan);
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
			std::vector<TNodeID>* nodeIDs = NULL,
			std::map<TNodeID, node_props_t>* nodes_params = NULL,
			bool only_unused = true) const;
		/**\brief Fill the optimal paths for each combination of the given
		 * nodeIDs.
		 */
		void fillOptPaths(
			const std::set<TNodeID>& nodeIDs, paths_t* opt_paths) const;
		/**\brief Using the fetched LaserScans and nodeID positions, compute the
		 * occupancy gridmap of the given neighbor
		 */
		void computeGridMap() const;
		const mrpt::maps::COccupancyGridMap2D::Ptr& getGridMap() const;
		/**\brief Fill the given occupancy gridmap object with the current
		 * neighbor's grdmap.
		 */
		void getGridMap(mrpt::maps::COccupancyGridMap2D::Ptr& map) const;
		/**\brief Return True if there are new data (node positions and
		 * corresponding LaserScans available)
		 *
		 * Caller is responsible of setting the underlying has_new_* variables
		 * to false using the resetFlags method upon successful integration of
		 * (some of) the nodes
		 */
		bool hasNewData() const;
		std::string getAgentNs() const
		{
			return this->agent.topic_namespace.data;
		}
		void resetFlags() const;
		bool operator==(const TNeighborAgentProps& other) const
		{
			return (this->agent == other.agent);
		}
		bool operator<(const TNeighborAgentProps& other) const
		{
			return (this->agent < other.agent);
		}
		/** Utility method for fetching the ROS LaserScan that corresponds to a
		 * nodeID.
		 */
		const sensor_msgs::LaserScan* getLaserScanByNodeID(
			const TNodeID nodeID) const;

		/** Ref to the outer class.  */
		CGraphSlamEngine_MR<GRAPH_T>& engine;

		/** GraphSlamAgent instance of the neighbor.
		 * @note This should be an actual instance instead of a ref
		 */
		const mrpt_msgs::GraphSlamAgent agent;
		/**\brief Decide whether there are enough new nodes + scans that have
		 * not been integrated in the graph yet.
		 *
		 * Method takes in account only the nodeIDs that have a valid LaserScan
		 *
		 * \return True if there are more than \a new_batch_size valid nodes +
		 * scans available for integration, false otherwise
		 */
		bool hasNewNodesBatch(int new_batch_size);

		/**\brief Set the color related to the current neighbor agent
		 *
		 * Handy in visualization operations
		 */
		void setTColor(const TColor& color_in) { color = color_in; }
		/**\brief Unique color of current TNeighborAgentProps instance
		 */
		TColor color;

		/**\name Neighbor cached properties */
		/**\{ */
		/**\brief NodeIDs that I have received from this graphSLAM agent. */
		std::set<TNodeID> nodeIDs_set;
		/**\brief Poses that I have received from this graphSLAM agent. */
		typename GRAPH_T::global_poses_t poses;
		/**\brief ROS LaserScans that I have received from this graphSLAM agent.
		 */
		std::vector<mrpt_msgs::NodeIDWithLaserScan> ros_scans;
		/**\brief Have I already integrated  this node in my graph?
		 * \note CGraphSlamEngine_MR instance is responsible of setting these
		 * values to true when it integrates them in own graph
		 */
		std::map<TNodeID, bool> nodeID_to_is_integrated;
		/**\} */

		/**\name Subscriber/Services Instances */
		/**\{ */
		ros::Subscriber last_regd_nodes_sub;
		ros::Subscriber last_regd_id_scan_sub;

		ros::ServiceClient cm_graph_srvclient;
		/**\} */

		/**\name Full topic names / service names
		 * \brief Names of the full topic paths that the neighbor publishes
		 * nodes, LaserScans at.
		 */
		/**\{ */
		std::string last_regd_nodes_topic;
		std::string last_regd_id_scan_topic;

		std::string cm_graph_service;
		/**\} */

		/**\brief Pointer to the gridmap object of the neighbor
		 */
		mutable mrpt::maps::COccupancyGridMap2D::Ptr gridmap_cached;

		mutable bool has_new_nodes;
		mutable bool has_new_scans;

		int m_queue_size;
		/**\brief NodeHandle passed by the calling CGraphSlamEngine_MR class */
		ros::NodeHandle* nh;
		bool has_setup_comm;

		/**\brief CPose2D connecting own graph origin to neighbor first received
		 * and integrated node.
		 * If this pose is 0 then it is not yet computed.
		 */
		mrpt::poses::CPose2D tf_self_to_neighbor_first_integrated_pose;
		/**\brief Last nodeID/CPose2D of neighbor that was integrated - Pose is
		 * taken with regards to neighbor's frame of reference.
		 */
		std::pair<TNodeID, mrpt::poses::CPose2D>
			last_integrated_pair_neighbor_frame;
	};
	typedef std::vector<TNeighborAgentProps*> neighbors_t;

	const neighbors_t& getVecOfNeighborAgentProps() const
	{
		return m_neighbors;
	}
	/**\brief Return true if current CGraphSlamEngine_MR object initially
	 * registered this nodeID, false otherwise.
	 * \param[in] nodeID nodeID for which the query is made.
	 * \exc runtime_error In case given nodeID isn't registered at all in the
	 * graph
	 */
	bool isOwnNodeID(
		const TNodeID nodeID, const global_pose_t* pose_out = NULL) const;

   private:
	/**\brief Traverse all neighbors for which I know the inter-graph
	 * transformation, run addNodeBatchFromNeighbor.
	 */
	bool addNodeBatchesFromAllNeighbors();
	/**\brief neighbors for which I know the inter-graph
	 * transformation, add new batches of fetches nodeIDs and scans
	 *
	 * Try to integrate the newly received nodeIDs and laser Scans in own graph
	 * as a batch
	 *
	 * \note Batch integration (instead of individual nodeID + LaserScan every
	 * time a new one is received) is preferred since the neighbor is given time
	 * to optimize the node positions
	 */
	bool addNodeBatchFromNeighbor(TNeighborAgentProps* neighbor);
	/**\brief Using map-merging find a potentuial transofrmation between own
	 * graph map to another agent's map and using that transformation add other
	 * agent's nodes to own graph
	 *
	 * Method delegates task to the findMatchesWithNeighboor method.
	 *
	 * \return True on successful graph integration with any neighbor, false
	 * otherwise
	 */
	bool findTFsWithAllNeighbors();
	/**
	 * \brief Method is used only when I haven't found any transformation
	 * between mine and the current neighbor's graph. When I do find one, I just
	 * add every new nodeID + LS directly in own graph since I can now connect
	 * the new ones with the already integrated nodes of the neighbor.
	 *
	 * \sa findTFsWithAllNeighbors
	 */
	bool findTFWithNeighbor(TNeighborAgentProps* neighbor);
	/**\brief Fill the TNeighborAgentProps instance based on the given
	 * agent_ID_str string that is provided.
	 *
	 * \return False if the agent_ID_str doesn't correspond to any registered
	 * TNeighborAgentProps, True otherwise
	 */
	bool getNeighborByAgentID(
		const std::string& agent_ID_str, TNeighborAgentProps*& neighbor) const;
	/**\brief Update the last registered NodeIDs and corresponding positions of
	 * the current agent.
	 *
	 * \note Method is automatically called from usePublishersBroadcasters, but
	 * it publishes to the corresponding topics only on new node additions.
	 * Node positions may change due to optimization but we will be notified of
	 * this change anyway after a new node addition.
	 *
	 * \return true on publication to corresponding topics, false otherwise
	 * \sa pubLastRegdIDScan
	 */
	bool pubUpdatedNodesList();
	/**\brief Update the last registered NodeID and LaserScan of the current
	 * agent.
	 *
	 * \note Method is automatically called from usePublishersBroadcasters, but
	 * it publishes to the corresponding topics only on new node/LS additions
	 *
	 * \return true on publication to corresponding topics, false otherwise
	 * \sa pubUpdatedNodesList
	 */
	bool pubLastRegdIDScan();

	void usePublishersBroadcasters();

	void setupSubs();
	void setupPubs();
	void setupSrvs();

	/**\brief Compute and fill the Condensed Measurements Graph
	 */
	bool getCMGraph(
		mrpt_msgs::GetCMGraph::Request& req,
		mrpt_msgs::GetCMGraph::Response& res);

	void readParams();
	void readROSParameters();
	void printParams() const;
	mrpt::poses::CPose3D getLSPoseForGridMapVisualization(
		const TNodeID nodeID) const;
	void setObjectPropsFromNodeID(
		const TNodeID nodeID, mrpt::opengl::CSetOfObjects::Ptr& viz_object);
	/**\brief Overriden method that takes in account registration of multiple
	 * nodes of other running graphSLAM agents
	 *
	 */
	void monitorNodeRegistration(
		bool registered = false, std::string class_name = "Class");
	/**\brief Fill given set with the nodeIDs that were initially registered by
	 * the current graphSLAM engine (and not by any neighboring agent.
	 */
	void getAllOwnNodes(std::set<TNodeID>* nodes_set) const;
	void getNodeIDsOfEstimatedTrajectory(std::set<TNodeID>* nodes_set) const;
	void getRobotEstimatedTrajectory(
		typename GRAPH_T::global_poses_t* graph_poses) const;

	/**\brief GraphSlamAgent instance pointer to TNeighborAgentProps
	 *
	 * \note elements of vector should persist even if the neighbor is no longer
	 * in range since they contain previous laser scans.
	 */
	neighbors_t m_neighbors;
	/**\brief Map from neighbor instance to transformation from own to
	 * neighbor's graph
	 *
	 * As long as the corresponding variable for a specific agent is False
	 * current engine keeps trying the map-matching proc to find a common
	 * transformation between own and neighbor graphs.
	 */
	std::map<TNeighborAgentProps*, bool> m_neighbor_to_found_initial_tf;

	/**\name Subscribers - Publishers
	 *
	 * ROS Topic Subscriber/Publisher/Service instances
	 * */
	/**\{*/

	ros::Publisher m_list_neighbors_pub;
	/**\brief Publisher of the laserScan + the corresponding (last) registered
	 * node.
	 */
	ros::Publisher m_last_regd_id_scan_pub;
	/**\brief Publisher of the last registered nodeIDs and positions.
	 *
	 * \note see m_num_last_regd_nodes variable for the exact number of
	 * published nodeID and position pairs.
	 */
	ros::Publisher m_last_regd_nodes_pub;

	ros::ServiceServer m_cm_graph_srvserver;
	/**\}*/

	/**\name Topic Names
	 *
	 * Names of the topics that the class instance subscribes or publishes to
	 */
	/**\{*/

	/**\brief Condensed Measurements topic \a namespace */
	std::string m_mr_ns;
	/**\brief Name of topic at which we publish information about the agents
	 * that we can currently communicate with.
	 */
	std::string m_list_neighbors_topic;
	/**\brief Name of the topic that the last \b registered laser scan (+
	 * corresponding nodeID) is published at
	 */
	std::string m_last_regd_id_scan_topic;
	/**\brief Name of the topic that the last X registered nodes + positions
	 * are going to be published at
	 */
	std::string m_last_regd_nodes_topic;

	/**\brief Name of the server which is to be called when other agent wants to
	 * have a subgraph of certain nodes returned.
	 */
	std::string m_cm_graph_service;

	/**\}*/

	/**\brief Last known size of the m_nodes_to_laser_scans2D map */
	size_t m_nodes_to_laser_scans2D_last_size;

	/**\brief CConnectionManager instance */
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;

	/**\brief NodeHandle pointer - inherited by the parent. Redefined here for
	 * convenience.
	 */
	ros::NodeHandle* m_nh;
	/**\brief Last known size of the m_nodes map */
	size_t m_graph_nodes_last_size;

	/**\brief Display the Deciders/Optimizers with which we are running as well
	 * as the namespace of the current agent.
	 */
	/**\{ */
	double m_offset_y_nrd;
	double m_offset_y_erd;
	double m_offset_y_gso;
	double m_offset_y_namespace;

	int m_text_index_nrd;
	int m_text_index_erd;
	int m_text_index_gso;
	int m_text_index_namespace;
	/**\} */

	/**\brief Indicates whether multiple nodes were just registered.
	 * Used for checking correct node registration in the monitorNodeRgistration
	 * method.
	 */
	bool m_registered_multiple_nodes;

	/**\brief AsyncSpinner that is used to query the CM-Graph service in case a
	 * new request arrives
	 */
	ros::AsyncSpinner cm_graph_async_spinner;

	/**\brief Parameters used during the alignment operation
	 */
	mrpt::slam::CGridMapAligner::TConfigParams m_alignment_options;

	TColorManager m_neighbor_colors_manager;

	std::string m_sec_alignment_params;
	std::string m_sec_mr_slam_params;

	struct TOptions : CLoadableOptions
	{
		typedef self_t engine_mr_t;

		TOptions(const engine_mr_t& engine_in);
		~TOptions();
		void loadFromConfigFile(
			const CConfigFileBase& source, const std::string& section);
		void dumpToTextStream(std::ostream& out) const;

		/**\brief Be conservative when it comes to deciding the initial
		 * transformation of own graph with regards to graphs of the neighboring
		 * agents. If true engine won't use map merging but instead will be
		 * waiting for rendez-vous with other agents to determine the tf.
		 *
		 * \warning Rendez-vous behavior is not yet implemented.
		 */
		bool conservative_find_initial_tfs_to_neighbors;
		/**\brief After an inter-graph transformation is found between own graph
		 * and a neighbor's map, newly fetched neighbor's nodes are added in
		 * batches.
		 */
		int nodes_integration_batch_size;
		/**\brief Max number of last registered NodeIDs + corresponding
		 * positions to publish.
		 *
		 * This is necessary for the other GraphSLAM agents so that they can use
		 * this information to localize the current agent in their own map and
		 * later make a querry for the Condensed Measurements Graph.
		 */
		int num_last_regd_nodes;
		/**\brief Lowest number of nodes that should exist in a group of nodes
		 * before evaluating it. These nodes are fetched by the other running
		 * graphSLAM agents
		 *
		 * \note This should be set >= 3
		 * \sa inter_group_node_count_thresh_minadv
		 */
		size_t inter_group_node_count_thresh;
		/**\brief Minimum advised limit of inter_group_node_count_thresh
		 * \sa inter_group_node_count_thresh
		 */
		size_t inter_group_node_count_thresh_minadv;

		/** Instance of engine which uses this struct */
		const engine_mr_t& engine;

	} m_opts;
};

}  // namespace graphslam
}  // namespace mrpt

// pseudo-split decleration from implementation
#include "mrpt_graphslam_2d/CGraphSlamEngine_MR_impl.h"
