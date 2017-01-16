/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMENGINE_CM_H
#define CGRAPHSLAMENGINE_CM_H

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"
#include <std_msgs/String.h>

#include <mrpt_msgs/NodeIDWithLaserScan.h>
#include <mrpt_msgs/NetworkOfPoses.h>
#include <mrpt_msgs/NodeIDWithPose_vec.h>
#include <mrpt_msgs/GetCMGraph.h> // service
#include <mrpt_bridge/network_of_poses.h>
#include <mrpt_bridge/laser_scan.h>
#include <sensor_msgs/LaserScan.h>

#include <mrpt/math/utils.h>
#include <mrpt/system/os.h>
#include <mrpt/graphslam/ERD/CLoopCloserERD.h>

#include <set>
#include <iterator>

namespace mrpt { namespace graphslam {

/** \brief mrpt::graphslam::CGraphSlamEngine derived class for interacting
 * executing CondensedMeasurements multi-robot graphSLAM
 */
template<class GRAPH_T>
class CGraphSlamEngine_CM : public CGraphSlamEngine_ROS<GRAPH_T>
{
public:
	typedef CGraphSlamEngine_ROS<GRAPH_T> parent_t;
	typedef CGraphSlamEngine_CM<GRAPH_T> self_t;
	typedef typename GRAPH_T::constraint_t::type_value pose_t;
	typedef std::pair<
		mrpt::utils::TNodeID,
		mrpt::obs::CObservation2DRangeScanPtr> MRPT_NodeIDWithLaserScan;
	typedef std::vector<mrpt::vector_uint> partitions_t;
	typedef typename mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_T> hypot_t;
	typedef std::vector<hypot_t> hypots_t;
	typedef std::vector<hypot_t*> hypotsp_t;
	typedef mrpt::graphslam::deciders::CLoopCloserERD<GRAPH_T> loop_closer_t;
	typedef typename loop_closer_t::TNodeProps TNodeProps;

	CGraphSlamEngine_CM(
			ros::NodeHandle* nh,
			const std::string& config_file,
			const std::string& rawlog_fname="",
			const std::string& fname_GT="",
			mrpt::graphslam::CWindowManager* win_manager=NULL,
			mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* node_reg=NULL,
			mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* edge_reg=NULL,
			mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* optimizer=NULL
			);

	~CGraphSlamEngine_CM();

	bool _execGraphSlamStep(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);

	void initClass();

	/**\brief Struct responsible for holding properties (nodeIDs, node
	 * positions, LaserScans) that have been registered by a nearby
	 * GraphSlamAgent.
	 */
	struct TNeighborAgentProps {
		/**\brief Constructor */
		TNeighborAgentProps(
				CGraphSlamEngine_CM<GRAPH_T>& engine_in,
				const mrpt_msgs::GraphSlamAgent& agent_in);
		/**\brief Destructor */
		~TNeighborAgentProps();

		/**\brief Setup the necessary subscribers for fetching nodes, laserScans for
		 * the current neighbor
		 */
		void setupSubs();
		/**\name Subscriber callback methods
		 * Methods to be called when data is received on the subscribed topics
		 */
		/**\{ */
		/**\brief Update nodeIDs + corresponding estimated poses */
		void updateNodes(const mrpt_msgs::NodeIDWithPose_vec::ConstPtr& nodes);
		/**\brief Fill the LaserScan of the last registered nodeID */
		void updateLastRegdIDScan(
				const mrpt_msgs::NodeIDWithLaserScan::ConstPtr& last_regd_id_scan);
		/**\} */

		/**\brief Return cached list of nodeIDs (with their corresponding poses,
		 * LaserScans) 
		 *
		 * \param[in] only_unused Include only the nodes that have not already been
		 * used in the current CGraphSlamEngine's graph
		 * \param[out] nodeIDs Pointer to vector of nodeIDs that are actually
		 * returned. This argument is redundant but may be convinient in case that
		 * just the nodeIDs are required
		 * \param[out] node_params Pointer to the map of nodeIDs \rightarrow
		 * Corresponding properties that is to be filled by the method
		 */
		void getUnusedNodes(
				mrpt::vector_uint* nodeIDs=NULL,
				std::map<
					mrpt::utils::TNodeID,
					TNodeProps>* nodes_params=NULL,
				bool only_unused=true) const;

		bool operator==(
				const TNeighborAgentProps other) const {
			return (this->agent == other.agent);
		}
		bool operator<(
				const TNeighborAgentProps other) const {
			return (this->agent < other.agent);
		}
		/**\brief Utility method for fetching the ROS LaserScan that corresponds to
		 * a nodeID
		 */
		const sensor_msgs::LaserScan& getLaserScanByNodeID(
				const mrpt::utils::TNodeID& nodeID) const;

		/**\brief Pointer to the GraphSlamAgent instance of the neighbor */
		const mrpt_msgs::GraphSlamAgent& agent;

		/**\name Neighbor cached properties */
		/**\{ */
		/**\brief NodeIDs that I have received from this graphSLAM agent. */
		std::set<mrpt::utils::TNodeID> nodeIDs_set;
		/**\brief Poses that I have received from this graphSLAM agent. */
		typename GRAPH_T::global_poses_t poses;
		/**\brief ROS LaserScans that I have received from this graphSLAM agent. */
		std::vector<mrpt_msgs::NodeIDWithLaserScan> ros_scans;
		/**\brief Have I already integrated  this node in my graph?
		 * \note CGraphSlamEngine_CM instance is responsible of setting these values to
		 * true when it integrates them in own graph
		 */
		std::map<mrpt::utils::TNodeID, bool> nodeID_to_is_integrated;
		/**\} */

		/**\name Subscriber Instances */
		/**\{ */
		ros::Subscriber last_regd_nodes_sub;
		ros::Subscriber last_regd_id_scan_sub;
		/**\} */

		/**\brief Constant reference to the outer class
		 */
		CGraphSlamEngine_CM<GRAPH_T>& engine;
		/**\name Full topic names
		 * \brief Names of the full topic paths that the neighbor publishes nodes,
		 * LaserScans at.
		 */
		/**\{ */
		std::string last_regd_nodes_topic;
		std::string last_regd_id_scan_topic;
		/**\} */
		
		int m_queue_size;
		/**\brief NodeHandle passed by the calling CGraphSlamEngine_CM class
		 */
		ros::NodeHandle* nh;

	};
	typedef std::vector<TNeighborAgentProps> neighbors_t;
	
	const neighbors_t& getVecOfNeighborAgentProps() const {
		return m_neighbors;
	}


private:
	void findMatchesWithNeighbors();
	// TODO
	void regMatchesWithNeighbors();

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

	/**\brief Overriden method that takes in account registration of multiple
	 * nodes of other running graphSLAM agents
	 *
	 */
	void monitorNodeRegistration(
			bool registered=false,
			std::string class_name="Class");

	/**\brief GraphSlamAgent instance pointer to TNeighborAgentProps
	 *
	 * \note elements of vector should persist even if the neighbor is no longer
	 * in range since they contain previous laser scans.
	 */
	neighbors_t m_neighbors;
	/**\brief Map from TNeighborAgentProps instance to a boolean indicating if
	 * that neighbor added new any new nodeID.
	 */
	std::map<TNeighborAgentProps, bool> m_neighbor_to_got_updated;
	/**\brief Class member version of the nearby SLAM agents */
	mrpt_msgs::GraphSlamAgents m_nearby_slam_agents;

	/**\name Subscribers - Publishers
	 *
	 * ROS Topic Subscriber/Publisher/Service instances
	 * */
	/**\{*/

	ros::Publisher m_list_neighbors_pub;
	/**\brief Publisher of the laserScan + the corresponding (last) registered node.
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
	std::string m_cm_ns;
	/**\brief Name of topic at which we publish information about the agents that
	 * we can currently communicate with.
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

	/**\brief Name of the server which is to be called when other agent wants to have a
	 * subgraph of certain nodes returned.
	 */
	std::string m_cm_graph_service;

	/**\}*/

	/**\brief Last known size of the m_nodes_to_laser_scans2D map 
	 */
	size_t m_nodes_to_laser_scans2D_last_size;
	/**\brief Last known size of the m_nodes map
	 */
	size_t m_graph_nodes_last_size;

	/**\brief Number of last registered NodeIDs + corresponding positions to publish.
	 *
	 * This is necessary for the other GraphSLAM agents so that they can use this
	 * information to localize the current agent in their own map and later
	 * make a querry for the Condensed Measurements Graph.
	 */
	int m_num_last_regd_nodes;


	/**\brief CConnectionManager instance */
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;


	/**\brief NodeHandle pointer - inherited by the parent. Redefined here for
	 * convenience.
	 */
	ros::NodeHandle* m_nh;

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

};


} } // end of namespaces

// pseudo-split decleration from implementation
#include "mrpt_graphslam_2d/CGraphSlamEngine_CM_impl.h"

#endif /* end of include guard: CGRAPHSLAMENGINE_CM_H */
