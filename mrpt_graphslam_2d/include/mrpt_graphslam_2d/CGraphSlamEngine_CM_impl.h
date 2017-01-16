/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMENGINE_CM_IMPL_H
#define CGRAPHSLAMENGINE_CM_IMPL_H

namespace mrpt { namespace graphslam {

template<class GRAPH_T>
CGraphSlamEngine_CM<GRAPH_T>::CGraphSlamEngine_CM(
		ros::NodeHandle* nh,
		const std::string& config_file,
		const std::string& rawlog_fname/* ="" */,
		const std::string& fname_GT /* ="" */,
		mrpt::graphslam::CWindowManager* win_manager /* = NULL */,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>* node_reg /* = NULL */,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>* edge_reg /* = NULL */,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>* optimizer /* = NULL */):
	parent_t::CGraphSlamEngine_ROS(
			nh,
			config_file,
			rawlog_fname,
			fname_GT,
			win_manager,
			node_reg,
			edge_reg,
			optimizer),
	m_conn_manager(
			dynamic_cast<mrpt::utils::COutputLogger*>(this), nh),
	m_nh(nh)
{

	this->initClass();

}

template<class GRAPH_T>
CGraphSlamEngine_CM<GRAPH_T>::~CGraphSlamEngine_CM() {
}

template<class GRAPH_T>
bool CGraphSlamEngine_CM<GRAPH_T>::_execGraphSlamStep(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) { 
	THROW_EXCEPTION("CGraphSlamEngine_CM works with CNetworkOfPoses2DInf_NA types");
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::findMatchesWithNeighbors() { 
	THROW_EXCEPTION("CGraphSlamEngine_CM works with CNetworkOfPoses2DInf_NA types");
}

template<>
void CGraphSlamEngine_CM<mrpt::graphs::CNetworkOfPoses2DInf_NA>::
findMatchesWithNeighbors() {

	// we are dealing with a CLoopCloserERD type in the mr-case.
	loop_closer_t* edge_reg =
		dynamic_cast<loop_closer_t*>(this->m_edge_reg);
	ASSERTMSG_(edge_reg,
			"Only the CLoopCloserERD classes and its derivatives can be used.");

	// groupA => own nodes
	// Utilize all own partitions
	// Each partition will be taken as a group in itself => groupA
	partitions_t own_partitions;
	edge_reg->getCurrPartitions(&own_partitions);

	for(partitions_t::iterator
			partitions_it = own_partitions.begin();
			partitions_it != own_partitions.end();
			++partitions_it) {

 		// groupA => own nodes
		vector_uint& groupA = *partitions_it;
		// Avoid multiple integration (propagation) of information - see Lazaro
		// IV.b
		// remove all the nodeIDs that correspond to nodes registered by other
		// agents.
		const std::string& own_ns = m_conn_manager.getTrimmedNs();
		for (vector_uint::iterator group_it = groupA.begin();
				group_it != groupA.end();
				++group_it) {
			if (m_graph.nodes.at(*group_it).agent_ID_str != own_ns) {
				groupA.erase(group_it);
			}
		}

		// get the registered nodes of the other agents.
		for (typename std::map<TNeighborAgentProps, bool>::iterator
				neighbors_it = m_neighbor_to_got_updated.begin();
				neighbors_it != m_neighbor_to_got_updated.end();
				++neighbors_it) {

			// if its status isn't updated don't run matching proc.
			if (!neighbors_it->second) { continue; }

			// groupB => neighbor's cached registered nodes
			// Ask only of the nodes that haven't been already integrated in the
			// graph
			vector_uint groupB;
			std::map<
				mrpt::utils::TNodeID,
				TNodeProps> nodes_params;
			neighbors_it->first.getUnusedNodes(&groupB, &nodes_params, /*only_unused=*/true);

			// run matching proc. between groupA and groupB
			hypotsp_t hypots_pool;
			hypotsp_t valid_hypots;
			// TODO: Compilation error
			edge_reg->generateHypotsPool(groupA, groupB, &hypots_pool);
			edge_reg->evaluateGroups(groupA, groupB, hypots_pool, &valid_hypots);

			// continue only if matches are found.
			if (!valid_hypots.size()) { continue; }
			// valid hypots have been found!

			MRPT_LOG_ERROR_STREAM << "Matches have been found!";

			// Ask for the condensed graph of the matched nodes

			// REnumber the nodes according to own graph - start from latest
			// registered nodeID
			//
			// TODO

			// Insert nodes/edges in the graph
			// TODO

			// Call for a Full graph visualization update and Dijkstra update -
			// CGraphSlamOptimizer
			// TODO

			// delete all hypotheses - generated in the heap...
			MRPT_LOG_DEBUG_STREAM << "Deleting the generated hypotheses pool..." ;
			for (typename hypotsp_t::iterator
					it = hypots_pool.begin(); it != hypots_pool.end(); ++it) {
				delete *it;
			}
		}
	}
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::regMatchesWithNeighbors() { }


template<>
bool CGraphSlamEngine_CM<mrpt::graphs::CNetworkOfPoses2DInf_NA>::
_execGraphSlamStep(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) { 
	// TODO - Check the whole method - all the associated method calls,
	// arguments, NULLs, print messages, ==> This has to work *correctly*

	using namespace mrpt::graphslam::deciders;
	using namespace mrpt;

	// call parent method
	parent_t::_execGraphSlamStep(
			action, observations, observation, rawlog_entry);

	// find matches between own nodes and those of the neighbors
	//this->findMatchesWithNeighbors();
	//this->regMatchesWithNeighbors();
} // end of _execGraphSlamStep

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::initClass() {
	using namespace mrpt::graphslam;
	using namespace mrpt::utils;

	// initialization of topic namespace names
	// TODO - put these into seperate method
	m_cm_ns = "cm_info";

	// initialization of topic names
	// TODO - put these into seperate method
	m_list_neighbors_topic   = m_cm_ns + "/" + "neighbors";
	m_cm_graph_service       = m_cm_ns + "/" + "get_cm_graph";
	m_last_regd_id_scan_topic = m_cm_ns + "/" + "last_nodeID_laser_scan";
	m_last_regd_nodes_topic = m_cm_ns + "/" + "last_regd_nodes";

	this->m_class_name = "CGraphSlamEngine_CM_" + m_conn_manager.getTrimmedNs();
	this->setLoggerName(this->m_class_name);
	this->setupComm();

	// Make sure that master_discovery and master_sync are up before trying to
	// connect to other agents
	std::vector<string> nodes_up;
	ros::master::getNodes(nodes_up);
	// If both master_dicovery and master_sync are running, then the
	// /master_sync/get_sync_info service should be available
	MRPT_LOG_INFO_STREAM << "Waiting for master_discovery, master_sync nodes to come up....";
	ros::service::waitForService("/master_sync/get_sync_info"); // block until it is
	MRPT_LOG_INFO_STREAM << "master_discovery, master_sync are available.";

	// in case of CondensedMeasurements specific deciders/optimizers (they
	// inherit from the CDeciderOrOptimizer_ROS interface) set the
	// CConnectionManager*
	// NOTE: It's not certain, even though we are running the Condensed measurements
	// algorithm that all these classes do inherit from the
	// CRegistrationDeciderOrOptimizer_CM base class. They might be more generic and not
	// care about the multi-robot nature of the algorithm (e.g. optimization
	// scheme)
	{ // NRD
	 	CRegistrationDeciderOrOptimizer_CM<GRAPH_T>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_T>*>(this->m_node_reg);

	 	if (dec_opt_cm) {
	 	 	dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
	 	}
	}
	{ // ERD - CLoopCloserERD_CM in any case.
		CRegistrationDeciderOrOptimizer_CM<GRAPH_T>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_T>*>(this->m_edge_reg);
		ASSERT_(dec_opt_cm);
	 	dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
	 	dec_opt_cm->setCGraphSlamEnginePtr(this);
	}
	{ // GSO
		CRegistrationDeciderOrOptimizer_CM<GRAPH_T>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_T>*>(this->m_optimizer);
		if (dec_opt_cm) {
	 		dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
		}
	}

	// display messages with the names of the deciders, optimizer and agent string ID
	if (this->m_enable_visuals) {
		// NRD
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y* = */ &m_offset_y_nrd,
				/* text_index* = */ &m_text_index_nrd);
		this->m_win_manager->addTextMessage(this->m_offset_x_left, -m_offset_y_nrd,
				mrpt::format("NRD: %s", this->m_node_reg->getClassName().c_str()),
				TColorf(0,0,0),
				/* unique_index = */ m_text_index_nrd);

		// ERD
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y* = */ &m_offset_y_erd,
				/* text_index* = */ &m_text_index_erd);
		this->m_win_manager->addTextMessage(
				this->m_offset_x_left, -m_offset_y_erd,
				mrpt::format("ERD: %s", this->m_edge_reg->getClassName().c_str()),
				TColorf(0,0,0),
				/* unique_index = */ m_text_index_erd);

		// GSO
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y* = */ &m_offset_y_gso,
				/* text_index* = */ &m_text_index_gso);
		this->m_win_manager->addTextMessage(
				this->m_offset_x_left, -m_offset_y_gso,
				mrpt::format("GSO: %s", this->m_optimizer->getClassName().c_str()),
				TColorf(0,0,0),
				/* unique_index = */ m_text_index_gso);

		// Agent Namespace
		this->m_win_manager->assignTextMessageParameters(
				/* offset_y* = */ &m_offset_y_namespace,
				/* text_index* = */ &m_text_index_namespace);
		this->m_win_manager->addTextMessage(
				this->m_offset_x_left, -m_offset_y_namespace,
				mrpt::format("Agent: %s", m_nh->getNamespace().c_str()),
				TColorf(0,0,0),
				/* unique_index = */ m_text_index_namespace);

	}
	
	
	this->readParams();
}


template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::usePublishersBroadcasters() {
	using namespace mrpt_bridge;
	using namespace mrpt_msgs;
	using namespace std;
	using namespace mrpt::math;
	using ::operator==;

	// call the parent class
	parent_t::usePublishersBroadcasters();

	// update list of neighbors that the current agent can communicate with.
	m_conn_manager.getNearbySlamAgents(
				&m_nearby_slam_agents,
				/*ignore_self = */ true);
	m_list_neighbors_pub.publish(m_nearby_slam_agents);

	// for each *new* GraphSlamAgent we should add a TNeighborAgentProps instance,
	// and initialize its subscribers so that we fetch every new LaserScan,
	// modified nodes list it publishes
	{
		for (GraphSlamAgents::_list_type::const_iterator
				it = m_nearby_slam_agents.list.begin();
				it != m_nearby_slam_agents.list.end();
				++it) {

			const GraphSlamAgent& gsa = *it;

			// Is the current GraphSlamAgent already registered?
			auto search = [gsa](const TNeighborAgentProps& neighbor) {
				return (neighbor.agent == gsa);
			};
			typename neighbors_t::iterator neighbor_it = find_if(
					m_neighbors.begin(),
					m_neighbors.end(), search);

			if (neighbor_it == m_neighbors.end()) { // current gsa not found, add it

				m_neighbors.push_back(TNeighborAgentProps(*this, gsa));
				TNeighborAgentProps& latest_neighbor = m_neighbors.back();
				latest_neighbor.setupSubs();
				MRPT_LOG_INFO_STREAM << m_nh->getNamespace()
					<< ": Initialized NeighborAgentProps instance..."
					<< endl;

				m_neighbor_to_got_updated.insert(make_pair(
							latest_neighbor, false));
			}

		}
	}


	// TODO - do not resent the nodes received and integrated by other agents
	// update the last X NodeIDs + positions; Do it only when a new node is inserted.
	//
	// WARNING: Node positions may change due to optimization but we will be
	// notified of this change anyway after a new node addition
	//
	// - Nodes with the larger IDs are inserted at the *end* of the map
	// - Nodes with the smallest IDs are popped from the beginning of the map
	if (this->m_graph_nodes_last_size < this->m_graph.nodes.size()) {

		// at which nodeID do I start inserting?
		typename GRAPH_T::global_poses_t::const_iterator start_elem =
			m_num_last_regd_nodes > this->m_graph.nodes.size() ?
			this->m_graph.nodes.begin() :
			std::prev(this->m_graph.nodes.end(), m_num_last_regd_nodes);

		// fill the NodeIDWithPose_vec msg
		NodeIDWithPose_vec ros_nodes;

		for (typename GRAPH_T::global_poses_t::const_iterator cit = start_elem;
				cit != this->m_graph.nodes.end();
				++cit) {

			mrpt_msgs::NodeIDWithPose curr_node;
			curr_node.nodeID = cit->first; // nodeID
			mrpt_bridge::convert(cit->second, curr_node.pose); // pose

			ros_nodes.vec.push_back(curr_node);
		}

		m_last_regd_nodes_pub.publish(ros_nodes);


		// update the last known size
		m_graph_nodes_last_size = this->m_graph.nodes.size();
	}

	// Update the last registered scan + associated nodeID
	// Last registered scan always corresponds to the *last* element of the of the
	// published NodeIDWithPose_vec that is published above.
	//
	// - Check if map is empty
	// - Have I already published the last laser scan?
	if (!this->m_nodes_to_laser_scans2D.empty() &&
			m_nodes_to_laser_scans2D_last_size < this->m_nodes_to_laser_scans2D.size()) {
		// last registered scan
		MRPT_NodeIDWithLaserScan mrpt_last_regd_id_scan =
			*(this->m_nodes_to_laser_scans2D.rend());
		ASSERT_(mrpt_last_regd_id_scan.second);

		// convert to ROS msg
		mrpt_msgs::NodeIDWithLaserScan ros_last_regd_id_scan;
		convert(*(mrpt_last_regd_id_scan.second), ros_last_regd_id_scan.scan);
		ros_last_regd_id_scan.nodeID = mrpt_last_regd_id_scan.first;

		m_last_regd_id_scan_pub.publish(ros_last_regd_id_scan);

		// update the last known size.
		m_nodes_to_laser_scans2D_last_size = this->m_nodes_to_laser_scans2D.size();
	}

} // end of usePublishersBroadcasters

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::readParams() {
	this->readROSParameters();
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::readROSParameters() {
	// call parent method first in case the parent wants to ask for any ROS
	// parameters
	parent_t::readROSParameters();

	m_nh->param<int>(m_cm_ns + "/" + "num_last_registered_nodes",
			m_num_last_regd_nodes, 10);

}


template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::setupSubs() { }

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::setupPubs() {
	using namespace mrpt_msgs;
	using namespace sensor_msgs;

	// 
	m_list_neighbors_pub = m_nh->advertise<GraphSlamAgents>(
			m_list_neighbors_topic,
			this->m_queue_size,
			/*latch = */ true);

	// last registered laser scan - by default this corresponds to the last
	// nodeID of the vector of last registered nodes.
	m_last_regd_id_scan_pub = m_nh->advertise<NodeIDWithLaserScan>(
			m_last_regd_id_scan_topic,
			this->m_queue_size,
			/*latch = */ true);

	// last X nodeIDs + positions
	m_last_regd_nodes_pub = m_nh->advertise<NodeIDWithPose_vec>(
			m_last_regd_nodes_topic,
			this->m_queue_size,
			/*latch = */ true);



}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::setupSrvs() {

	m_cm_graph_srvserver = this->m_nh->template advertiseService(
			m_cm_graph_service,
			&CGraphSlamEngine_CM<GRAPH_T>::getCMGraph, this);

	// service for asking for others' cm graphs?
}

template<class GRAPH_T>
bool CGraphSlamEngine_CM<GRAPH_T>::getCMGraph(
		mrpt_msgs::GetCMGraph::Request& req,
		mrpt_msgs::GetCMGraph::Response& res) {
	
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	set<TNodeID> nodes_set(req.nodeIDs.begin(), req.nodeIDs.end());
	MRPT_LOG_INFO_STREAM << "Called the GetCMGraph service for nodeIDs: " << getSTLContainerAsString(nodes_set);

	// fill the given Response with the ROS NetworkOfPoses
	GRAPH_T mrpt_subgraph;
	this->m_graph.extractSubGraph(nodes_set, &mrpt_subgraph,
			/*root_node = */ INVALID_NODEID,
			/*auto_expand_set=*/false);
	mrpt_subgraph.saveToTextFile("extracted_graph.graph");
	mrpt_bridge::convert(mrpt_subgraph, res.cm_graph);

	return true;
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::monitorNodeRegistration(
		bool registered/*=false*/,
		std::string class_name/*="Class"*/) {
	if (m_registered_multiple_nodes) {
		m_registered_multiple_nodes = !m_registered_multiple_nodes;
	}
	else {
		parent_t::monitorNodeRegistration(registered, class_name);
	}
}

//////////////////////////////////////////////////////////////////////////////////////

template<class GRAPH_T>
CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::TNeighborAgentProps(
		CGraphSlamEngine_CM<GRAPH_T>& engine_in,
		const mrpt_msgs::GraphSlamAgent& agent_in):
	engine(engine_in),
	agent(agent_in),
	m_queue_size(1)
{

	nh = engine.m_nh;

	// fill the full paths of the topics to subscribe
	// ASSUMPTION: agents namespaces start at the root "/"
	this->last_regd_nodes_topic = "/" + agent.topic_namespace.data + "/" +
		engine.m_last_regd_nodes_topic;
	this->last_regd_id_scan_topic = "/" + agent.topic_namespace.data + "/" +
		engine.m_last_regd_id_scan_topic;

}

template<class GRAPH_T>
CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::~TNeighborAgentProps() { }

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::setupSubs() {
	using namespace mrpt_msgs;
	using namespace mrpt::utils;

	last_regd_nodes_sub = nh->subscribe<NodeIDWithPose_vec>(
			last_regd_nodes_topic,
			m_queue_size,
			&TNeighborAgentProps::updateNodes, this);
	last_regd_id_scan_sub = nh->subscribe<NodeIDWithLaserScan>(
			last_regd_id_scan_topic,
			m_queue_size,
			&TNeighborAgentProps::updateLastRegdIDScan, this);

	engine.logFmt(
			LVL_WARN, // TODO - change this
			"TNeighborAgentProps: Successfully set up subscribers "
			"to agent topics for namespace [%s].",
			agent.topic_namespace.data.c_str()
			);
	
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::updateNodes(
		const mrpt_msgs::NodeIDWithPose_vec::ConstPtr& nodes) {
	using namespace mrpt_msgs;
	using namespace mrpt_bridge;
	using namespace std;
	using namespace mrpt::utils;

	typedef typename GRAPH_T::constraint_t::type_value pose_t;

	for (NodeIDWithPose_vec::_vec_type::const_iterator
			n_it = nodes->vec.begin();
			n_it != nodes->vec.end();
			++n_it) {

		// insert in the set if not already there.
		TNodeID  nodeID = static_cast<TNodeID>(n_it->nodeID);
		std::pair<set<TNodeID>::iterator, bool> res =
			nodeIDs_set.insert(nodeID);

		// if I just inserted this node mark it as not used (in own graph)
		if (res.second) { // insertion took place.
			nodeID_to_is_integrated.insert(make_pair(n_it->nodeID, false));
		}

		// update the poses
		pose_t curr_pose;
		// note: use "operator[]" instead of "insert" so that if the key already
		// exists, the corresponding value is changed rather than ignored.
		poses[static_cast<TNodeID>(n_it->nodeID)] =
			convert(n_it->pose, curr_pose);

	}

	// mark this TNeighborAgentProps instance as updated so that I can take it in
	// account afterwards
	engine.m_neighbor_to_got_updated.at(*this) = true;

	//print nodeIDs just for verification
	//cout << "NodeIDs for topic namespace: " << agent.topic_namespace.data << endl;
	//mrpt::math::printSTLContainer(vector<TNodeID>(
				//nodeIDs_set.begin(), nodeIDs_set.end()));
	// print poses just for verification
	//cout << "Poses for topic namespace: " << agent.topic_namespace.data << endl;
	//for (typename GRAPH_T::global_poses_t::const_iterator
			//p_it = poses.begin();
			//p_it != poses.end();
			//++p_it) {
		//cout << "nodeID: " << p_it->first << " | pose: " << p_it->second << endl;
	//}

}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::updateLastRegdIDScan(
		const mrpt_msgs::NodeIDWithLaserScan::ConstPtr& last_regd_id_scan) {
	using namespace std;
	using namespace mrpt::utils;
	ASSERT_(last_regd_id_scan);

	//cout << "In updateLastRegdIDScan method." << endl;

	// make sure I haven't received any LaserScan for the current nodeID
	// TODO

	// Pose may not be available due to timing in publishing of the corresponding
	// topics. Just keep it in ROS form and then convert them to MRPT form when
	// needed.
	TNodeID curr_node = static_cast<TNodeID>(last_regd_id_scan->nodeID);
	ros_scans.push_back(*last_regd_id_scan);
}

template<class GRAPH_T>
void CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::getUnusedNodes(
		vector_uint* nodeIDs/*=NULL*/,
		std::map<
			mrpt::utils::TNodeID,
			TNodeProps>* nodes_params/*=NULL*/,
		bool only_unused/*=true*/) const {
	using namespace mrpt::obs;

	ASSERT_(nodeIDs || nodes_params);

	// traverse all nodes
	for (std::set<mrpt::utils::TNodeID>::const_iterator
			n_it = nodeIDs_set.begin();
			n_it != nodeIDs_set.end();
			++n_it) {

		// Should I return only the unused ones?
		if (only_unused && nodeID_to_is_integrated.at(*n_it)) { continue; }

		// add the nodeID
		if (nodeIDs) {
			nodeIDs->push_back(*n_it);
		}

		// add the node properties
		if (nodes_params) {
			std::pair<
				mrpt::utils::TNodeID,
				TNodeProps> params;
			const pose_t* p = &poses.at(*n_it);

			params.first = *n_it;
			params.second.pose = *p;
			CObservation2DRangeScanPtr mrpt_scan = CObservation2DRangeScan::Create();
			mrpt_bridge::convert(
					getLaserScanByNodeID(*n_it),
					*p,
					*mrpt_scan);
			params.second.scan = mrpt_scan;

			nodes_params->insert(params);
		}
	}
}

template<class GRAPH_T>
const sensor_msgs::LaserScan& CGraphSlamEngine_CM<GRAPH_T>::TNeighborAgentProps::
getLaserScanByNodeID(
	const mrpt::utils::TNodeID& nodeID) const {

	// assert that the current nodeID exists in the nodeIDs_set
	ASSERT_(nodeIDs_set.find(nodeID) != nodeIDs_set.end());

	for (std::vector<mrpt_msgs::NodeIDWithLaserScan>::const_iterator
			it = ros_scans.begin();
			it != ros_scans.end();
			++it) {
		if (it->nodeID == nodeID) {
			return it->scan;
		}
	}
	THROW_EXCEPTION(
			mrpt::format(
				"\nCan't find LaserScan of nodeID \"%lu\" in the ros_scans vector.\n",
				static_cast<unsigned long>(nodeID)));
}

} } // end of namespaces

#endif /* end of include guard: CGRAPHSLAMENGINE_CM_IMPL_H */
