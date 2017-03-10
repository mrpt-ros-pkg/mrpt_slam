/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLOOPCLOSERERD_CM_IMPL_H
#define CLOOPCLOSERERD_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
template<class GRAPH_T>
CLoopCloserERD_CM<GRAPH_T>::CLoopCloserERD_CM() {
	// CLoopCloser Ctor is automatically called.

	this->is_mr_slam_class = true;

	// since this is for MR-SLAM, we do expect more than one node registered
	// between successive calls
	this->m_override_registered_nodes_check = true;

	this->initializeLoggers("CLoopCloserERD_CM");
}

template<class GRAPH_T>
void CLoopCloserERD_CM<GRAPH_T>::addBatchOfNodeIDsAndScans(
		const std::map<
			mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr>& nodeIDs_to_scans2D) {
	cm_parent_t::addBatchOfNodeIDsAndScans(nodeIDs_to_scans2D);

	// update the last known number of nodeIDs
	this->m_last_total_num_nodes = this->m_graph->nodeCount();

	this->updateMapPartitions(/*full update=*/ true,
			/* is_first_time_node_reg = */ false);
	
} // end of addBatchOfNodeIDsAndScans

template<class GRAPH_T>
void CLoopCloserERD_CM<GRAPH_T>::addScanMatchingEdges(
		mrpt::utils::TNodeID curr_nodeID) {
	MRPT_START;

	// Do scan-matching only if I have initially registered curr_nodeID in the
	// graph.
	bool is_own = this->m_engine->isOwnNodeID(curr_nodeID);
	if (is_own) {
		lc_parent_t::addScanMatchingEdges(curr_nodeID);
	}

	MRPT_END;
}

template<class GRAPH_T>
void CLoopCloserERD_CM<GRAPH_T>::fetchNodeIDsForScanMatching(
		const mrpt::utils::TNodeID& curr_nodeID,
		std::set<mrpt::utils::TNodeID>* nodes_set) {
	ASSERT_(nodes_set);

	size_t fetched_nodeIDs = 0;
	for (int nodeID_i = static_cast<int>(curr_nodeID)-1;
			((fetched_nodeIDs <= this->m_laser_params.prev_nodes_for_ICP) &&
			 (nodeID_i >= 0)); 
			--nodeID_i) {
		bool is_own = this->m_engine->isOwnNodeID(nodeID_i);
		if (is_own) {
			nodes_set->insert(nodeID_i);
			fetched_nodeIDs++;
		}
	}
}
		

template<class GRAPH_T>
CLoopCloserERD_CM<GRAPH_T>::~CLoopCloserERD_CM() {
	// CLoopCloser Dtor is automatically called.
}

// Member methods implementations
template<class GRAPH_T>
bool CLoopCloserERD_CM<GRAPH_T>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {

	lc_parent_t::updateState(action, observations, observation);

	// search for possible edges with the other agent's graph.
	// TODO

}


} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_CM_IMPL_H */
