/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace mrpt { namespace graphslam { namespace deciders {

// Ctors, Dtors
template<class GRAPH_T>
CLoopCloserERD_MR<GRAPH_T>::CLoopCloserERD_MR() {
	// CLoopCloser Ctor is automatically called.

	// since this is for MR-SLAM, we do expect more than one node registered
	// between successive calls
	this->m_override_registered_nodes_check = true;

	this->initializeLoggers("CLoopCloserERD_MR");
}

template<class GRAPH_T>
void CLoopCloserERD_MR<GRAPH_T>::addBatchOfNodeIDsAndScans(
		const std::map<
			TNodeID,
			mrpt::obs::CObservation2DRangeScan::Ptr>& nodeIDs_to_scans2D) {
	mr_parent_t::addBatchOfNodeIDsAndScans(nodeIDs_to_scans2D);

	this->updateMapPartitions(/*full update=*/ true,
			/* is_first_time_node_reg = */ false);

} // end of addBatchOfNodeIDsAndScans

template<class GRAPH_T>
void CLoopCloserERD_MR<GRAPH_T>::addScanMatchingEdges(
		TNodeID curr_nodeID) {
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
void CLoopCloserERD_MR<GRAPH_T>::fetchNodeIDsForScanMatching(
		const TNodeID& curr_nodeID,
		std::set<TNodeID>* nodes_set) {
	ASSERT_(nodes_set);

	size_t fetched_nodeIDs = 0;
	for (int nodeID_i = static_cast<int>(curr_nodeID)-1;
			(fetched_nodeIDs <= static_cast<size_t>(this->m_laser_params.prev_nodes_for_ICP) &&
			 nodeID_i >= 0);
			--nodeID_i) {
		if (this->m_engine->isOwnNodeID(nodeID_i)) {
			nodes_set->insert(nodeID_i);
			fetched_nodeIDs++;
		}
	}
}

// Member methods implementations
template<class GRAPH_T>
bool CLoopCloserERD_MR<GRAPH_T>::updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation ) {

	bool success = lc_parent_t::updateState(action, observations, observation);

	// search for possible edges with the other agent's graph.
	// TODO

	return success;
}


} } } // end of namespaces

