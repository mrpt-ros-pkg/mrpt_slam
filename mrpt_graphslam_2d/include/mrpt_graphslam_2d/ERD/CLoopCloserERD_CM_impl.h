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

template<class GRAPH_T>
bool CLoopCloserERD_CM<GRAPH_T>::queryEngineForScan(
		const global_pose_t& p,
		mrpt::obs::CObservation2DRangeScanPtr mrpt_scan) {
	THROW_EXCEPTION("Generic template method is not implemented.");
}

template<>
bool CLoopCloserERD_CM<mrpt::graphs::CNetworkOfPoses2DInf_NA>::queryEngineForScan(
		const global_pose_t& p,
		mrpt::obs::CObservation2DRangeScanPtr mrpt_scan) {
	using namespace mrpt;
	using namespace std;
	using namespace mrpt::graphslam;
	using namespace mrpt_msgs;
	using namespace mrpt_bridge;
	using namespace mrpt::poses;

	ASSERTMSG_(this->m_engine,
			"CGraphSlamEngine_CM instance doesn't contain a valid pointer. Exiting.");
	ASSERTMSG_(mrpt_scan,
			"\nCObservation2DRangeScanPtr provided does not contain a valid object. Exiting.\n");

	// find the TNeighborAgentProps instance
	auto agent_search = [p](const engine_t::TNeighborAgentProps& neighbor) {
		return (neighbor.agent.topic_namespace.data == p.agent_ID_str);
	};
	std::vector<typename engine_t::TNeighborAgentProps>::const_iterator
		neighbor_it = find_if(
			m_engine->getVecOfNeighborAgentProps().begin(),
			m_engine->getVecOfNeighborAgentProps().end(), agent_search);

	ASSERTMSG_(neighbor_it != m_engine->getVecOfNeighborAgentProps().end(),
			format(
				"Could not find CGraphSlamEngine::NeighborAgentProps instance"
				"with GraphSlamAgent.agent_ID_str \"%s\"", p.agent_ID_str.c_str()));

	// find the ros LaserScan specific to the nodeID that I am looking for
	const std::vector<NodeIDWithLaserScan>& ros_scans =
		neighbor_it->ros_scans;
	auto scan_search = [p](const NodeIDWithLaserScan& nodeID_with_scan) {
		return nodeID_with_scan.nodeID == p.nodeID_loc;
	};
	std::vector<NodeIDWithLaserScan>::const_iterator
		nodeID_with_scan_it = find_if(
			ros_scans.begin(),
			ros_scans.end(), scan_search);

	bool found_scan = nodeID_with_scan_it != ros_scans.end();
	if (!found_scan) {
		return false;
	}

	// fill up the mrpt_scan pointer
	convert(nodeID_with_scan_it->scan, CPose3D(p), *mrpt_scan);
	return true;
}

} } } // end of namespaces

#endif /* end of include guard: CLOOPCLOSERERD_CM_IMPL_H */
