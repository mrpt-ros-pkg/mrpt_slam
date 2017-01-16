/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_CM_IMPL_H
#define CICPCRITERIANRD_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CICPCriteriaNRD_CM<GRAPH_T>::CICPCriteriaNRD_CM() {
	this->initializeLoggers("CICPCriteriaNRD_CM");
}

template<class GRAPH_T>
typename GRAPH_T::global_pose_t
CICPCriteriaNRD_CM<GRAPH_T>::addNodeAnnotsToPose(
		const global_pose_t& pose) const { }

template<>
typename mrpt::graphs::CNetworkOfPoses2DInf_NA::global_pose_t
CICPCriteriaNRD_CM<mrpt::graphs::CNetworkOfPoses2DInf_NA>::addNodeAnnotsToPose(
		const global_pose_t& pose) const {

	global_pose_t pose_out = pose;

	pose_out.agent_ID_str = this->own_ns;
	// Assumption: addNodeAnnotsToPose is going to be called right before the
	// actual registration.
	// Mark it with the nodeID that is up-next 
	pose_out.nodeID_loc = this->m_graph->nodeCount();

	return pose_out;
}

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIANRD_CM_IMPL_H */
