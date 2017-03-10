/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEDGEREGISTRATIONDECIDER_CM_IMPL_H
#define CEDGEREGISTRATIONDECIDER_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CEdgeRegistrationDecider_CM<GRAPH_T>::CEdgeRegistrationDecider_CM() {}

template<class GRAPH_T>
CEdgeRegistrationDecider_CM<GRAPH_T>::~CEdgeRegistrationDecider_CM() {}

template<class GRAPH_T>
void CEdgeRegistrationDecider_CM<GRAPH_T>::addBatchOfNodeIDsAndScans(
		const std::map<
			mrpt::utils::TNodeID,
			mrpt::obs::CObservation2DRangeScanPtr>& nodeIDs_to_scans2D) {

	this->m_nodes_to_laser_scans2D.insert(
			nodeIDs_to_scans2D.begin(),
			nodeIDs_to_scans2D.end());

	this->m_last_total_num_nodes = this->m_graph->nodeCount();

} // end of addBatchOfNodeIDsAndScans


} } } // end of namespaces

#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_CM_IMPL_H */
