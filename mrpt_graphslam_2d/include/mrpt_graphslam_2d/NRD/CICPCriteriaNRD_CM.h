/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_CM_H
#define CICPCRITERIANRD_CM_H

#include "mrpt_graphslam_2d/interfaces/CNodeRegistrationDecider_CM.h"
#include <mrpt/graphslam/NRD/CICPCriteriaNRD.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
class CICPCriteriaNRD_CM :
	public virtual CICPCriteriaNRD<GRAPH_T>,
	public virtual CNodeRegistrationDecider_CM<GRAPH_T>
{
	public:
		typedef CNodeRegistrationDecider_CM<GRAPH_T> parent_cm;
		typedef CICPCriteriaNRD<GRAPH_T> parent_mrpt;
		typedef typename GRAPH_T::global_pose_t global_pose_t;

		CICPCriteriaNRD_CM();


	private:

	/**\brief Decorate the class according to the TMRSlamNodeAnnotation  fields
	 *
	 * \note Do this only for the nodes that are initially registered in the graph by
	 * the current CGraphSlamEngine_t class. Nodes of other graphSLAM-agents that
	 * are to be integrated must have already filled these fields.
	 */
	global_pose_t addNodeAnnotsToPose(const global_pose_t& pose) const;


};

} } } // end of namespaces

#include "CICPCriteriaNRD_CM_impl.h"

#endif /* end of include guard: CICPCRITERIANRD_CM_H */
