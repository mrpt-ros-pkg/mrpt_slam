/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_MR.h"
#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Node Registration Decider Interface Class.
 *
 * \b Node Registration Decider classes that are to be used in a multi-robot
 * SLAM scheme according to the Condensed Measurements multi-robot strategy by
 * M.T. Lazaro et al. [1] are to inherit from this method.
 *
 * \note Condensed Measurements-related classes are suffixed with _MR.
 *
 * \note For an example of inheriting from this class, see the
 * mrpt::graphslam::deciders::CFixedIntervalsNRD_MR.
 *
 * [1] <a
 * href="http://webdiis.unizar.es/~mtlazaro/papers/Lazaro-IROS13.pdf">Multi-robot
 * SLAM using Condensed Measurements</a> - M.T. Lazaro, L.M. Paz, P. Pinies,
 * J.A. Castellanos, G. Grisetti
 */
template<class GRAPH_T>
class CNodeRegistrationDecider_MR :
	public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer_MR<GRAPH_T>,
	public virtual CNodeRegistrationDecider<GRAPH_T>
{
public:
	typedef typename GRAPH_T::global_pose_t global_pose_t;

	CNodeRegistrationDecider_MR ();
	~CNodeRegistrationDecider_MR ();
protected:
	/**\brief Decorate a pose according to the TMRSlamNodeAnnotation fields
	 *
	 * \note Do this only for the nodes that are initially registered in the graph by
	 * the current CGraphSlamEngine_t class. Nodes of other graphSLAM-agents that
	 * are to be integrated must have already filled these fields.
	 */
	void addNodeAnnotsToPose(global_pose_t* pose) const;
};

} } } // end of namespaces

#include "CNodeRegistrationDecider_MR_impl.h"
