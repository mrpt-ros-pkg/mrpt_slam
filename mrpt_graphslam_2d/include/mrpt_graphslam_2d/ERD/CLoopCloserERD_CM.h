/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLOOPCLOSERERD_CM_H
#define CLOOPCLOSERERD_CM_H

#include "mrpt_graphslam_2d/interfaces/CEdgeRegistrationDecider_CM.h"
#include <mrpt_bridge/laser_scan.h>
#include <mrpt/graphslam/ERD/CLoopCloserERD.h>
#include <mrpt_msgs/NodeIDWithLaserScan.h>

namespace mrpt { namespace graphslam { namespace deciders {

/** Loop Closer Edge Registration Decider class that can also be used in
 * multi-robot SLAM operations since it can utilize information from other
 * robot agents and correct own graph according to the strategy described in [1].
 *
 * \note Condensed Measurements-related classes are suffixed with _CM.
 *
 * [1] <a
 * href="http://webdiis.unizar.es/~mtlazaro/papers/Lazaro-IROS13.pdf">Multi-robot
 * SLAM using Condensed Measurements</a> - M.T. Lazaro, L.M. Paz, P. Pinies,
 * J.A. Castellanos, G. Grisetti
 */
template<class GRAPH_T>
class CLoopCloserERD_CM :
	public virtual CLoopCloserERD<GRAPH_T>,
	public virtual CEdgeRegistrationDecider_CM<GRAPH_T>
{
public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef CLoopCloserERD<GRAPH_T> lc_parent_t; /**< parent class */
	typedef CEdgeRegistrationDecider_CM<GRAPH_T> cm_parent_t; /**< parent class */
	typedef CLoopCloserERD_CM<GRAPH_T> decider_t; /**< handy self type */
	typedef typename lc_parent_t::constraint_t constraint_t;
	typedef typename lc_parent_t::pose_t pose_t;
	typedef typename lc_parent_t::range_ops_t range_ops_t;
	typedef typename lc_parent_t::partitions_t partitions_t;
	typedef typename lc_parent_t::nodes_to_scans2D_t nodes_to_scans2D_t;
	typedef mrpt::graphslam::CGraphSlamEngine_CM<GRAPH_T> engine_t;
	typedef typename GRAPH_T::global_pose_t global_pose_t;
	/**\}*/

	// Ctor, Dtor
	CLoopCloserERD_CM();
	~CLoopCloserERD_CM();

	// member implementations
	bool updateState(
			mrpt::obs::CActionCollectionPtr action,
			mrpt::obs::CSensoryFramePtr observations,
			mrpt::obs::CObservationPtr observation );
	void addBatchOfNodeIDsAndScans(
			const std::map<
				mrpt::utils::TNodeID,
				mrpt::obs::CObservation2DRangeScanPtr>& nodeIDs_to_scans2D);
	void addScanMatchingEdges(mrpt::utils::TNodeID curr_nodeID);

protected:

};

} } } // end of namespaces

#include "CLoopCloserERD_CM_impl.h"
#endif /* end of include guard: CLOOPCLOSERERD_CM_H */
