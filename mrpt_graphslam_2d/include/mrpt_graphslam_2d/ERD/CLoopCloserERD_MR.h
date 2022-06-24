/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#pragma once

#include "mrpt_graphslam_2d/interfaces/CEdgeRegistrationDecider_MR.h"
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/graphslam/ERD/CLoopCloserERD.h>
#include <mrpt_msgs/NodeIDWithLaserScan.h>

namespace mrpt
{
namespace graphslam
{
namespace deciders
{
/** Loop Closer Edge Registration Decider class that can also be used in
 * multi-robot SLAM operations since it can utilize information from other
 * robot agents and correct own graph according to the strategy described in
 * [1].
 *
 * \note Multi-robot-related classes are suffixed with _MR.
 */
template <class GRAPH_T>
class CLoopCloserERD_MR : public virtual CLoopCloserERD<GRAPH_T>,
						  public virtual CEdgeRegistrationDecider_MR<GRAPH_T>
{
   public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef CLoopCloserERD<GRAPH_T> lc_parent_t; /**< parent class */
	typedef CEdgeRegistrationDecider_MR<GRAPH_T>
		mr_parent_t; /**< parent class */
	typedef CLoopCloserERD_MR<GRAPH_T> decider_t; /**< handy self type */
	typedef typename lc_parent_t::constraint_t constraint_t;
	typedef typename lc_parent_t::pose_t pose_t;
	typedef typename lc_parent_t::range_ops_t range_ops_t;
	typedef typename lc_parent_t::partitions_t partitions_t;
	typedef typename lc_parent_t::nodes_to_scans2D_t nodes_to_scans2D_t;
	typedef mrpt::graphslam::CGraphSlamEngine_MR<GRAPH_T> engine_t;
	typedef typename GRAPH_T::global_pose_t global_pose_t;
	/**\}*/

	CLoopCloserERD_MR();

	// member implementations
	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation);
	void addBatchOfNodeIDsAndScans(
		const std::map<TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>&
			nodeIDs_to_scans2D);
	void addScanMatchingEdges(TNodeID curr_nodeID);
	void fetchNodeIDsForScanMatching(
		const TNodeID& curr_nodeID, std::set<TNodeID>* nodes_set);

   protected:
};

}  // namespace deciders
}  // namespace graphslam
}  // namespace mrpt

#include "CLoopCloserERD_MR_impl.h"
