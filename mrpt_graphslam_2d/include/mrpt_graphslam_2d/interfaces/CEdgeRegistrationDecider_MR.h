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

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_MR.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"
#include <mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h>
#include <string>
#include <mrpt/graphs/TNodeID.h>

using namespace mrpt::graphs;

namespace mrpt
{
namespace graphslam
{
namespace deciders
{
/**\brief Edge Registration Decider virtual method.
 *
 * \b Edge Registration Decider classes that are to be used in a multi-robot
 * SLAM scheme according to the Condensed Measurements multi-robot strategy by
 * M.T. Lazaro et al. [1] are to inherit from this method.
 *
 * \note Condensed Measurements-related classes are suffixed with _MR.
 *
 * \note For an example of inheriting from this class, see the
 * mrpt::graphslam::deciders::CLoopCloserERD_MR.
 *
 * [1] <a
 * href="http://webdiis.unizar.es/~mtlazaro/papers/Lazaro-IROS13.pdf">Multi-robot
 * SLAM using Condensed Measurements</a> - M.T. Lazaro, L.M. Paz, P. Pinies,
 * J.A. Castellanos, G. Grisetti
 */
template <class GRAPH_T>
class CEdgeRegistrationDecider_MR
	: public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer_MR<
		  GRAPH_T>,
	  public virtual mrpt::graphslam::deciders::
		  CRangeScanEdgeRegistrationDecider<GRAPH_T>
{
   public:
	CEdgeRegistrationDecider_MR();
	~CEdgeRegistrationDecider_MR();
	virtual void addBatchOfNodeIDsAndScans(
		const std::map<TNodeID, mrpt::obs::CObservation2DRangeScan::Ptr>&
			nodeIDs_to_scans2D);

   protected:
};

}  // namespace deciders
}  // namespace graphslam
}  // namespace mrpt

#include "CEdgeRegistrationDecider_MR_impl.h"
