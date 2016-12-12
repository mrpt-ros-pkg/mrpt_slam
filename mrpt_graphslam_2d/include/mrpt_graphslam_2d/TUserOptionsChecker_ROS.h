/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef TUSEROPTIONSCHECKER_ROS_H
#define TUSEROPTIONSCHECKER_ROS_H

#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include "mrpt_graphslam_2d/ERD/CLoopCloserERD_MR.h"

namespace mrpt { namespace graphslam { namespace detail {

struct TUserOptionsChecker_ROS :
	public mrpt::graphslam::detail::TUserOptionsChecker {

	/**\name handy typedefs for the creation of deciders/optimzer instances from
	 * the corresponding strings
	 */
	/**\{*/
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> node_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> edge_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> optimizers_t;
	/**\brief Parent class */
	typedef mrpt::graphslam::detail::TUserOptionsChecker parent;
	/**\}*/

	TUserOptionsChecker_ROS();
	~TUserOptionsChecker_ROS();
	void createDeciderOptimizerMappings();
	void populateDeciderOptimizerProperties();

};


} } } // end of namespaces

#endif /* end of include guard: TUSEROPTIONSCHECKER_ROS_H */
