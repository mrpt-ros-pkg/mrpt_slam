/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <ros/console.h>

#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include "mrpt_graphslam_2d/NRD/CFixedIntervalsNRD_MR.h"
#include "mrpt_graphslam_2d/NRD/CICPCriteriaNRD_MR.h"
#include "mrpt_graphslam_2d/ERD/CLoopCloserERD_MR.h"

namespace mrpt { namespace graphslam { namespace apps {

template<class GRAPH_T>
struct TUserOptionsChecker_ROS:
	public mrpt::graphslam::apps::TUserOptionsChecker<GRAPH_T> {

	/**\name handy typedefs for the creation of deciders/optimzer instances from
	 * the corresponding strings
	 *
 	 * \note ROS-related classes are suffixed with _ROS
	 */
	/**\{*/
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>*(*)()>
			node_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>*(*)()>
			edge_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>*(*)()>
			optimizers_t;
	/**\brief Parent class */
	typedef mrpt::graphslam::apps::TUserOptionsChecker<GRAPH_T> parent;
	/**\}*/

	TUserOptionsChecker_ROS();
	~TUserOptionsChecker_ROS();
	/**\brief Create deciders, optimizers specific to the ROS case */
	void createDeciderOptimizerMappings();
	void populateDeciderOptimizerProperties();

};


} } } // end of namespaces
#include "mrpt_graphslam_2d/TUserOptionsChecker_ROS_impl.h"

