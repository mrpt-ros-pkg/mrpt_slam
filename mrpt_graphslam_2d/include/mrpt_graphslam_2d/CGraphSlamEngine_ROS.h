/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMENGINE_ROS_H
#define CGRAPHSLAMENGINE_ROS_H

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"

#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <ros/ros.h>

namespace mrpt { namespace graphslam { 

/**\brief Class template that provides a wrapper around the MRPT
 * CGraphSlamEngine class template and implements methods for interacting with
 * ROS.
 *
  */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamEngine_ROS : public CGraphSlamEngine<GRAPH_t>
{
public:
	typedef CGraphSlamEngine<GRAPH_t> parent;

	CGraphSlamEngine_ROS(
			ros::NodeHandle* nh,
			const std::string& config_file,
			const std::string& rawlog_fname="",
			const std::string& fname_GT="",
			mrpt::graphslam::CWindowManager* win_manager=NULL,
			mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg=NULL,
			mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg=NULL,
			mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer=NULL
			);
	virtual ~CGraphSlamEngine_ROS();

	void initClass();

protected:
	ros::NodeHandle* m_nh;


};

} } // end of namespaces

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS_impl.h"

#endif /* end of include guard: CGRAPHSLAMENGINE_ROS_H */
