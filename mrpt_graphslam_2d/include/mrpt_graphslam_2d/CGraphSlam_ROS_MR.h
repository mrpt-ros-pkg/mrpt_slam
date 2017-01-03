/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAM_ROS_MR_H
#define CGRAPHSLAM_ROS_MR_H

#include "mrpt_graphslam_2d/CGraphSlam_ROS.h"

class CGraphSlam_ROS_MR : public CGraphSlam_ROS
{
public:
	/**\brief type of graph constraints */
	typedef typename mrpt::graphs::CNetworkOfPoses2DInf::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename mrpt::graphs::CNetworkOfPoses2DInf::constraint_t::type_value pose_t;

	CGraphSlam_ROS_MR(
			mrpt::utils::COutputLogger* logger_in,
			ros::NodeHandle* nh_in
			);
	CGraphSlam_ROS_MR() { THROW_EXCEPTION("Not implemented."); }
	~CGraphSlam_ROS_MR();


private:
	/* data */
};



#endif /* end of include guard: CGRAPHSLAM_ROS_MR_H */
