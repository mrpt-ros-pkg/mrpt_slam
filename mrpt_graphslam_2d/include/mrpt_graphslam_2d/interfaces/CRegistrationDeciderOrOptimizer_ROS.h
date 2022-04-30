#pragma once

#include <ros/ros.h>

#include <mrpt/system/COutputLogger.h>
#include <mrpt/graphslam/interfaces/CRegistrationDeciderOrOptimizer.h>

namespace mrpt { namespace graphslam {

/**\brief Interface class that all ROS-specific deciders/optimizers can inherit
 * from.
 *
 * \note ROS-related classes are suffixed with _ROS
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CRegistrationDeciderOrOptimizer_ROS :
	public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t>
{
public:
	CRegistrationDeciderOrOptimizer_ROS();
	virtual ~CRegistrationDeciderOrOptimizer_ROS();

	virtual void setNodeHandle(ros::NodeHandle* nh);

protected:
	/**\brief NodeHandle instance
	 */
	ros::NodeHandle* m_nh;


};


} } // end of namespaces

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS_impl.h"

