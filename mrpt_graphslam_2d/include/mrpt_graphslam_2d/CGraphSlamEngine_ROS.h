#ifndef CGRAPHSLAMENGINE_ROS_H
#define CGRAPHSLAMENGINE_ROS_H

#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <ros/ros.h>
#include "mrpt_graphslam_2d/interfaces/CCondensedMeasurements.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

namespace mrpt { namespace graphslam { 

/**\brief Class template that provides a wrapper around the MRPT
 * CGraphSlamEngine class template and implements methods for interacting with
 * ROS.
 *
 * \note Class is specialized for executing the Condensed Measurements
 * multi-robot graphSLAM.
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
	~CGraphSlamEngine_ROS();

	void initCGraphSlamEngine_ROS();

	bool execGraphSlamStep(
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);
	bool execGraphSlamStep(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);

protected:
	ros::NodeHandle* m_nh;
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;


};

} } // end of namespaces

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS_impl.h"

#endif /* end of include guard: CGRAPHSLAMENGINE_ROS_H */
