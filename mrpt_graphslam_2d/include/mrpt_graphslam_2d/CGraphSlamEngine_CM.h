#ifndef CGRAPHSLAMENGINE_CM_H
#define CGRAPHSLAMENGINE_CM_H

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

namespace mrpt { namespace graphslam {

/** \brief mrpt::graphslam::CGraphSlamEngine derived class for interacting
 * executing CondensedMeasurements multi-robot graphSLAM
 */
template<class GRAPH_t>
class CGraphSlamEngine_CM : public CGraphSlamEngine_ROS<GRAPH_t>
{
public:
	typedef CGraphSlamEngine_ROS<GRAPH_t> parent;

	CGraphSlamEngine_CM(
			ros::NodeHandle* nh,
			const std::string& config_file,
			const std::string& rawlog_fname="",
			const std::string& fname_GT="",
			mrpt::graphslam::CWindowManager* win_manager=NULL,
			mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg=NULL,
			mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg=NULL,
			mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer=NULL
			);

	~CGraphSlamEngine_CM();

	virtual bool execGraphSlamStep(
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);
	virtual bool execGraphSlamStep(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);

	void initClass();

private:
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;
};


} } // end of namespaces

// pseudo-split decleration from implementation
#include "mrpt_graphslam_2d/CGraphSlamEngine_CM_impl.h"

#endif /* end of include guard: CGRAPHSLAMENGINE_CM_H */
