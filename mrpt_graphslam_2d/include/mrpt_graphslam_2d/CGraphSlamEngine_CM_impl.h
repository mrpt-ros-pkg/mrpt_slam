#ifndef CGRAPHSLAMENGINE_CM_IMPL_H
#define CGRAPHSLAMENGINE_CM_IMPL_H

template<class GRAPH_t>
CGraphSlamEngine_CM<GRAPH_t>::CGraphSlamEngine_CM(
		ros::NodeHandle* nh,
		const std::string& config_file,
		const std::string& rawlog_fname/* ="" */,
		const std::string& fname_GT /* ="" */,
		mrpt::graphslam::CWindowManager* win_manager /* = NULL */,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg /* = NULL */,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg /* = NULL */,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer /* = NULL */):
	parent::CGraphSlamEngine_ROS(
			nh,
			config_file,
			rawlog_fname,
			fname_GT,
			win_manager,
			node_reg,
			edge_reg,
			optimizer),
	m_conn_manager(
			dynamic_cast<mrpt::utils::COutputLogger*>(this), nh)
{

	this->initClass();

}

template<class GRAPH_t>
CGraphSlamEngine_CM<GRAPH_t>::~CGraphSlamEngine_CM() {
}

template<class GRAPH_t>
bool CGraphSlamEngine_CM<GRAPH_t>::execGraphSlamStep(
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) { 
	parent::execGraphSlamStep(observation, rawlog_entry);
}

template<class GRAPH_t>
bool CGraphSlamEngine_CM<GRAPH_t>::execGraphSlamStep(
		mrpt::obs::CActionCollectionPtr& action,
		mrpt::obs::CSensoryFramePtr& observations,
		mrpt::obs::CObservationPtr& observation,
		size_t& rawlog_entry) { 

	// call parent method
	parent::execGraphSlamStep(
			action, observations, observation, rawlog_entry);

	// notify of the neighbors
}

template<class GRAPH_t>
void CGraphSlamEngine_CM<GRAPH_t>::initClass() {
	using namespace mrpt::graphslam;

	// in case of CondensedMeasurements specific deciders/optimizers (they
	// inherit from the CDeciderOrOptimizer_ROS interface) set the
	// CConnectionManager*
	// NOTE
	// It's not certain, even though we are running the Condensed measurements
	// algorithm that all these classes do inherit from the
	// CRegistrationDeciderOrOptimizer_CM base class. They might be more generic and not
	// care about the multi-robot nature of the algorithm (e.g. optimization
	// scheme)
	{
	 	CRegistrationDeciderOrOptimizer_CM<GRAPH_t>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_t>*>(this->m_node_reg);

	 	if (dec_opt_cm) {
	 	 	dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
	 	}
	}
	{
		CRegistrationDeciderOrOptimizer_CM<GRAPH_t>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_t>*>(this->m_edge_reg);
		if (dec_opt_cm) {
	 		dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
		}
	}
	{
		CRegistrationDeciderOrOptimizer_CM<GRAPH_t>* dec_opt_cm =
	 	 	dynamic_cast<CRegistrationDeciderOrOptimizer_CM<GRAPH_t>*>(this->m_optimizer);
		if (dec_opt_cm) {
	 		dec_opt_cm->setCConnectionManagerPtr(&m_conn_manager);
		}
	}
	
}

#endif /* end of include guard: CGRAPHSLAMENGINE_CM_IMPL_H */
