#pragma once

namespace mrpt { namespace graphslam {

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::CRegistrationDeciderOrOptimizer_MR() {
	this->is_mr_slam_class = true;
}

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::~CRegistrationDeciderOrOptimizer_MR() { }

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::setCConnectionManagerPtr(
		mrpt::graphslam::detail::CConnectionManager* conn_manager) {
	ASSERTMSG_(conn_manager, "\nInvalid CConnectionManager* pointer.\n");

	m_conn_manager = conn_manager;
	own_ns = m_conn_manager->getTrimmedNs();
}

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::setCGraphSlamEnginePtr(
		const engine_t* engine) {
	ASSERTMSG_(engine, "CGraphSlamEngine pointer is NULL");
	m_engine = engine;
}



} } // end of namespaces

