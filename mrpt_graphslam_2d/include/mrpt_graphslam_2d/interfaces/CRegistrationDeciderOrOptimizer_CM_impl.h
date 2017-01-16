#ifndef CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H
#define CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H

namespace mrpt { namespace graphslam {

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_CM<GRAPH_T>::CRegistrationDeciderOrOptimizer_CM() { }

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_CM<GRAPH_T>::~CRegistrationDeciderOrOptimizer_CM() { }

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_CM<GRAPH_T>::setCConnectionManagerPtr(
		mrpt::graphslam::detail::CConnectionManager* conn_manager) {
	ASSERTMSG_(conn_manager, "\nInvalid CConnectionManager* pointer.\n");

	m_conn_manager = conn_manager;
	own_ns = m_conn_manager->getTrimmedNs();
}

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_CM<GRAPH_T>::setCGraphSlamEnginePtr(
		const engine_t* engine) {
	ASSERTMSG_(engine, "CGraphSlamEngine pointer is NULL");
	m_engine = engine;
}



} } // end of namespaces

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H */
