#ifndef CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H
#define CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H

namespace mrpt { namespace graphslam {

template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_CM<GRAPH_t>::CRegistrationDeciderOrOptimizer_CM() { }

template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_CM<GRAPH_t>::~CRegistrationDeciderOrOptimizer_CM() { }

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer_CM<GRAPH_t>::setCConnectionManagerPtr(
		mrpt::graphslam::detail::CConnectionManager* conn_manager) {
	ASSERTMSG_(conn_manager, "\nInvalid CConnectionManager* instance.\n");

	m_conn_manager = conn_manager;
}


} } // end of namespaces

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_CM_IMPL_H */
