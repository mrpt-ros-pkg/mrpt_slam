#ifndef CCONDENSEDMEASUREMENTS_IMPL_H
#define CCONDENSEDMEASUREMENTS_IMPL_H

namespace mrpt { namespace graphslam {

template<class GRAPH_t>
CCondensedMeasurements<GRAPH_t>::CCondensedMeasurements() { }

template<class GRAPH_t>
CCondensedMeasurements<GRAPH_t>::~CCondensedMeasurements() { }

template<class GRAPH_t>
void CCondensedMeasurements<GRAPH_t>::setCConnectionManagerPtr(
		mrpt::graphslam::detail::CConnectionManager* conn_manager) {
	ASSERTMSG_(conn_manager, "\nInvalid CConnectionManager* instance.\n");

	m_conn_manager = conn_manager;
}


} } // end of namespaces

#endif /* end of include guard: CCONDENSEDMEASUREMENTS_IMPL_H */
