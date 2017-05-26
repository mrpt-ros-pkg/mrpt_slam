#ifndef CREGISTRATIONDECIDEROROPTIMIZER_ROS_IMPL_H
#define CREGISTRATIONDECIDEROROPTIMIZER_ROS_IMPL_H

namespace mrpt { namespace graphslam {


template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::CRegistrationDeciderOrOptimizer_ROS() { }
template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::~CRegistrationDeciderOrOptimizer_ROS() { }

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::setNodeHandle(ros::NodeHandle* nh) {
	ASSERTMSG_(nh, "\nInvalid NodeHandle instance was provided.\n");

	m_nh = nh;
}


} } // end of namespaces

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_ROS_IMPL_H */
