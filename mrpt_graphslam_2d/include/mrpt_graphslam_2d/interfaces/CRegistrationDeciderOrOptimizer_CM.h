#ifndef CREGISTRATIONDECIDEROROPTIMIZER_CM_H
#define CREGISTRATIONDECIDEROROPTIMIZER_CM_H

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

namespace mrpt { namespace graphslam {

/**\brief Interface for implementing deciders/optimizers related to the Condensed
 * Measurements multi-robot graphSLAM algorithm.
 *
 * \note Condensed Measurements-related classes are suffixed with _CM.
 *
 */
template<class GRAPH_t>
class CRegistrationDeciderOrOptimizer_CM : public mrpt::graphslam::CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>
{
public:
	CRegistrationDeciderOrOptimizer_CM();
	~CRegistrationDeciderOrOptimizer_CM();

	virtual void setCConnectionManagerPtr(
			mrpt::graphslam::detail::CConnectionManager* conn_manager);

protected:
	/**\brief Pointer to the CConnectionManager instance
	 */
	mrpt::graphslam::detail::CConnectionManager* m_conn_manager;

};

} } // end of namespaces

// template methods implementations
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM_impl.h"

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_CM_H */
