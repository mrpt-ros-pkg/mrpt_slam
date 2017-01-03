#ifndef CCONDENSEDMEASUREMENTS_H
#define CCONDENSEDMEASUREMENTS_H

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

namespace mrpt { namespace graphslam {

/**\brief Interface for implementing deciders/optimizers related to the Condensed
 * Measurements multi-robot graphSLAM algorithm.
 */
template<class GRAPH_t>
class CCondensedMeasurements : public mrpt::graphslam::CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>
{
public:
	CCondensedMeasurements();
	~CCondensedMeasurements();

	virtual void setCConnectionManagerPtr(
			mrpt::graphslam::detail::CConnectionManager* conn_manager);

protected:
	/**\brief Pointer to the CConnectionManager instance
	 */
	mrpt::graphslam::detail::CConnectionManager* m_conn_manager;

};

} } // end of namespaces

// template methods implementations
#include "mrpt_graphslam_2d/interfaces/CCondensedMeasurements_impl.h"

#endif /* end of include guard: CCONDENSEDMEASUREMENTS_H */
