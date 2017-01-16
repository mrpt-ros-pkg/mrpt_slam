#ifndef CREGISTRATIONDECIDEROROPTIMIZER_CM_H
#define CREGISTRATIONDECIDEROROPTIMIZER_CM_H

#include "mrpt_graphslam_2d/CGraphSlamEngine_CM.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

// forward declaration
namespace mrpt { namespace graphslam {
template<class GRAPH_t> class CGraphSlamEngine_CM;
} }// end of namespaces

namespace mrpt { namespace graphslam {

/**\brief Interface for implementing deciders/optimizers related to the Condensed
 * Measurements multi-robot graphSLAM algorithm.
 *
 * \warning Beware that this class <b>does not</b> inherit from the
 * mrpt::graphslam::CNodeRegistrationDeciderOrOptimizer.
 *
 * \note Condensed Measurements-related classes are suffixed with _CM.
 *
 */
template<class GRAPH_T>
class CRegistrationDeciderOrOptimizer_CM :
	public mrpt::graphslam::CRegistrationDeciderOrOptimizer_ROS<GRAPH_T>
{
public:
	typedef CGraphSlamEngine_CM<GRAPH_T> engine_t;

	CRegistrationDeciderOrOptimizer_CM();
	~CRegistrationDeciderOrOptimizer_CM();

	void setCGraphSlamEnginePtr(const engine_t* engine);
	virtual void setCConnectionManagerPtr(
			mrpt::graphslam::detail::CConnectionManager* conn_manager);

protected:
	/**\brief Pointer to the CConnectionManager instance
	 */
	mrpt::graphslam::detail::CConnectionManager* m_conn_manager;
	/**\brief Constant pointer to the CGraphSlamEngine instance.
	 *
	 */
	const engine_t* m_engine;
	std::string own_ns;


};

} } // end of namespaces

// template methods implementations
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM_impl.h"

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_CM_H */
