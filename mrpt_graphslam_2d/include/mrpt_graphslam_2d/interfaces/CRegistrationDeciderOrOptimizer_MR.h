#pragma once

#include "mrpt_graphslam_2d/CGraphSlamEngine_MR.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

// forward declaration
namespace mrpt { namespace graphslam {
template<class GRAPH_t> class CGraphSlamEngine_MR;
} }// end of namespaces

namespace mrpt { namespace graphslam {

/**\brief Interface for implementing deciders/optimizers related to the Condensed
 * Measurements multi-robot graphSLAM algorithm.
 *
 * \warning Beware that this class <b>does not</b> inherit from the
 * mrpt::graphslam::CNodeRegistrationDeciderOrOptimizer.
 *
 * \note Condensed Measurements-related classes are suffixed with _MR.
 *
 */
template<class GRAPH_T>
class CRegistrationDeciderOrOptimizer_MR :
	public mrpt::graphslam::CRegistrationDeciderOrOptimizer_ROS<GRAPH_T>
{
public:
	typedef CGraphSlamEngine_MR<GRAPH_T> engine_t;

	CRegistrationDeciderOrOptimizer_MR();
	~CRegistrationDeciderOrOptimizer_MR();

	void setCGraphSlamEnginePtr(const engine_t* engine);
	virtual void setCConnectionManagerPtr(
			mrpt::graphslam::detail::CConnectionManager* conn_manager);

protected:
	/**\brief Pointer to the CConnectionManager instance
	 */
	mrpt::graphslam::detail::CConnectionManager* m_conn_manager;
	/**\brief Constant pointer to the CGraphSlamEngine_MR instance.
	 */
	const engine_t* m_engine;
	std::string own_ns;


};

} } // end of namespaces

// template methods implementations
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_MR_impl.h"

