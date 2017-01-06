#ifndef CGRAPHSLAMENGINE_CM_H
#define CGRAPHSLAMENGINE_CM_H

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"
#include <std_msgs/String.h>

#include <mrpt_msgs/NetworkOfPoses.h>
#include <mrpt_msgs/GetCMGraph.h>

namespace mrpt { namespace graphslam {

/** \brief mrpt::graphslam::CGraphSlamEngine derived class for interacting
 * executing CondensedMeasurements multi-robot graphSLAM
 */
template<class GRAPH_t>
class CGraphSlamEngine_CM : public CGraphSlamEngine_ROS<GRAPH_t>
{
public:
	typedef CGraphSlamEngine_ROS<GRAPH_t> parent;

	CGraphSlamEngine_CM(
			ros::NodeHandle* nh,
			const std::string& config_file,
			const std::string& rawlog_fname="",
			const std::string& fname_GT="",
			mrpt::graphslam::CWindowManager* win_manager=NULL,
			mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg=NULL,
			mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg=NULL,
			mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer=NULL
			);

	~CGraphSlamEngine_CM();

	bool execGraphSlamStep(
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);
	bool execGraphSlamStep(
			mrpt::obs::CActionCollectionPtr& action,
			mrpt::obs::CSensoryFramePtr& observations,
			mrpt::obs::CObservationPtr& observation,
			size_t& rawlog_entry);

	void initClass();

private:
	void usePublishersBroadcasters();

	void setupSubs();
	void setupPubs();
	void setupSrvs();

	/**\brief Compute and fill the Condensed Measurements Graph
	 */
	bool getCMGraph(
			mrpt_msgs::GetCMGraph::Request& req,
			mrpt_msgs::GetCMGraph::Response& res);
	/**\brief ROS Topic namespaces related variables
	 */
	/**\{*/
	/**\brief Condensed Measurements topic namespace */
	std::string m_cm_ns;
	/**\}*/

	/**\name Subscribers - Publishers
	 *
	 * ROS Topic Subscriber/Publisher/Service instances
	 * */
	/**\{*/

	ros::Publisher m_list_neighbors_pub;

	ros::ServiceServer m_cm_graph_srvserver;
	/**\}*/

	/**\name Topic Names
	 *
	 * Names of the topics that the class instance subscribes or publishes to
	 */
	/**\{*/
	std::string m_list_neighbors_topic;
	/**\name of the server which is to be called when other agent wants to have a
	 * subgraph of certain nodes returned.
	 */
	std::string m_cm_graph_service;

	/**\}*/


	/**\brief CConnectionManager instance */
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;

};


} } // end of namespaces

// pseudo-split decleration from implementation
#include "mrpt_graphslam_2d/CGraphSlamEngine_CM_impl.h"

#endif /* end of include guard: CGRAPHSLAMENGINE_CM_H */
