/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#pragma once

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS.h"

#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <ros/ros.h>

namespace mrpt { namespace graphslam {

/**\brief Class template that provides a wrapper around the MRPT
 * CGraphSlamEngine class template and implements methods for interacting with
 * ROS.
 *
  */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamEngine_ROS : public CGraphSlamEngine<GRAPH_t>
{
public:
	typedef CGraphSlamEngine<GRAPH_t> parent;

	CGraphSlamEngine_ROS(
			ros::NodeHandle* nh,
			const std::string& config_file,
			const std::string& rawlog_fname="",
			const std::string& fname_GT="",
			mrpt::graphslam::CWindowManager* win_manager=NULL,
			mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* node_reg=NULL,
			mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* edge_reg=NULL,
			mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* optimizer=NULL
			);
	virtual ~CGraphSlamEngine_ROS();
	/**\brief Wrapper method around the protected setup* class methods.
	 *
	 * Handy for setting up publishers, subscribers, services
	 * all at once.
	 *
	 */
	void setupComm();

	/**\brief Initialize object instance. */
	void initClass();
	ros::NodeHandle* m_nh;
protected:
	/**\brief Read the problem configuration parameters
	 *
	 * \sa readROSParameters, CGraphSlamEngine::loadParams
	 */
	void readParams();
	/**\brief Provide feedback about the SLAM operation
	 *
	 * Method makes the necessary calls to all the publishers of the class and
	 * broadcaster instances
	 */
	virtual void usePublishersBroadcasters();
	/**\brief Read configuration parameters from the ROS parameter server.
	 *
	 * \note Method is automatically called on object construction.
	 * \sa readParams, initClass
	 */
	void readROSParameters();
	virtual bool _execGraphSlamStep(
			mrpt::obs::CActionCollection::Ptr& action,
			mrpt::obs::CSensoryFrame::Ptr& observations,
			mrpt::obs::CObservation::Ptr& observation,
			size_t& rawlog_entry);

	/**\name setup* ROS-related methods
	 *\brief Methods for setting up topic subscribers, publishers, and
	 * corresponding services
	 *
	 * \sa setupComm
	 */
	/**\{*/
	virtual void setupSubs();
	virtual void setupPubs();
	virtual void setupSrvs();
	/**\}*/
	/**\brief Custom Callback queue for processing requests for the
	 * services outside the standard CallbackQueue.
	 *
	 * \note Logical thing would be to define it in CGraphSlamEngine_MR, but that
	 * results in an segfault with the following error message:
	 * ```
	 * pure virtual method called
	 * terminate called without an active exception
	 * ```
	 */
	ros::CallbackQueue custom_service_queue;


	int m_queue_size;
};

} } // end of namespaces

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS_impl.h"

