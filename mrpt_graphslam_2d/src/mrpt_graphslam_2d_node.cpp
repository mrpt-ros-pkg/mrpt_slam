/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

// MRPT headers
#include <mrpt/system/COutputLogger.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <mrpt/system/string_utils.h>

#include <cstdlib>
#include <cstring>

// ROS headers
#include "mrpt_graphslam_2d/CGraphSlamHandler_ROS.h"

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::apps;

using namespace std;

/** Main function of the mrpt_graphslam application */
int main(int argc, char** argv)
{
	std::string node_name = "mrpt_graphslam_2d";

	COutputLogger logger;
	logger.setLoggerName(node_name);
	logger.logFmt(LVL_WARN, "Initializing %s node...\n", node_name.c_str());

	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

	ros::Rate loop_rate(10);

	try
	{
		// Initialization
		TUserOptionsChecker_ROS<CNetworkOfPoses2DInf> options_checker;
		CGraphSlamHandler_ROS<CNetworkOfPoses2DInf> graphslam_handler(
			&logger, &options_checker, &nh);
		graphslam_handler.readParams();
		graphslam_handler.initEngine_ROS();
		graphslam_handler.setupComm();

		// print the parameters just for verification
		graphslam_handler.printParams();

		bool cont_exec = true;
		while (ros::ok() && cont_exec)
		{
			cont_exec = graphslam_handler.usePublishersBroadcasters();

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	catch (exception& e)
	{
		ROS_ERROR_STREAM(
			"Finished with a (known) exception!" << std::endl
												 << e.what() << std::endl);
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		ROS_ERROR_STREAM("Finished with a (unknown) exception!" << std::endl);
		mrpt::system::pause();
		return -1;
	}
}
