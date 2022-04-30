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

/** Main function of the mrpt_graphslam condensed-measurements _application */
int main(int argc, char** argv)
{
	COutputLogger logger;

	try
	{
		std::string node_name = "mrpt_graphslam_2d_mr";

		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;

		node_name = node_name + nh.getNamespace();
		logger.setLoggerName(node_name);
		logger.logFmt(LVL_WARN, "Initialized %s node...\n", node_name.c_str());

		ros::Rate loop_rate(10);

		// Initialization
		TUserOptionsChecker_ROS<CNetworkOfPoses2DInf_NA> options_checker;
		CGraphSlamHandler_ROS<CNetworkOfPoses2DInf_NA> graphslam_handler(
			&logger, &options_checker, &nh);
		graphslam_handler.readParams();
		graphslam_handler.initEngine_MR();
		graphslam_handler.setupComm();

		std::string ns = nh.getNamespace();
		// overwite default results directory due to the multi-robot nature
		graphslam_handler.setResultsDirName(
			std::string(ns.begin() + 2, ns.end()));

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
		cout << "Known error!" << endl;
		logger.logFmt(LVL_ERROR, "Caught exception: %s", e.what());
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		cout << "Unknown error!" << endl;
		logger.logFmt(LVL_ERROR, "Finished with unknown exception. Exiting\n.");
		mrpt::system::pause();
		return -1;
	}
}
