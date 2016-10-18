/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// MRPT headers
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>

// ROS headers
#include "mrpt_graphslam_2d/CGraphSlamResources.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::supplementary;

using namespace std;

/** Main function of the mrpt_graphslam_application */
int main(int argc, char **argv)
{
	std::string node_name = "mrpt_graphslam_2d";

	COutputLogger logger;
	logger.setLoggerName(node_name);
	logger.logFmt(LVL_WARN,
			"Initializing mrpt_graphslam_node node...\n");

  ros::init(argc, argv, node_name);
	ros::NodeHandle nh("graphslam_engine");

	ros::Rate loop_rate(10);


	try {
		// CGraphSlamResources initialization
		CGraphSlamResources resources(&logger, &nh);
		resources.readParams();
		resources.setupCommunication();
		// print the parameters just for verification
		resources.printParams();

		bool cont_exec = true;
		while (ros::ok() && cont_exec) {
			cont_exec = resources.usePublishersBroadcasters();

			ros::spinOnce();
			loop_rate.sleep();
		}

		//
		// Postprocessing
		//

		resources.generateReport();

	}
	catch (exception& e) {
		ROS_ERROR_STREAM(
				"Finished with a (known) exception!"
				<< std::endl
				<< e.what()
				<< std::endl);
		mrpt::system::pause();
		return -1;
	}
	catch (...) {
		ROS_ERROR_STREAM(
				"Finished with a (unknown) exception!"
				<< std::endl);
		mrpt::system::pause();
		return -1;
	}


}

