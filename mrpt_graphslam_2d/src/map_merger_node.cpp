/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include <rclcpp/rclcpp.hpp>
#include <mrpt/system/COutputLogger.h>
#include "mrpt_graphslam_2d/CMapMerger.h"

using namespace mrpt::system;
using namespace mrpt::graphslam;
using namespace std;

/**\brief Node that fetches the local maps produced by the graphSLAM agents and
 * joins them together using a RANSAC-based map-merging technique
 *
 * Node is to be used for inspecting the overall graphSLAM procedure and
 * present the user with a final version of all the independent maps after
 * merging.
 */
int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("map_merger");

	COutputLogger logger;
	logger.setLoggerName("map_merger");
	logger.setMinLoggingLevel(LVL_DEBUG);
	logger.logFmt(LVL_WARN, "Initialized map_merger node...\n");

	CMapMerger map_merger(&logger, node.get());

	rclcpp::Rate loop_rate(10);
	while (rclcpp::ok())
	{
		bool continue_exec = map_merger.updateState();
		if (!continue_exec)
		{
			break;
		}
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}
