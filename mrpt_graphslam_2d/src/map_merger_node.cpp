/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <mrpt_msgs/GraphSlamAgents.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros1bridge/map.h>
#include "mrpt_graphslam_2d/CMapMerger.h"
#include "mrpt_graphslam_2d/TNeighborAgentMapProps.h"

using namespace mrpt::graphslam;
using namespace mrpt::graphslam::detail;
using namespace mrpt_msgs;
using namespace mrpt::maps;
using namespace ros;
using namespace nav_msgs;
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
	// init ROS Node
	std::string node_name = "map_merger";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// initialize logger.
	COutputLogger logger;
	logger.setLoggerName(node_name);
	logger.setMinLoggingLevel(LVL_DEBUG);
	logger.logFmt(LVL_WARN, "Initialized %s node...\n", node_name.c_str());

	CMapMerger map_merger(&logger, &nh);

	bool continue_exec = true;
	while (ros::ok() && continue_exec)
	{
		continue_exec = map_merger.updateState();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
