/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#pragma once

#include "mrpt_graphslam_2d/CConnectionManager.h"
#include "mrpt_graphslam_2d/TNeighborAgentMapProps.h"
#include "mrpt_graphslam_2d/misc/common.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <mrpt_msgs/GraphSlamAgents.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/math/utils.h>
#include <mrpt/graphslam/misc/CWindowManager.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/img/TColorManager.h>

#include <iterator>

const mrpt::poses::CPose3D EMPTY_POSE;

using namespace mrpt::system;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::obs;

namespace mrpt
{
namespace graphslam
{
/**\brief Class responsible of the execution of the map_merger_node.
 *
 * Instance of the graph asks of the generated maps of all the running
 * GraphSlamAgents computes a feasible map-merging, if any, and returns the
 * resulting global map to the user.
 */
class CMapMerger
{
   public:
	typedef std::map<TNeighborAgentMapProps*, COccupancyGridMap2D::Ptr> maps_t;
	typedef std::map<TNeighborAgentMapProps*, bool> neighbor_to_is_used_t;
	typedef std::map<TNeighborAgentMapProps*, mrpt::poses::CPose2D>
		neighbor_to_rel_pose_t;

	/**\brief Robot trajectory visual object type */
	typedef std::map<TNeighborAgentMapProps*, mrpt::opengl::CSetOfLines::Ptr>
		trajectories_t;
	typedef std::vector<TNeighborAgentMapProps*> neighbors_t;
	CMapMerger(mrpt::system::COutputLogger* logger_in, ros::NodeHandle* nh_in);
	~CMapMerger();
	void mergeMaps();
	/**\brief Query and fetch the list of new graphSLAM agents.
	 *
	 * \return True if execution is to continue normally.
	 */
	bool updateState();

   private:
	/**\brief Compact method for monitoring the given keystrokes for the given
	 * observer.
	 */
	void monitorKeystrokes(mrpt::graphslam::CWindowObserver* win_observer);
	void initWindowVisuals(mrpt::graphslam::CWindowManager* win_manager);
	mrpt::graphslam::CWindowManager* initWindowVisuals();

	/**\brief CConnectionManager instance for fetching the running graphSLAM
	 * agents
	 */
	neighbors_t m_neighbors;
	std::map<TNeighborAgentMapProps*, CWindowManager*> m_neighbors_to_windows;
	mrpt::system::COutputLogger* m_logger;
	ros::NodeHandle* m_nh;
	mrpt::graphslam::detail::CConnectionManager m_conn_manager;

	/**\brief Topic namespace under which current node is going to be
	 * publishing.
	 */
	std::string m_global_ns;
	/**\brief Topic namespace under which, options that are used during the map
	 * alignment procedure are fetched from
	 */
	std::string m_options_ns;
	std::string m_feedback_ns;
	size_t m_queue_size;
	mrpt::slam::CGridMapAligner::TConfigParams m_alignment_options;

	std::string quit_keypress1;
	std::string quit_keypress2;
	std::string map_merge_keypress;

	bool save_map_merging_results;

	mrpt::graphslam::CWindowManager* m_fused_map_win_manager;

};	// end of CMapMerger

}  // namespace graphslam
}  // namespace mrpt
