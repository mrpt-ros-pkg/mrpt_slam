/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#include "mrpt_graphslam_2d/CMapMerger.h"

using namespace mrpt::graphslam;
using namespace mrpt::graphslam::detail;
using namespace mrpt_msgs;
using namespace mrpt::maps;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace ros;
using namespace nav_msgs;
using namespace std;

// helper methods

////////////////////////////////////////////////////////////

/**\return True if the object was actually removed. */
bool removeObjectFrom3DScene(
	mrpt::gui::CDisplayWindow3D* win, std::string obj_name = "")
{
	using namespace mrpt::opengl;
	ASSERT_(win);
	bool res = true;
	COpenGLScene::Ptr& scene = win->get3DSceneAndLock();

	if (obj_name.empty())
	{
		cout << "Clearing entire scene." << endl;
		scene.reset();
	}
	else
	{
		CRenderizable::Ptr obj = scene->getByName(obj_name);
		if (obj)
		{
			scene->removeObject(obj);
		}
		else
		{
			res = false;
		}
	}

	win->unlockAccess3DScene();
	win->forceRepaint();
	return res;
}

/**\return True if the object was actually removed. */
bool removeObjectFrom3DScene(
	mrpt::graphslam::CWindowManager* win_manager, std::string obj_name = "")
{
	ASSERT_(win_manager);
	return removeObjectFrom3DScene(win_manager->win, obj_name);
}

template <class RENDERIZABLE_OBJECT>
void addToWindow(
	mrpt::gui::CDisplayWindow3D* win, const RENDERIZABLE_OBJECT& o,
	const std::string& obj_name = "",
	const mrpt::poses::CPose3D& obj_pose = EMPTY_POSE)
{
	using namespace mrpt::opengl;

	COpenGLScene::Ptr& scene = win->get3DSceneAndLock();
	CSetOfObjects::Ptr obj = o.getVisualization();

	obj->setPose(obj_pose);

	if (!obj_name.empty())
	{
		obj->setName(obj_name);
	}

	scene->insert(obj);

	win->unlockAccess3DScene();
	win->forceRepaint();
}

void addAxis(mrpt::gui::CDisplayWindow3D* win)
{
	using namespace mrpt;
	using namespace mrpt::opengl;
	ASSERT_(win);

	COpenGLScene::Ptr& scene = win->get3DSceneAndLock();
	opengl::CAxis::Ptr obj =
		opengl::CAxis::Create(-6, -6, -6, 6, 6, 6, 2, 2, true);
	obj->setLocation(0, 0, 0);
	scene->insert(obj);
	win->unlockAccess3DScene();
	win->forceRepaint();
}

void addGrid(mrpt::gui::CDisplayWindow3D* win)
{
	using namespace mrpt;
	using namespace mrpt::opengl;
	ASSERT_(win);

	COpenGLScene::Ptr& scene = win->get3DSceneAndLock();
	opengl::CGridPlaneXY::Ptr obj =
		opengl::CGridPlaneXY::Create(-20, 20, -20, 20, 0, 1);
	obj->setColor(0.7, 0.7, 0.7);
	obj->setLocation(0, 0, 0);
	scene->insert(obj);

	win->unlockAccess3DScene();
	win->forceRepaint();
}

void addSupWidgets(mrpt::gui::CDisplayWindow3D* win)
{
	ASSERT_(win);

	addAxis(win);
	addGrid(win);
}

////////////////////////////////////////////////////////////

CMapMerger::CMapMerger(
	mrpt::system::COutputLogger* logger_in, ros::NodeHandle* nh_in)
	: m_logger(logger_in),
	  m_nh(nh_in),
	  m_conn_manager(logger_in, nh_in),
	  m_queue_size(1),
	  quit_keypress1("q"),
	  quit_keypress2("Ctrl+c"),
	  map_merge_keypress("n"),
	  save_map_merging_results(true)
{
	ASSERT_(m_nh);

	m_global_ns = "/map_merger";

	// GridMap Alignment options to be used in merging.
	m_alignment_options.methodSelection = CGridMapAligner::amModifiedRANSAC;
	////options.methodSelection = CGridMapAligner::amRobustMatch; // ASSERTION
	/// ERROR
	m_alignment_options.min_ICP_goodness = 0.60;
	m_alignment_options.maxKLd_for_merge = 0.90;
	m_alignment_options.ransac_minSetSizeRatio = 0.40;
	// m_alignment_options.loadFromConfigFileName(
	//"/home/bergercookie/mrpt/share/mrpt/config_files/grid-matching/gridmatch_example.ini",
	//"grid-match");
	m_alignment_options.dumpToConsole();

	// initialize CDisplayWindow for fused map
	m_fused_map_win_manager = this->initWindowVisuals();
	m_fused_map_win_manager->win->setWindowTitle("Fused map");
	this->monitorKeystrokes(m_fused_map_win_manager->observer);
}

void CMapMerger::monitorKeystrokes(
	mrpt::graphslam::CWindowObserver* win_observer)
{
	ASSERT_(win_observer);

	win_observer->registerKeystroke(quit_keypress1, "Finish execution");
	win_observer->registerKeystroke(
		map_merge_keypress, "Compute next available grid-merging");
}

CMapMerger::~CMapMerger()
{
	// delete the generated neighbors intances
	for (neighbors_t::iterator n_it = m_neighbors.begin();
		 n_it != m_neighbors.end(); ++n_it)
	{
		delete m_neighbors_to_windows.at(*n_it)->observer;
		delete m_neighbors_to_windows.at(*n_it)->win;
		delete m_neighbors_to_windows.at(*n_it);
		delete *n_it;
	}

	// delete fused window manager
	delete m_fused_map_win_manager->win;
	delete m_fused_map_win_manager->observer;
	delete m_fused_map_win_manager;

	m_logger->logFmt(LVL_WARN, "Exiting...");
}

bool CMapMerger::updateState()
{
	MRPT_START;

	// get the new GraphSlamAgents
	const GraphSlamAgents& nearby_slam_agents =
		m_conn_manager.getNearbySlamAgents();
	// m_logger->logFmt(LVL_DEBUG, "nearby_slam_agents size: %lu\n",
	// static_cast<unsigned long>(nearby_slam_agents.list.size()));

	for (GraphSlamAgents::_list_type::const_iterator it =
			 nearby_slam_agents.list.begin();
		 it != nearby_slam_agents.list.end(); ++it)
	{
		const GraphSlamAgent& gsa = *it;

		// Is the current GraphSlamAgent already registered?
		auto search = [gsa](const TNeighborAgentMapProps* neighbor) {
			return (neighbor->agent == gsa);
		};
		typename neighbors_t::iterator n_it =
			find_if(m_neighbors.begin(), m_neighbors.end(), search);

		if (n_it == m_neighbors.end())
		{  // current gsa not found, add it

			m_neighbors.push_back(
				new TNeighborAgentMapProps(m_logger, gsa, m_nh));
			TNeighborAgentMapProps* latest_neighbor = m_neighbors.back();
			latest_neighbor->setupComm();
			m_logger->logFmt(
				LVL_WARN,
				"Initialized NeighborAgentMapProps instance for agent %s...",
				latest_neighbor->agent.topic_namespace.data.c_str());

			// initialize the window
			mrpt::graphslam::CWindowManager* win_manager = initWindowVisuals();
			win_manager->win->setWindowTitle(
				latest_neighbor->agent.topic_namespace.data);
			m_neighbors_to_windows.insert(
				make_pair(latest_neighbor, win_manager));
			this->monitorKeystrokes(win_manager->observer);
		}
	}  // end for all fetched GraphSlamAgents

	// run through the open windows - Exit if instructed
	bool continue_exec = true;
	for (map<TNeighborAgentMapProps*, CWindowManager*>::const_iterator it =
			 m_neighbors_to_windows.begin();
		 it != m_neighbors_to_windows.end(); ++it)
	{
		CWindowManager* win_manager = it->second;

		std::map<std::string, bool> events_map;
		win_manager->observer->returnEventsStruct(&events_map);

		if (events_map.at(map_merge_keypress))
		{
			mergeMaps();
		}
		win_manager->win->forceRepaint();

		// continue or exit
		if (!win_manager->isOpen() || events_map.at(quit_keypress1) ||
			events_map.at(quit_keypress2))
		{
			continue_exec = false;
			break;
		}
	}

	// Fetch the events for the main (fused map) window
	if (continue_exec)
	{
		std::map<std::string, bool> events_map;
		m_fused_map_win_manager->observer->returnEventsStruct(&events_map);
		if (events_map.at(map_merge_keypress))
		{
			mergeMaps();
		}
		m_fused_map_win_manager->win->forceRepaint();
		continue_exec = m_fused_map_win_manager->isOpen() &&
						!events_map.at(quit_keypress1) &&
						!events_map.at(quit_keypress2);
	}

	return continue_exec;

	MRPT_END;
}  // end of updateState

void CMapMerger::mergeMaps()
{
	MRPT_START;
	m_logger->logFmt(LVL_WARN, "In mergeMaps.");

	CGridMapAligner gridmap_aligner;
	gridmap_aligner.options = m_alignment_options;

	// List of maps that is to be filled.
	maps_t mrpt_gridmaps;
	trajectories_t mrpt_trajectories;

	// traverse Neighbor instances - get their nav_msgs::OccupancyGrid maps,
	// trajectories
	for (neighbors_t::const_iterator n_it = m_neighbors.begin();
		 n_it != m_neighbors.end(); ++n_it)
	{
		TNeighborAgentMapProps& neighbor = **n_it;
		CWindowManager* neighbor_win_manager = m_neighbors_to_windows.at(*n_it);

		// reset visuals for each neighbor's window
		removeObjectFrom3DScene(neighbor_win_manager->win);
		addSupWidgets(neighbor_win_manager->win);

		if (neighbor.nav_map && neighbor.nav_robot_trajectory)
		{
			//
			// map
			//
			COccupancyGridMap2D::Ptr map = COccupancyGridMap2D::Create();
			m_logger->logFmt(
				LVL_INFO, "Adding map of agent \"%s\" to the stack",
				neighbor.agent.topic_namespace.data.c_str());
			mrpt::ros1bridge::fromROS(*neighbor.nav_map, *map);

			// visualize map in corresponding window
			addToWindow(neighbor_win_manager->win, *map);
			mrpt_gridmaps.insert(make_pair(*n_it, map));

			//
			// trajectory
			//
			CSetOfLines::Ptr curr_traj = CSetOfLines::Create();
			curr_traj->setPose(CPose3D(0, 0, 0.3, 0, 0, 0));
			curr_traj->setLineWidth(1.5);
			curr_traj->setColor(TColorf(0, 0, 1));
			// append a dummy line so that you can later use append using
			// CSetOfLines::appendLienStrip method.
			curr_traj->appendLine(
				/* 1st */ 0, 0, 0,
				/* 2nd */ 0, 0, 0);

			for (nav_msgs::Path::_poses_type::const_iterator pth_it =
					 neighbor.nav_robot_trajectory->poses.begin();
				 pth_it != neighbor.nav_robot_trajectory->poses.end(); ++pth_it)
			{
				curr_traj->appendLineStrip(
					pth_it->pose.position.x, pth_it->pose.position.y, 0);
			}
			// visualize trajectory
			{
				COpenGLScene::Ptr& scene =
					neighbor_win_manager->win->get3DSceneAndLock();
				scene->insert(curr_traj);

				neighbor_win_manager->win->unlockAccess3DScene();
				neighbor_win_manager->win->forceRepaint();
			}

			// cache trajectory so that I can later visualize it in the fused
			// map
			{
				// operate on copy of object - it is already inserted and used
				// in another window
				auto tmp = CSetOfLines::Ptr(
					dynamic_cast<CSetOfLines*>(curr_traj->clone()));
				auto curr_traj = mrpt::ptr_cast<CSetOfLines>::from(tmp);
				ASSERT_(curr_traj);
				mrpt_trajectories.insert(make_pair(&neighbor, curr_traj));
			}

		}  // end if both nav_map and nav_robot_trajectory exist
	}  // end for all neighbors traversal

	// join all gathered MRPT gridmaps
	if (mrpt_gridmaps.size() >= 2)
	{
		m_logger->logFmt(
			LVL_INFO, "Executing map merging for [%lu] gridmaps",
			static_cast<unsigned long>(mrpt_gridmaps.size()));
		int merge_counter = 0;

		// save results
		std::string output_dir_fname = "map_merger_results";
		if (save_map_merging_results)
		{
			m_logger->logFmt(
				LVL_INFO, "Saving map-merging results to \"%s\"",
				output_dir_fname.c_str());

			// mrpt::system::TTimeStamp cur_date(getCurrentTime());
			// string cur_date_str(dateTimeToString(cur_date));
			// string
			// cur_date_validstr(fileNameStripInvalidChars(cur_date_str));
			// std::string output_dir_fname = "map_merger_results_" +
			// cur_date_validstr;
			bool does_exist = directoryExists(output_dir_fname);
			if (does_exist)
			{
				deleteFilesInDirectory(output_dir_fname);
			}
			else
			{
				m_logger->logFmt(
					LVL_INFO, "Initializing gridmaps output directory.\n");
				createDirectory(output_dir_fname);
			}
		}

		// initialize final fused map
		COccupancyGridMap2D::Ptr fused_map = COccupancyGridMap2D::Create();
		fused_map->copyMapContentFrom(*mrpt_gridmaps.begin()->second);

		ASSERT_(fused_map);

		{
			// clear the fused map visuals
			COpenGLScene::Ptr& fused_scene =
				m_fused_map_win_manager->win->get3DSceneAndLock();
			fused_scene.reset();
			m_logger->logFmt(LVL_INFO, "Clearing the fused map visuals");

			addSupWidgets(m_fused_map_win_manager->win);

			// add first map
			CSetOfObjects::Ptr first_map_obj = fused_map->getVisualization();

			// each map in the fused display will have a different name - based
			// on the topic namespace
			first_map_obj->setName(format(
				"map_%s", mrpt_gridmaps.begin()
							  ->first->agent.topic_namespace.data.c_str()));
			fused_scene->insert(first_map_obj);

			m_fused_map_win_manager->win->unlockAccess3DScene();
			m_fused_map_win_manager->win->forceRepaint();
		}

		// save the first gridmap
		if (save_map_merging_results)
		{
			stringstream ss;
			ss << output_dir_fname << "/"
			   << "map" << merge_counter;
			(mrpt_gridmaps.begin()->second)
				->saveMetricMapRepresentationToFile(ss.str());
		}

		// value at which to display each new gridmap in the display window
		double off_z = 1;
		double off_z_step = 1;

		// map of TNeighborAgentMapProps* to a corresponding flag specifying
		// whether the COccupancyGridMap is correctly aligned can thus can be
		// used.
		neighbor_to_is_used_t neighbor_to_is_used;
		// first gridmap frame coincedes with the global frame - used anyway

		// map from TNeighborAgentMapProps* to a corresponding  relative pose
		// with regards to the global fused map
		neighbor_to_rel_pose_t neighbor_to_rel_pose;
		for (neighbors_t::const_iterator n_it = m_neighbors.begin();
			 n_it != m_neighbors.end(); ++n_it)
		{
			neighbor_to_is_used[*n_it] = false;
			neighbor_to_rel_pose[*n_it] = CPose2D();
		}

		// mark first as used - one trajectory should be there anyway
		neighbor_to_is_used[*m_neighbors.begin()] = true;

		// for all captured gridmaps - except the first
		for (maps_t::iterator m_it = std::next(mrpt_gridmaps.begin());
			 m_it != mrpt_gridmaps.end(); ++m_it, ++merge_counter)
		{
			TNeighborAgentMapProps* curr_neighbor = m_it->first;
			COccupancyGridMap2D* curr_gridmap = m_it->second.get();

			// run alignment procedure
			CGridMapAligner::TReturnInfo results;
			CPosePDFGaussian init_estim;
			m_logger->logFmt(
				LVL_INFO, "Trying to align the maps, initial estimation: %s",
				init_estim.mean.asString().c_str());

			TMetricMapAlignmentResult alignRes;

			const CPosePDF::Ptr pdf_tmp = gridmap_aligner.AlignPDF(
				fused_map.get(), curr_gridmap, init_estim, alignRes);

			const auto run_time = alignRes.executionTime;

			m_logger->logFmt(LVL_INFO, "Elapsed Time: %f", run_time);

			CPosePDFSOG::Ptr pdf_out = CPosePDFSOG::Create();
			pdf_out->copyFrom(*pdf_tmp);

			CPose2D pose_out;
			CMatrixDouble33 cov_out;
			pdf_out->getMostLikelyCovarianceAndMean(cov_out, pose_out);

			// dismiss this?
			if (results.goodness > 0.999 || isEssentiallyZero(pose_out))
			{
				continue;
			}

			neighbor_to_rel_pose.at(curr_neighbor) = pose_out;
			neighbor_to_is_used.at(curr_neighbor) = true;

			m_logger->logFmt(
				LVL_INFO, "%s\n",
				getGridMapAlignmentResultsAsString(*pdf_tmp, results).c_str());

			// save current gridmap
			if (save_map_merging_results)
			{
				stringstream ss;
				ss << output_dir_fname << "/"
				   << "map" << merge_counter;
				curr_gridmap->saveMetricMapRepresentationToFile(ss.str());
			}

			// save correspondences image
			if (save_map_merging_results)
			{
				stringstream ss;
				ss << output_dir_fname << "/"
				   << "fusing_proc_with"
				   << "_" << merge_counter;
				COccupancyGridMap2D::saveAsEMFTwoMapsWithCorrespondences(
					ss.str(), fused_map.get(), curr_gridmap,
					results.correspondences);
			}

			// Add current map to the fused map visualization
			{
				COpenGLScene::Ptr fused_scene =
					m_fused_map_win_manager->win->get3DSceneAndLock();

				// insert current map
				m_logger->logFmt(
					LVL_INFO,
					"Adding map to the fused map visualiztion using "
					"transformation %s",
					pose_out.asString().c_str());
				CSetOfObjects::Ptr curr_map_obj =
					curr_gridmap->getVisualization();

				curr_map_obj->setName(format(
					"map_%s",
					curr_neighbor->agent.topic_namespace.data.c_str()));
				curr_map_obj->setPose(pose_out + CPose3D(0, 0, off_z, 0, 0, 0));
				off_z += off_z_step;
				fused_scene->insert(curr_map_obj);

				m_fused_map_win_manager->win->unlockAccess3DScene();
				m_fused_map_win_manager->win->forceRepaint();
			}

			// TODO - merge current gridmap into the fused map by using the
			// found transformation

		}  // end for all gridmaps

		TColorManager traj_color_mngr;	// different color to each trajectory
		// Traverse and add all the trajectories to the fused map visualization
		for (trajectories_t::const_iterator it = mrpt_trajectories.begin();
			 it != mrpt_trajectories.end(); ++it)
		{
			TNeighborAgentMapProps* curr_neighbor = it->first;

			if (!neighbor_to_is_used.at(curr_neighbor))
			{
				m_logger->logFmt(
					LVL_WARN,
					"Skipping visualizing trajectory of agent %s in the fused "
					"map",
					curr_neighbor->agent.topic_namespace.data.c_str());
				continue;
			}

			CSetOfLines::Ptr curr_traj = it->second;
			m_logger->logFmt(
				LVL_WARN, "Adding #%lu lines...",
				static_cast<unsigned long>(curr_traj->getLineCount()));

			// set the pose of the trajectory
			CPose3D rel_pose(neighbor_to_rel_pose.at(curr_neighbor));
			// elevate by the last used offset
			rel_pose += CPose3D(0, 0, off_z, 0, 0, 0);

			curr_traj->setColor(traj_color_mngr.getNextTColorf());
			curr_traj->setPose(rel_pose);
			curr_traj->setName(format(
				"traj_%s", curr_neighbor->agent.topic_namespace.data.c_str()));
			{  // save 3D Scene
				COpenGLScene::Ptr fused_scene =
					m_fused_map_win_manager->win->get3DSceneAndLock();
				fused_scene->insert(curr_traj);

				m_fused_map_win_manager->win->unlockAccess3DScene();
				m_fused_map_win_manager->win->forceRepaint();
			}
		}

		{  // save the COpenGLScene

			COpenGLScene::Ptr fused_scene =
				m_fused_map_win_manager->win->get3DSceneAndLock();
			std::string fname = output_dir_fname + "/" + "output_scene.3DScene";
			fused_scene->saveToFile(fname);

			m_fused_map_win_manager->win->unlockAccess3DScene();
			m_fused_map_win_manager->win->forceRepaint();
		}

		// save final fused map
		// TODO

	}  // end if gridmap.size() >= 2
	MRPT_END;
}

CWindowManager* CMapMerger::initWindowVisuals()
{
	mrpt::graphslam::CWindowManager* win_manager = new CWindowManager();
	this->initWindowVisuals(win_manager);
	return win_manager;
}

void CMapMerger::initWindowVisuals(mrpt::graphslam::CWindowManager* win_manager)
{
	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::graphslam;
	ASSERT_(win_manager);

	CWindowObserver* win_observer = new CWindowObserver();
	CDisplayWindow3D* win =
		new CDisplayWindow3D("GraphSlam building procedure", 800, 600);
	win->setPos(400, 200);
	win_observer->observeBegin(*win);
	{
		COpenGLScene::Ptr& scene = win->get3DSceneAndLock();
		COpenGLViewport::Ptr main_view = scene->getViewport("main");
		win_observer->observeBegin(*main_view);
		win->unlockAccess3DScene();
	}

	// pass the window and the observer pointers to the CWindowManager instance
	win_manager->setCDisplayWindow3DPtr(win);
	win_manager->setWindowObserverPtr(win_observer);

	addSupWidgets(win_manager->win);
	m_logger->logFmt(LVL_DEBUG, "Initialized CDisplayWindow3D...");
}
