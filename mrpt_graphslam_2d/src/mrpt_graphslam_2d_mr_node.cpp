// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

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

// ROS 2 headers
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

/** Main function of the mrpt_graphslam condensed-measurements multi-robot application */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  COutputLogger logger;
  logger.setLoggerName("mrpt_graphslam_2d_mr");
  logger.logFmt(LVL_WARN, "Initializing mrpt_graphslam_2d_mr node...\n");

  try {
                // Initialization
    TUserOptionsChecker_ROS<CNetworkOfPoses2DInf_NA> options_checker;
    auto graphslam_node =
      std::make_shared<CGraphSlamHandler_ROS<CNetworkOfPoses2DInf_NA>>(
                                &logger, &options_checker);

    graphslam_node->readParams();
    graphslam_node->initEngine_MR();
    graphslam_node->setupComm();

                // overwrite default results directory with the node's namespace
    {
      std::string ns = graphslam_node->get_namespace();
                        // strip leading '/'
      const auto first = ns.find_first_not_of(" /");
      if (first != std::string::npos) {
        graphslam_node->setResultsDirName(
                                        std::string(ns.begin() + first, ns.end()));
      }
    }

                // print the parameters just for verification
    graphslam_node->printParams();

                // Multi-robot: MultiThreadedExecutor allows the reentrant callback
                // group used by getCMGraph service to run concurrently with the main
                // sensor-processing callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(graphslam_node);
    executor.spin();
  } catch (exception & e) {
    RCLCPP_ERROR(
                        rclcpp::get_logger("mrpt_graphslam_2d_mr"),
                        "Finished with a (known) exception!\n%s", e.what());
    return EXIT_FAILURE;
  } catch (...) {
    RCLCPP_ERROR(
                        rclcpp::get_logger("mrpt_graphslam_2d_mr"),
                        "Finished with an unknown exception!");
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
