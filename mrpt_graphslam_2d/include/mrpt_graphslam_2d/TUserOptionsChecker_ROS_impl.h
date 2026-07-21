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
   +---------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{namespace graphslam
{namespace apps
{

template<class GRAPH_T>
TUserOptionsChecker_ROS<GRAPH_T>::TUserOptionsChecker_ROS()
{
}

template<class GRAPH_T>
TUserOptionsChecker_ROS<GRAPH_T>::~TUserOptionsChecker_ROS()
{
}

template<class GRAPH_T>
void TUserOptionsChecker_ROS<GRAPH_T>::createDeciderOptimizerMappings()
{
  using namespace std;
  using namespace mrpt::graphs;
  using namespace mrpt::graphslam::apps;
  using namespace mrpt::graphslam::deciders;
  parent::createDeciderOptimizerMappings();

#ifdef MRPT_GRAPHSLAM_MR_DECIDERS
  // MR node/edge registration deciders
  this->node_regs_map["CICPCriteriaNRD_MR"] =
    parent::template createNodeRegistrationDecider<CICPCriteriaNRD_MR<GRAPH_T>>;
  this->node_regs_map["CFixedIntervalsNRD_MR"] =
    parent::template createNodeRegistrationDecider<CFixedIntervalsNRD_MR<GRAPH_T>>;
  this->edge_regs_map["CLoopCloserERD_MR"] =
    parent::template createEdgeRegistrationDecider<CLoopCloserERD_MR<GRAPH_T>>;
#endif  // MRPT_GRAPHSLAM_MR_DECIDERS

} // end of createDeciderOptimizerMappings

template<class GRAPH_T>
void TUserOptionsChecker_ROS<GRAPH_T>::populateDeciderOptimizerProperties()
{
  using namespace mrpt::graphslam::apps;
  using namespace std;

  parent::populateDeciderOptimizerProperties();

#ifdef MRPT_GRAPHSLAM_MR_DECIDERS
  // Populate properties for MR deciders so they appear in dumpRegistrarsToConsole output
  {
    auto * dec = new TRegistrationDeciderProps;
    dec->name = "CICPCriteriaNRD_MR";
    dec->description =
      "Multi-robot variant of CICPCriteriaNRD: registers nodes using ICP "
      "and coordinates with peer agents";
    dec->type = "Node";
    dec->rawlog_format = "#2 - Observation-only";
    dec->is_mr_slam_class = true;
    dec->is_slam_2d = true;
    this->regs_descriptions.push_back(dec);
  }
  {
    auto * dec = new TRegistrationDeciderProps;
    dec->name = "CFixedIntervalsNRD_MR";
    dec->description =
      "Multi-robot variant of CFixedIntervalsNRD: registers nodes at "
      "fixed distance intervals and coordinates with peer agents";
    dec->type = "Node";
    dec->rawlog_format = "Both";
    dec->is_mr_slam_class = true;
    dec->is_slam_2d = true;
    this->regs_descriptions.push_back(dec);
  }
  {
    auto * dec = new TRegistrationDeciderProps;
    dec->name = "CLoopCloserERD_MR";
    dec->description =
      "Multi-robot loop-closer edge registration decider: performs "
      "scan-matching across peer robot graphs";
    dec->type = "Edge";
    dec->rawlog_format = "Both";
    dec->is_mr_slam_class = true;
    dec->is_slam_2d = true;
    this->regs_descriptions.push_back(dec);
  }
#endif  // MRPT_GRAPHSLAM_MR_DECIDERS
}

}}}   //end namespaces
