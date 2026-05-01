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

        // TODO TICKET-004: MR node/edge registration deciders not yet ported to ROS 2
        // this->node_regs_map["CICPCriteriaNRD_MR"] =
        //     parent::template createNodeRegistrationDecider<CICPCriteriaNRD_MR<GRAPH_T>>;
        // this->node_regs_map["CFixedIntervalsNRD_MR"] =
        //     parent::template createNodeRegistrationDecider<CFixedIntervalsNRD_MR<GRAPH_T>>;
        // this->edge_regs_map["CLoopCloserERD_MR"] =
        //     parent::template createEdgeRegistrationDecider<CLoopCloserERD_MR<GRAPH_T>>;

} // end of createDeciderOptimizerMappings

template<class GRAPH_T>
void TUserOptionsChecker_ROS<GRAPH_T>::populateDeciderOptimizerProperties()
{
  using namespace mrpt::graphslam::apps;
  using namespace std;

  parent::populateDeciderOptimizerProperties();

        // TODO TICKET-004: MR decider/optimizer properties not yet ported to ROS 2
        // CICPCriteriaNRD_MR, CFixedIntervalsNRD_MR, CLoopCloserERD_MR descriptions
}

}}}   //end namespaces
