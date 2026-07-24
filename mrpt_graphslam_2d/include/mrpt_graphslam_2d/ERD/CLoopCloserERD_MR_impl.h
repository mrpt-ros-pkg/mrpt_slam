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
{namespace deciders
{

// Ctors, Dtors
template<class GRAPH_T>
CLoopCloserERD_MR<GRAPH_T>::CLoopCloserERD_MR()
: lc_parent_t(),
  mr_parent_t()
{
  // A peer graph can add more than one node between successive updates.
  this->m_override_registered_nodes_check = true;

  this->initializeLoggers("CLoopCloserERD_MR");
}

template<class GRAPH_T>
void CLoopCloserERD_MR<GRAPH_T>::addBatchOfNodeIDsAndScans(
  const std::map<
    TNodeID,
    mrpt::obs::CObservation2DRangeScan::Ptr> & nodeIDs_to_scans2D)
{
  this->m_nodes_to_laser_scans2D.insert(
    nodeIDs_to_scans2D.begin(), nodeIDs_to_scans2D.end());
  this->m_last_total_num_nodes = this->m_graph->nodeCount();
}


}}}   // end of namespaces
