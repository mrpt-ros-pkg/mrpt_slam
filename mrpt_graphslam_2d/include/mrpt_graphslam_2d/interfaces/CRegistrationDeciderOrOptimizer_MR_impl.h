// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

// TODO TICKET-004: Multi-robot file — not yet ported to ROS 2.
#pragma once

namespace mrpt
{namespace graphslam
{

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::CRegistrationDeciderOrOptimizer_MR()
{
  this->is_mr_slam_class = true;
}

template<class GRAPH_T>
CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::~CRegistrationDeciderOrOptimizer_MR() {}

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::setCConnectionManagerPtr(
  mrpt::graphslam::detail::CConnectionManager * conn_manager)
{
  ASSERTMSG_(conn_manager, "\nInvalid CConnectionManager* pointer.\n");

  m_conn_manager = conn_manager;
  own_ns = m_conn_manager->getTrimmedNs();
}

template<class GRAPH_T>
void CRegistrationDeciderOrOptimizer_MR<GRAPH_T>::setCGraphSlamEnginePtr(
  const engine_t * engine)
{
  ASSERTMSG_(engine, "CGraphSlamEngine pointer is NULL");
  m_engine = engine;
}


}}  // end of namespaces
