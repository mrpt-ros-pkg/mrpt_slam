// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#pragma once

namespace mrpt
{namespace graphslam
{

template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::CRegistrationDeciderOrOptimizer_ROS()
: m_node(nullptr) {}

template<class GRAPH_t>
CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::~CRegistrationDeciderOrOptimizer_ROS() {}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer_ROS<GRAPH_t>::setNodeHandle(rclcpp::Node * node)
{
  ASSERTMSG_(node, "\nInvalid rclcpp::Node instance was provided.\n");
  m_node = node;
}

}}  // end of namespaces
