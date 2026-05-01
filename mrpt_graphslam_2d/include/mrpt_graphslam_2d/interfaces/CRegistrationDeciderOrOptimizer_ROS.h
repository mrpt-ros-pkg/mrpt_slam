// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mrpt/system/COutputLogger.h>
#include <mrpt/graphslam/interfaces/CRegistrationDeciderOrOptimizer.h>

namespace mrpt
{namespace graphslam
{

/**\brief Interface class that all ROS-specific deciders/optimizers can inherit
 * from.
 *
 * \note ROS-related classes are suffixed with _ROS
 */
template<class GRAPH_t = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CRegistrationDeciderOrOptimizer_ROS
  : public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t>
{
public:
  CRegistrationDeciderOrOptimizer_ROS();
  virtual ~CRegistrationDeciderOrOptimizer_ROS();

  virtual void setNodeHandle(rclcpp::Node * node);

protected:
        /**\brief Pointer to the rclcpp::Node instance */
  rclcpp::Node * m_node;
};

}}  // end of namespaces

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_ROS_impl.h"
