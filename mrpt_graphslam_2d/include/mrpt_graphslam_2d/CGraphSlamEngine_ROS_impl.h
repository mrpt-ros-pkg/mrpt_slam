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
{

template<class GRAPH_t>
CGraphSlamEngine_ROS<GRAPH_t>::CGraphSlamEngine_ROS(
  rclcpp::Node * node,
  const std::string & config_file,
  const std::string & rawlog_fname/* ="" */,
  const std::string & fname_GT /* ="" */,
  mrpt::graphslam::CWindowManager * win_manager /* = NULL */,
  mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t> * node_reg /* = NULL */,
  mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t> * edge_reg /* = NULL */,
  mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t> * optimizer /* = NULL */)
:parent::CGraphSlamEngine(
    config_file,
    rawlog_fname,
    fname_GT,
    win_manager,
    node_reg,
    edge_reg,
    optimizer),
  m_node(node)
{
  this->initClass();
}

template<class GRAPH_t>
CGraphSlamEngine_ROS<GRAPH_t>::~CGraphSlamEngine_ROS()
{
  MRPT_LOG_DEBUG_STREAM("In Destructor: Deleting CGraphSlamEngine_ROS instance...");
}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::initClass()
{
  using namespace mrpt::graphslam;
  this->m_class_name = "CGraphSlamEngine_ROS";
  this->setLoggerName(this->m_class_name);

  m_queue_size = 10;

  this->setupComm();

        // in case of ROS specific deciders/optimizers (they inherit from the CDeciderOrOptimizer_ROS interface)
        // set the Node handle
  {
    CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> * dec_opt_ros =
      dynamic_cast<CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> *>(this->m_node_reg);

    if (dec_opt_ros) {
      dec_opt_ros->setNodeHandle(m_node);
    }
  }
  {
    CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> * dec_opt_ros =
      dynamic_cast<CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> *>(this->m_edge_reg);
    if (dec_opt_ros) {
      dec_opt_ros->setNodeHandle(m_node);
    }
  }
  {
    CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> * dec_opt_ros =
      dynamic_cast<CRegistrationDeciderOrOptimizer_ROS<GRAPH_t> *>(this->m_optimizer);
    if (dec_opt_ros) {
      dec_opt_ros->setNodeHandle(m_node);
    }
  }

  this->readParams();
}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::readParams()
{

  this->readROSParameters();
}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::readROSParameters() {}

template<class GRAPH_t>
bool CGraphSlamEngine_ROS<GRAPH_t>::_execGraphSlamStep(
  mrpt::obs::CActionCollection::Ptr & action,
  mrpt::obs::CSensoryFrame::Ptr & observations,
  mrpt::obs::CObservation::Ptr & observation,
  size_t & rawlog_entry)
{

  bool continue_exec = parent::_execGraphSlamStep(
                        action, observations, observation, rawlog_entry);
  this->usePublishersBroadcasters();

  return continue_exec;
}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::usePublishersBroadcasters() {}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::setupComm()
{
  MRPT_LOG_INFO_STREAM(
                "Setting up subscribers, publishers, services...");

        // setup subscribers, publishers, services...
  this->setupSubs();
  this->setupPubs();
  this->setupSrvs();

}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::setupSubs()
{
  MRPT_LOG_DEBUG_STREAM("Setting up subscribers...");

}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::setupPubs()
{
  MRPT_LOG_DEBUG_STREAM("Setting up publishers...");

}

template<class GRAPH_t>
void CGraphSlamEngine_ROS<GRAPH_t>::setupSrvs()
{
  MRPT_LOG_DEBUG_STREAM("Setting up services...");

}


}}  // end of namespaces
