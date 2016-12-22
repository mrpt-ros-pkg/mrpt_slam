/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
r  +---------------------------------------------------------------------------+ */

#include "mrpt_graphslam_2d/TUserOptionsChecker_ROS.h"

using namespace mrpt::graphslam::detail;

TUserOptionsChecker_ROS::TUserOptionsChecker_ROS() {
}

TUserOptionsChecker_ROS::~TUserOptionsChecker_ROS() {
}

void TUserOptionsChecker_ROS::createDeciderOptimizerMappings() {
	MRPT_START;
	using namespace std;
	using namespace mrpt::graphs;
	using namespace mrpt::graphslam::detail;
	using namespace mrpt::graphslam::deciders;
	parent::createDeciderOptimizerMappings();

	// node registration deciders
	node_regs_map["CFixedIntervalsNRD_MR"] =
		&createNodeRegistrationDecider<CFixedIntervalsNRD<CNetworkOfPoses2DInf> >;
	node_regs_map["CICPCriteriaNRD_MR"] =
		&createNodeRegistrationDecider<CICPCriteriaNRD<CNetworkOfPoses2DInf> >;

	// edge registration deciders
	this->edge_regs_map["CLoopCloserERD_MR"] =
		this->createEdgeRegistrationDecider<CLoopCloserERD_MR<CNetworkOfPoses2DInf> >;

	// optimizers

	MRPT_END;
}

void TUserOptionsChecker_ROS::populateDeciderOptimizerProperties() {
	MRPT_START;
	using namespace mrpt::graphslam::detail;
	using namespace std;

	parent::populateDeciderOptimizerProperties();

	{ // CFixedIntervalsNRD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CFixedIntervalsNRD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CFixedIntervalsNRD class based on \"Condensed Measurements\"";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}
	{ // CICPCriteriaNRD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaNRD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CICPCriteriaNRD class based on \"Condensed Measurements\"";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}
	{ // CLoopCloserERD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CLoopCloserERD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CLoopCloserERD class based on \"Condensed Measurements\"";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}

	MRPT_END;
}
