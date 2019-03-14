/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

namespace mrpt { namespace graphslam { namespace optimizers {

/**\brief Interface for implementing graphSLAM optimizer classes specific to
 * the Condensed Measurements MR-SLAM case
 */
class CGraphSlamOptimizer_MR : 
	public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer_MR<GRAPH_T>
	public virtual CGraphSlamOptimizer<GRAPH_T>
{
	public:
	CGraphSlamOptimizer_MR ();
	~CGraphSlamOptimizer_MR ();

	private:


} } } // end of namespaces



