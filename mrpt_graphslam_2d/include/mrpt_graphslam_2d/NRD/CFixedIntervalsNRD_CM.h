/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CFIXEDINTERVALSNRD_CM_H
#define CFIXEDINTERVALSNRD_CM_H

#include "mrpt_graphslam_2d/interfaces/CNodeRegistrationDecider_CM.h"
#include <mrpt/graphslam/NRD/CICPCriteriaNRD.h>

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
class CFixedIntervalsNRD_CM :
	public virtual CFixedIntervalsNRD<GRAPH_T>,
	public virtual CNodeRegistrationDecider_CM<GRAPH_T>
{
	public:
		typedef CNodeRegistrationDecider_CM<GRAPH_T> parent_cm;
		typedef CFixedIntervalsNRD<GRAPH_T> parent_mrpt;
		typedef typename GRAPH_T::global_pose_t global_pose_t;

		CFixedIntervalsNRD_CM();

	private:


};

} } } // end of namespaces

#include "CFixedIntervalsNRD_CM_impl.h"


#endif /* end of include guard: CFIXEDINTERVALSNRD_CM_H */

