/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_MR_IMPL_H
#define CICPCRITERIANRD_MR_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CICPCriteriaNRD_MR<GRAPH_T>::CICPCriteriaNRD_MR() {
	this->initializeLoggers("CICPCriteriaNRD_MR");
}

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIANRD_MR_IMPL_H */
