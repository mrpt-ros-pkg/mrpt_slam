/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPCRITERIANRD_CM_IMPL_H
#define CICPCRITERIANRD_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CICPCriteriaNRD_CM<GRAPH_T>::CICPCriteriaNRD_CM() {
	this->initializeLoggers("CICPCriteriaNRD_CM");
}

} } } // end of namespaces

#endif /* end of include guard: CICPCRITERIANRD_CM_IMPL_H */
