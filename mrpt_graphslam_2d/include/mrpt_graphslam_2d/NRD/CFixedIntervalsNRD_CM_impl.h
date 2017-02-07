/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CFIXEDINTERVALSNRD_CM_IMPL_H
#define CFIXEDINTERVALSNRD_CM_IMPL_H

namespace mrpt { namespace graphslam { namespace deciders {

template<class GRAPH_T>
CFixedIntervalsNRD_CM<GRAPH_T>::CFixedIntervalsNRD_CM() {
	this->initializeLoggers("CFixedIntervalsNRD_CM");
}

} } } // end of namespaces



#endif /* end of include guard: CFIXEDINTERVALSNRD_CM_IMPL_H */

