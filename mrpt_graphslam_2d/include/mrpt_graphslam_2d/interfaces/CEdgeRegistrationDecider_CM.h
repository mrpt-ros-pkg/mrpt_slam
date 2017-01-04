/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CEDGEREGISTRATIONDECIDER_CM_H
#define CEDGEREGISTRATIONDECIDER_CM_H

#include "mrpt_graphslam_2d/interfaces/CRegistrationDeciderOrOptimizer_CM.h"
#include "mrpt_graphslam_2d/CConnectionManager.h"

#include <string>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Edge Registration Decider virtual method.
 *
 * \b Edge Registration Decider classes that are to be used in a multi-robot
 * SLAM scheme according to the Condensed Measurements multi-robot strategy by
 * M.T. Lazaro et al. [1] are to inherit from this method.
 *
 * \note Condensed Measurements-related classes are suffixed with _CM.
 *
 * \note For an example of inheriting from this class, see the
 * mrpt::graphslam::deciders::CLoopCloserERD_CM.
 *
 * [1] <a
 * href="http://webdiis.unizar.es/~mtlazaro/papers/Lazaro-IROS13.pdf">Multi-robot
 * SLAM using Condensed Measurements</a> - M.T. Lazaro, L.M. Paz, P. Pinies,
 * J.A. Castellanos, G. Grisetti
 */
template<class GRAPH_t>
class CEdgeRegistrationDecider_CM : public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer_CM<GRAPH_t>
{
public:
	CEdgeRegistrationDecider_CM ();
	~CEdgeRegistrationDecider_CM ();

protected:
};

} } } // end of namespaces

#include "CEdgeRegistrationDecider_CM_impl.h"
#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_CM_H */
