/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLOOPCLOSERERD_CM_H
#define CLOOPCLOSERERD_CM_H

#include <mrpt/graphslam/ERD/CLoopCloserERD.h>
#include "mrpt_graphslam_2d/interfaces/CEdgeRegistrationDecider_CM.h"

namespace mrpt { namespace graphslam { namespace deciders {

/** Loop Closer Edge Registration Decider class that can also be used in
 * multi-robot SLAM operations since it can utilize information from other
 * robot agents and correct own graph according to the strategy described in [1].
 *
 * \note Condensed Measurements-related classes are suffixed with _CM.
 *
 * [1] <a
 * href="http://webdiis.unizar.es/~mtlazaro/papers/Lazaro-IROS13.pdf">Multi-robot
 * SLAM using Condensed Measurements</a> - M.T. Lazaro, L.M. Paz, P. Pinies,
 * J.A. Castellanos, G. Grisetti
 */
template<class GRAPH_t>
class CLoopCloserERD_CM :
	public CLoopCloserERD<GRAPH_t>,
	public CEdgeRegistrationDecider_CM<GRAPH_t>
{
public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef typename GRAPH_t::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename GRAPH_t::constraint_t::type_value pose_t;
	/**\brief Typedef for accessing methods of the
	 * RangeScanRegistrationDecider_t parent class.
	 */
	typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider<GRAPH_t>
		range_scanner_t;
	typedef CLoopCloserERD<GRAPH_t> parent; /**< parent class */
	typedef CLoopCloserERD_CM<GRAPH_t> decider_t; /**< self type - Handy typedef */
	/**\brief Typedef for referring to a list of partitions */
	typedef std::vector<mrpt::vector_uint> partitions_t;
	typedef std::map<mrpt::utils::TNodeID,
					mrpt::obs::CObservation2DRangeScanPtr> nodes_to_scans2D_t;
	typedef typename GRAPH_t::edges_map_t::const_iterator edges_citerator;
	typedef typename GRAPH_t::edges_map_t::iterator edges_iterator;
	typedef typename mrpt::graphslam::detail::TGraphSlamHypothesis<GRAPH_t> hypot_t;
	typedef std::vector<hypot_t> hypots_t;
	typedef std::vector<hypot_t*> hypotsp_t;
	typedef std::map< std::pair<hypot_t*, hypot_t*>, double > hypotsp_to_consist_t;
	typedef mrpt::graphslam::TUncertaintyPath<GRAPH_t> path_t;
	/**\}*/

	// Ctor, Dtor
	CLoopCloserERD_CM();
	~CLoopCloserERD_CM();

	// member implementations
	bool updateState(
			mrpt::obs::CActionCollectionPtr action,
			mrpt::obs::CSensoryFramePtr observations,
			mrpt::obs::CObservationPtr observation );


protected:

};

} } } // end of namespaces

#include "CLoopCloserERD_CM_impl.h"
#endif /* end of include guard: CLOOPCLOSERERD_CM_H */
