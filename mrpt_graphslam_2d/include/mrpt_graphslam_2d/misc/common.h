/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) | |
   http://www.mrpt.org/                             | | | | Copyright (c)
   2005-2016, Individual contributors, see AUTHORS file        | | See:
   http://www.mrpt.org/Authors - All rights reserved.                   | |
   Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+
 */

/**\file common.h
 *
 * Defined methods used in different parts of the mrpt_graphslam_2d application.
 */

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/CGridMapAligner.h>
#include <sstream>
#include <mrpt/containers/stl_containers_utils.h>

namespace mrpt
{
namespace graphslam
{
namespace detail
{
std::string getGridMapAlignmentResultsAsString(
	const mrpt::poses::CPosePDF& pdf,
	const mrpt::slam::CGridMapAligner::TReturnInfo& ret_info);

bool isEssentiallyZero(const mrpt::poses::CPose2D& p);

}  // namespace detail
}  // namespace graphslam
}  // namespace mrpt
