/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT) | |
   http://www.mrpt.org/                             | | | | Copyright (c)
   2005-2016, Individual contributors, see AUTHORS file        | | See:
   http://www.mrpt.org/Authors - All rights reserved.                   | |
   Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+
 */

#include <mrpt/math/utils.h>
#include "mrpt_graphslam_2d/misc/common.h"
#include <mrpt/containers/stl_containers_utils.h>

using namespace mrpt::containers;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;
using namespace mrpt::math;

std::string mrpt::graphslam::detail::getGridMapAlignmentResultsAsString(
	const mrpt::poses::CPosePDF& pdf,
	const mrpt::slam::CGridMapAligner::TReturnInfo& ret_info)
{
	CPosePDFSOG::Ptr pdf_out = CPosePDFSOG::Create();
	pdf_out->copyFrom(pdf);
	CPose2D pose_out;
	CMatrixDouble33 cov_out;
	pdf_out->getMostLikelyCovarianceAndMean(cov_out, pose_out);

	stringstream ss;
	ss << "--------------------" << endl;
	ss << "Results: " << endl
	   << "\tPDFPtr pose: " << pdf.getMeanVal() << endl
	   << "\t# Correspondences: " << ret_info.correspondences.size() << endl
	   << "\tAlignment goodness: " << ret_info.goodness << endl
	   << "\tModes size: " << pdf_out->size() << endl
	   << "\tModes: " << endl
	   << getSTLContainerAsString(pdf_out->getSOGModes())
	   << "\tMost likely pose: " << pose_out << endl
	   << "\tCorresponding covariance matrix: " << endl
	   << cov_out << endl;
	ss << "--------------------" << endl;
	return ss.str();
}

bool mrpt::graphslam::detail::isEssentiallyZero(const mrpt::poses::CPose2D& p)
{
	double epsilon = 0.001;
	return (
		approximatelyEqual(p.x(), 0.0, epsilon) &&	// all 0s
		approximatelyEqual(p.y(), 0.0, epsilon) &&
		approximatelyEqual(p.phi(), 0.0, epsilon));
}
