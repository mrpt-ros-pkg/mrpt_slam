/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#include "mrpt_graphslam_2d/misc/common.h"

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;
using namespace mrpt::math;

std::string mrpt::graphslam::detail::getGridMapAlignmentResultsAsString(
			const mrpt::poses::CPosePDF& pdf,
			const mrpt::slam::CGridMapAligner::TReturnInfo& ret_info) {

			CPosePDFSOGPtr pdf_out = CPosePDFSOG::Create();
			pdf_out->copyFrom(pdf);
			CPose2D pose_out; CMatrixDouble33 cov_out;
			pdf_out->getMostLikelyCovarianceAndMean(cov_out, pose_out);

			stringstream ss;
			ss << "--------------------" << endl;
			ss << "Results: " << endl
				<< "\tPDFPtr pose: " << pdf.getMeanVal() << endl
				<< "\t# Correspondences: " << ret_info.correspondences.size() << endl
				<< "\tAlignment goodness: " << ret_info.goodness << endl
				<< "\tModes size: " << pdf_out->size() << endl
				<< "\tModes: " << endl << getSTLContainerAsString(pdf_out->getSOGModes())
				<< "\tMost likely pose: " << pose_out << endl
				<< "\tCorresponding covariance matrix: " << endl
				<< cov_out << endl;
			ss << "--------------------" << endl;
			return ss.str();
}

bool mrpt::graphslam::detail::essentiallyEqual(double a, double b, double epsilon) {
	return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}
bool mrpt::graphslam::detail::essentiallyEqual(double a, double b) {
	return essentiallyEqual(a, b, std::numeric_limits<double>::epsilon());
}

