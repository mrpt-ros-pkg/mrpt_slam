#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <mrpt/math/CMatrixFixed.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "mrpt_ekf_slam_2d/ekf_slam_math.hpp"

// ── makeRightHanded ──────────────────────────────────────────────────────────

TEST(MakeRightHanded2D, IdentityMatrixPreserved)
{
	// Identity is already right-handed (det = +1); nothing should change.
	Eigen::Matrix2d vecs = Eigen::Matrix2d::Identity();
	Eigen::Vector2d vals = Eigen::Vector2d::Ones();
	mrpt_ekf_slam_2d::makeRightHanded(vecs, vals);
	EXPECT_GT(vecs.determinant(), 0.0);
}

TEST(MakeRightHanded2D, FlippedVectorsBecomesRightHanded)
{
	// Swapped columns → left-handed (det = -1). After the call, det > 0.
	Eigen::Matrix2d vecs;
	vecs << 0, 1, 1, 0;  // det = -1
	Eigen::Vector2d vals(2.0, 1.0);
	mrpt_ekf_slam_2d::makeRightHanded(vecs, vals);
	EXPECT_GT(vecs.determinant(), 0.0);
}

TEST(MakeRightHanded2D, AlreadyRightHandedUnchanged)
{
	// det = +1 before the call; must still be +1 after.
	Eigen::Matrix2d vecs;
	vecs << 1, 0, 0, 1;  // det = +1
	Eigen::Vector2d vals(3.0, 2.0);
	mrpt_ekf_slam_2d::makeRightHanded(vecs, vals);
	EXPECT_NEAR(vecs.determinant(), 1.0, 1e-10);
}

// ── computeEllipseOrientationScale2D ─────────────────────────────────────────

TEST(EllipseOrientationScale2D, DiagonalCovarianceEigenvalues)
{
	// Diagonal covariance: eigenvalues equal the diagonal entries.
	// scale_x must be the major (larger) semi-axis.
	mrpt::math::CMatrixDouble22 cov;
	cov(0, 0) = 4.0;
	cov(1, 1) = 1.0;
	cov(0, 1) = cov(1, 0) = 0.0;

	double scale_x, scale_y, angle;
	mrpt_ekf_slam_2d::computeEllipseOrientationScale2D(
		scale_x, scale_y, angle, cov);

	EXPECT_NEAR(scale_x, 4.0, 1e-10);
	EXPECT_NEAR(scale_y, 1.0, 1e-10);
	EXPECT_GE(scale_x, scale_y);
}

TEST(EllipseOrientationScale2D, SymmetricCovarianceNonZeroAngle)
{
	// Off-diagonal covariance → major axis is not aligned with X.
	mrpt::math::CMatrixDouble22 cov;
	cov(0, 0) = 2.0;
	cov(1, 1) = 2.0;
	cov(0, 1) = cov(1, 0) = 1.0;

	double scale_x, scale_y, angle;
	mrpt_ekf_slam_2d::computeEllipseOrientationScale2D(
		scale_x, scale_y, angle, cov);

	EXPECT_GE(scale_x, scale_y);
	EXPECT_GT(std::abs(angle), 1e-6);
}

TEST(EllipseOrientationScale2D, EigenvaluesAreNonNegative)
{
	// A valid covariance matrix has non-negative eigenvalues.
	mrpt::math::CMatrixDouble22 cov;
	cov(0, 0) = 3.0;
	cov(1, 1) = 0.5;
	cov(0, 1) = cov(1, 0) = 0.5;

	double scale_x, scale_y, angle;
	mrpt_ekf_slam_2d::computeEllipseOrientationScale2D(
		scale_x, scale_y, angle, cov);

	EXPECT_GE(scale_x, 0.0);
	EXPECT_GE(scale_y, 0.0);
}

// ── is_file_exists ────────────────────────────────────────────────────────────

TEST(IsFileExists, ExistingFileReturnsTrue)
{
	char tmpname[] = "/tmp/gtest_ekf_slam_2d_XXXXXX";
	int fd = mkstemp(tmpname);
	ASSERT_GE(fd, 0);
	close(fd);

	EXPECT_TRUE(mrpt_ekf_slam_2d::is_file_exists(tmpname));

	std::remove(tmpname);
}

TEST(IsFileExists, NonexistentFileReturnsFalse)
{
	EXPECT_FALSE(mrpt_ekf_slam_2d::is_file_exists(
		"/tmp/this_file_does_not_exist_gtest_ekf_2d"));
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
