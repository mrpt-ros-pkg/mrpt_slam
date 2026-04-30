#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <mrpt/math/CMatrixFixed.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "mrpt_ekf_slam_3d/ekf_slam_math.hpp"

// ── makeRightHanded ──────────────────────────────────────────────────────────

TEST(MakeRightHanded3D, IdentityMatrixPreserved)
{
	// Identity is already right-handed (det = +1).
	Eigen::Matrix3d vecs = Eigen::Matrix3d::Identity();
	Eigen::Vector3d vals = Eigen::Vector3d::Ones();
	mrpt_ekf_slam_3d::makeRightHanded(vecs, vals);
	EXPECT_GT(vecs.determinant(), 0.0);
}

TEST(MakeRightHanded3D, FlippedVectorsBecomesRightHanded)
{
	// Swap col 0 and col 1 of identity → left-handed (det = -1).
	Eigen::Matrix3d vecs;
	vecs.col(0) = Eigen::Vector3d(0, 1, 0);
	vecs.col(1) = Eigen::Vector3d(1, 0, 0);
	vecs.col(2) = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d vals(2.0, 1.0, 0.5);
	mrpt_ekf_slam_3d::makeRightHanded(vecs, vals);
	EXPECT_GT(vecs.determinant(), 0.0);
}

TEST(MakeRightHanded3D, AlreadyRightHandedUnchanged)
{
	// Identity already has det = +1; must still have det ≈ +1 after call.
	Eigen::Matrix3d vecs = Eigen::Matrix3d::Identity();
	Eigen::Vector3d vals(4.0, 2.0, 1.0);
	mrpt_ekf_slam_3d::makeRightHanded(vecs, vals);
	EXPECT_NEAR(vecs.determinant(), 1.0, 1e-10);
}

// ── computeEllipseOrientationScale3D ─────────────────────────────────────────

TEST(EllipseOrientationScale3D, DiagonalCovarianceOrdering)
{
	// For a diagonal covariance the eigenvalues equal the diagonal entries.
	// We expect scale_x >= scale_y >= scale_z.
	mrpt::math::CMatrixDouble33 cov;
	cov(0, 0) = 9.0;
	cov(1, 1) = 4.0;
	cov(2, 2) = 1.0;
	cov(0, 1) = cov(1, 0) = 0.0;
	cov(0, 2) = cov(2, 0) = 0.0;
	cov(1, 2) = cov(2, 1) = 0.0;

	double scale_x, scale_y, scale_z;
	mrpt_ekf_slam_3d::computeEllipseOrientationScale3D(
		scale_x, scale_y, scale_z, cov);

	EXPECT_NEAR(scale_x, 9.0, 1e-10);
	EXPECT_NEAR(scale_y, 4.0, 1e-10);
	EXPECT_NEAR(scale_z, 1.0, 1e-10);
	EXPECT_GE(scale_x, scale_y);
	EXPECT_GE(scale_y, scale_z);
}

TEST(EllipseOrientationScale3D, EigenvaluesAreNonNegative)
{
	// Any valid covariance matrix has non-negative eigenvalues.
	mrpt::math::CMatrixDouble33 cov;
	cov(0, 0) = 5.0;
	cov(1, 1) = 3.0;
	cov(2, 2) = 1.0;
	cov(0, 1) = cov(1, 0) = 1.0;
	cov(0, 2) = cov(2, 0) = 0.5;
	cov(1, 2) = cov(2, 1) = 0.5;

	double scale_x, scale_y, scale_z;
	mrpt_ekf_slam_3d::computeEllipseOrientationScale3D(
		scale_x, scale_y, scale_z, cov);

	EXPECT_GE(scale_x, 0.0);
	EXPECT_GE(scale_y, 0.0);
	EXPECT_GE(scale_z, 0.0);
}

// ── is_file_exists ────────────────────────────────────────────────────────────

TEST(IsFileExists3D, ExistingFileReturnsTrue)
{
	char tmpname[] = "/tmp/gtest_ekf_slam_3d_XXXXXX";
	int fd = mkstemp(tmpname);
	ASSERT_GE(fd, 0);
	close(fd);

	EXPECT_TRUE(mrpt_ekf_slam_3d::is_file_exists(tmpname));

	std::remove(tmpname);
}

TEST(IsFileExists3D, NonexistentFileReturnsFalse)
{
	EXPECT_FALSE(mrpt_ekf_slam_3d::is_file_exists(
		"/tmp/this_file_does_not_exist_gtest_ekf_3d"));
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
