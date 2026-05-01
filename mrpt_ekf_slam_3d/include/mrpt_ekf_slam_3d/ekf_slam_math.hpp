// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: ekf_slam_math.hpp
 *
 * Pure-math free functions extracted from EKFslamWrapper (3D) so that unit
 * tests can exercise them without instantiating the full ROS 2 node.
 */

#pragma once

#include <Eigen/Dense>
#include <mrpt/math/CMatrixFixed.h>
#include <fstream>
#include <string>

namespace mrpt_ekf_slam_3d
{

/**
 * @brief Check whether a file exists at the given path.
 *
 * @param name  path to the file
 * @return true if the file can be opened for reading
 */
inline bool is_file_exists(const std::string & name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

/**
 * @brief Force a 3×3 eigenvector matrix to represent a right-handed
 *        coordinate system (det > 0).
 *
 * When the SelfAdjointEigenSolver returns a left-handed system the first
 * two columns (and their corresponding eigenvalues) are swapped so that
 * the resulting frame is right-handed.
 *
 * @param eigenvectors  3×3 column-eigenvector matrix (modified in-place)
 * @param eigenvalues   corresponding eigenvalue vector  (modified in-place)
 */
inline void makeRightHanded(
  Eigen::Matrix3d & eigenvectors, Eigen::Vector3d & eigenvalues)
{
  Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);
  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);
  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);
  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0) {
                // Left-handed — swap the first two columns and their eigenvalues.
    eigenvectors << c1, c0, c2;
    double e = eigenvalues[0];
    eigenvalues[0] = eigenvalues[1];
    eigenvalues[1] = e;
  } else {
    eigenvectors << c0, c1, c2;
  }
}

/**
 * @brief Compute the semi-axis scales of a 3-D covariance ellipsoid.
 *
 * Eigenvalues are sorted in descending order: scale_x >= scale_y >= scale_z.
 *
 * @param scale_x  output: largest eigenvalue
 * @param scale_y  output: middle eigenvalue
 * @param scale_z  output: smallest eigenvalue
 * @param cov      3×3 covariance matrix
 */
inline void computeEllipseOrientationScale3D(
  double & scale_x, double & scale_y, double & scale_z,
  const mrpt::math::CMatrixDouble33 & cov)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov.asEigen());
        // SelfAdjointEigenSolver returns eigenvalues in ascending order.
  scale_z = solver.eigenvalues()[0];
  scale_y = solver.eigenvalues()[1];
  scale_x = solver.eigenvalues()[2];
}

}  // namespace mrpt_ekf_slam_3d
