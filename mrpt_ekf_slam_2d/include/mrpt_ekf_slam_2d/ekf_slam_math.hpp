// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/*
 * File: ekf_slam_math.hpp
 *
 * Pure-math free functions extracted from EKFslamWrapper so that unit tests
 * can exercise them without instantiating the full ROS 2 node.
 */

#pragma once

#include <Eigen/Dense>
#include <mrpt/math/CMatrixFixed.h>
#include <cmath>
#include <fstream>
#include <string>

namespace mrpt_ekf_slam_2d
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
 * @brief Force a 2×2 eigenvector matrix to represent a right-handed
 *        coordinate system (determinant = +1).
 *
 * When the SelfAdjointEigenSolver returns a left-handed system the two
 * columns (and their corresponding eigenvalues) are swapped so that
 * det(eigenvectors) > 0.
 *
 * @param eigenvectors  2×2 column-eigenvector matrix (modified in-place)
 * @param eigenvalues   corresponding eigenvalue vector  (modified in-place)
 */
inline void makeRightHanded(
  Eigen::Matrix2d & eigenvectors, Eigen::Vector2d & eigenvalues)
{
        // Embed the 2-D columns into 3-D so that the cross product can determine
        // handedness.
  Eigen::Vector3d c0;
  c0.setZero();
  c0.head<2>() = eigenvectors.col(0);
  c0.normalize();
  Eigen::Vector3d c1;
  c1.setZero();
  c1.head<2>() = eigenvectors.col(1);
  c1.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc[2] < 0) {
                // Left-handed — swap columns and the matching eigenvalues.
    eigenvectors << c1.head<2>(), c0.head<2>();
    double e = eigenvalues[0];
    eigenvalues[0] = eigenvalues[1];
    eigenvalues[1] = e;
  } else {
    eigenvectors << c0.head<2>(), c1.head<2>();
  }
}

/**
 * @brief Compute the orientation angle and semi-axis scales of a 2-D
 *        covariance ellipse.
 *
 * Eigenvalues are sorted so that scale_x >= scale_y (scale_x is the major
 * semi-axis).  The returned angle is the direction of the major eigenvector
 * measured from the positive X-axis (atan2 convention, radians).
 *
 * @param scale_x  output: larger eigenvalue  (major axis²)
 * @param scale_y  output: smaller eigenvalue (minor axis²)
 * @param angle    output: orientation angle of the major axis (radians)
 * @param cov      2×2 covariance matrix
 */
inline void computeEllipseOrientationScale2D(
  double & scale_x, double & scale_y, double & angle,
  const mrpt::math::CMatrixDouble22 & cov)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov.asEigen());
        // SelfAdjointEigenSolver returns eigenvalues in ascending order.
        // Index 0 → smallest, index 1 → largest.
  scale_y = solver.eigenvalues()[0];
  scale_x = solver.eigenvalues()[1];
        // Orientation: angle of the major eigenvector (column 1).
  angle = std::atan2(
                solver.eigenvectors()(1, 1), solver.eigenvectors()(0, 1));
}

}  // namespace mrpt_ekf_slam_2d
