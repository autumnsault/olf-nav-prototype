#ifndef VECTOR_MATH_HPP
# define VECTOR_MATH_HPP

#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

const double ANGLE_SMALL = 1e-12;

Matrix3d skew(const Vector3d& a) {
  Matrix3d ax;
  ax << 0.0, -a(2), a(1),
    a(2), 0.0, -a(0),
    -a(1), a(0), 0.0;
  return ax;
}

/** @brief Convert a rotation vector to a DCM.
 *
 * @param[in]   v   rotation vector (unit rotation axis multiplied by
 *                  angle about that vector)
 *
 * @returns A 3D matrix.
 */
Matrix3d rotvec_to_matrix(const Vector3d& v) {
  double v_mag = v.norm();
  double c     = std::cos(v_mag);
  double s     = std::sin(v_mag);

  Vector3d vu = v / v_mag;

  Matrix3d T;
  if (v_mag < ANGLE_SMALL) T = Matrix3d::Identity();
  else {
    // Reference: https://github.com/mohawkjohn/pyquat/blob/master/pyquat/pyquat.c
    // Line 932
    T << c + vu[0]*vu[0] * (1.0 - c),
      vu[0]*vu[1] * (1.0 - c) + vu[2] * s,
      vu[0]*vu[2] * (1.0 - c) - vu[1] * s,
      vu[0]*vu[1] * (1.0 - c) - vu[2] * s,
      c + vu[1]*vu[1] * (1.0 - c),
      vu[1]*vu[2] * (1.0 - c) + vu[0] * s,
      vu[0]*vu[2] * (1.0 - c) + vu[1] * s,
      vu[1]*vu[2] * (1.0 - c) - vu[0] * s,
      c + vu[2]*vu[2] * (1.0 - c);
  }

  return T;
}

#endif
