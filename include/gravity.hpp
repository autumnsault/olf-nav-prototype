#ifndef GRAVITY_HPP
# define GRAVITY_HPP

#include <Eigen/Dense>

#include <cmath>

/** @brief Parameters for gravity model */
class GravBody {
protected:
  double mu;         /* (m3/s2) gravitational constant of planet */
  double eq_radius;  /* (m)     mean equatorial radius of planet */
public:
  GravBody(const double& mu_, const double& r_eq)
    : mu(mu_),
      eq_radius(r_eq)
  { }
  

  /** @brief Compute gravitational acceleration and gravity gradient
   *         (change in acceleration with respect to change in position)
   *
   * This function treats the planet as a point mass.
   *
   * @param[in]  r_pcpf   position in planet-centered, planet-fixed
   *                      coordinates; should have same units as
   *                      params->mu (meters, usually)
   * @param[out] a_pcpf   acceleration due to gravity in PCPF frame
   * @param[out] da_dr    partial of change in acceleration with respect
   *                      to change in position
   */
  void accel(const Eigen::Vector3d& r_pcpf,
	     Eigen::Vector3d& a_pcpf,
	     Eigen::Matrix3d& da_dr) const {
    double r2 = r_pcpf.squaredNorm();
    double r3 = r2 * sqrt(r2);
    double mu_over_r3 = mu / r3;

    // Compute acceleration vector
    a_pcpf = r_pcpf * -mu_over_r3;

    // Compute gravity gradient
    da_dr = (r_pcpf * r_pcpf.transpose() * 3.0 / r2 - Eigen::Matrix3d::Identity()) * mu_over_r3;
  }
};

#endif

