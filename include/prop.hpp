#ifndef PROP_HPP
# define PROP_HPP

#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::DiagonalMatrix;

#include "frames.hpp"
#include "gravity.hpp"
#include "vector_math.hpp"
#include "nav_types.hpp"


class Prop {
public:
  /* State */
  double time;
  Vector3d r_imu_inrtl;
  Vector3d v_imu_inrtl;
  Vector3d b_acc;
  Vector3d b_gyro;

  /* Attitude State */
  Matrix3d T_inrtl_to_body; /* (--) rotation matrix from inrtl frame to body frame */

  /* State Transition */
  Matrix<double,9,9>  Phi_xx_dot; /* (--) product of dynamics matrix and state transition
				   *      matrix */
  Matrix<double,9,21> Phi_xb_dot;
  
  Matrix<double,9,9>  Phi_xx;     /* (--) state transition matrix from previous update to
		                   *      present */
  Matrix<double,9,21> Phi_xb;
  DiagonalMatrix<double,21> Phi_bb; /* (--) ECRV portion of the state transition matrix */


  /** @brief Initialize propagation from a previous update cycle.
   *
   */
  Prop(const double& kf_time,
       const State& X_posterior)
    : time(kf_time)
    , Phi_xx_dot(Matrix9d::Zero())
    , Phi_xb_dot(Matrix<double,9,21>::Zero())
    , Phi_xx(Matrix9d::Identity())
    , Phi_xb(Matrix<double,9,21>::Zero())
  {
    for (size_t ii = 0; ii < 21; ++ii) {
      Phi_bb.diagonal()(ii) = 1 / -TAU;
    }

    // Perform reset
    T_inrtl_to_body = rotvec_to_matrix(X_posterior.block<3,1>(6,0)) * T_inrtl_to_body;

    // Copy out states for propagation
    for (size_t ii = 0; ii < 3; ++ii) {
      r_imu_inrtl[ii] = X_posterior[ii];
      v_imu_inrtl[ii] = X_posterior[ii+3];
      b_acc[ii]       = X_posterior[ii+9];
      b_gyro[ii]      = X_posterior[ii+12];
    }
  }
  

  /** @brief Propagation method for incorporating IMU measurements
   **        into the predicted state and covariance
   *
   * References:
   *
   * 1. D’Souza, C., & Hanak, C. (2011). The state transition and
   *    process noise matrices in the Orion FILTNAV and EKF. Houston,
   *    TX, US.
   *
   * @param[in]  dv_meas      (m/s) IMU acceleration measurement
   * @param[in]  dtheta_meas  (r) IMU angular velocity measurement
   */
  void propagate(const Vector3d& dv_meas,
		 const Vector3d& dtheta_meas,
		 const GravBody& planet,
		 const Vector3d& w_planet)
  {
    Matrix3d T_inrtl_to_pcpf = compute_T_inrtl_to_pcpf(time, w_planet);
    Vector3d r_imu_pcpf = T_inrtl_to_pcpf * r_imu_inrtl;
    
    // Compute gravitational acceleration and gravity gradient.
    Vector3d a_imu_pcpf;
    Matrix3d G_pcpf;
    planet.accel(r_imu_pcpf, a_imu_pcpf, G_pcpf);
    Matrix3d G = T_inrtl_to_pcpf.transpose() * G_pcpf;
    Vector3d a_grav_imu = T_inrtl_to_pcpf.transpose() * a_imu_pcpf;
    Vector3d a_nongrav_imu = T_inrtl_to_body.transpose() * (dv_meas / PROP_DT - b_acc);
    Vector3d a_total_imu = a_grav_imu + a_nongrav_imu;

    // Discretized attitude propagation
    Vector3d dtheta = (dtheta_meas / PROP_DT - b_gyro);
    Matrix3d T_inrtl_to_bodynext = rotvec_to_matrix(dtheta) * T_inrtl_to_body;
    
    // State propagation
    r_imu_inrtl += v_imu_inrtl * PROP_DT + a_total_imu * 0.5 * PROP_DT*PROP_DT + T_inrtl_to_body.transpose() * (Matrix3d::Identity() + skew(dtheta / 3.0)) * a_nongrav_imu * 0.5;
    v_imu_inrtl += a_total_imu * PROP_DT + T_inrtl_to_body.transpose() * (Matrix3d::Identity() + skew(dtheta * 0.5)) * a_nongrav_imu / PROP_DT;
    

    Matrix3d dvdot_dphi = Matrix3d::Zero(); // FIXME: Moment arm needed

    Matrix3d wx = skew(dtheta_meas / PROP_DT);

    

    // The four pieces of Phi_dot are:
    // Axx Phixb     Axx Phixb + Axb Phibb
    // 0             Abb Phibb

    // XX component of Phi_dot
    
    // Set first row
    Phi_xx_dot.block<3,9>(0,0) = Phi_xx.block<3,9>(3,0) * PROP_DT; // rr, rv, rphi

    // Set second row
    Phi_xx_dot.block<3,3>(3,0) = G * Phi_xx.block<3,3>(0,0) * PROP_DT;
    Phi_xx_dot.block<3,3>(3,3) = G * Phi_xx.block<3,3>(0,3) * PROP_DT;
    Phi_xx_dot.block<3,3>(3,6) = (G * Phi_xx.block<3,3>(0,6) + dvdot_dphi * Phi_xx.block<3,3>(6,6)) * PROP_DT;

    // Set third row
    Phi_xx_dot.block<3,3>(6,6) = wx * Phi_xx.block<3,3>(6,6) * PROP_DT;
    


    Matrix3d Phi_bb_gyro;
    Phi_bb_gyro << Phi_bb.diagonal()(3), 0.0, 0.0,
      0.0, Phi_bb.diagonal()(4), 0.0,
      0.0, 0.0, Phi_bb.diagonal()(5);

    // XB component of Phi_dot

    // Set first row
    Phi_xb_dot.block<3,6>(0,0) = Phi_xb.block<3,6>(3,0) * PROP_DT;

    // Set second row
    Phi_xb_dot.block<3,3>(3,0) = (G * Phi_xb.block<3,3>(0,0) + dvdot_dphi * Phi_xb.block<3,3>(6,0)) * PROP_DT;
    Phi_xb_dot.block<3,3>(3,3) = (G * Phi_xb.block<3,3>(0,3) + dvdot_dphi * Phi_xb.block<3,3>(6,3)) * PROP_DT;

    // Set third row
    Phi_xb_dot.block<3,3>(6,0) = -wx * Phi_xb.block<3,3>(6,0) * PROP_DT;
    Phi_xb_dot.block<3,3>(6,3) = (-wx * Phi_xb.block<3,3>(6,3) - Phi_bb_gyro) * PROP_DT;
    

    // Update state transition matrix
    Phi_xx += Phi_xx_dot;
    Phi_xb += Phi_xb_dot;

    // Prepare attitude for next propagation
    T_inrtl_to_body = T_inrtl_to_bodynext;

    // Update the time
    time += PROP_DT;
    
  }
  
};

#endif
