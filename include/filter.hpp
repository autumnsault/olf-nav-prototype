#ifndef FILTER_HPP
# define FILTER_HPP

#include <Eigen/Dense>

#include "nav_types.hpp"
#include "prop.hpp"

DiagonalMatrix<double,9> compute_process_noise(const double& dt) {
  DiagonalMatrix<double,9> Q;
  double qa = Q_ACCEL_PSD * dt,
    qv = qa * dt * 0.5,
    qr = qv * dt * 2.0 / 3.0,
    qw = Q_GYRO_PSD * dt;
  Q.diagonal() << qr, qr, qr, qa, qa, qa, qw, qw, qw;
  return Q;
}


class Filter {
public:
  Filter(const double& time, const Eigen::Matrix<double,6,1>& rv)
    : Phi(Cov::Identity())
    , P(Cov::Zero())
    , X(State::Zero())
    , dX(State::Zero())
  {
    // Initialize covariance to some reasonable values
    P.diagonal() <<
      rv,
      0.5, 0.5, 0.5,
      0.0001, 0.0001, 0.0001,
      0.00001, 0.00001, 0.00001,
      0.1, 0.1, 0.1,
      0.1, 0.1, 0.1,
      0.1, 0.1, 0.1,
      0.1, 0.1, 0.1,
      0.1, 0.1, 0.1;

    // Initialize state to some reasonable values
    X[0] = 6375000.0;
    X[4] = 10.0;
  }


  /** @brief Having propagated for a while in a Prop object, prepare
   **        the filter to receive an update at or just before the current
   **        time.
   *
   * This method computes the covariance at the current time using the
   * posterior covariance from the last update and the propagated
   * state transition matrix.
   *
   */
  void prepare_for_update(const Prop& prop) {
    double dt = prop.time - time;
    DiagonalMatrix<double,9> Q = compute_process_noise(dt);

    Phi << prop.Phi_xx, prop.Phi_xb, Matrix<double,21,30>::Zero();
    for (size_t ii = 0; ii < 21; ++ii) {
      Phi(ii+9,ii+9) = prop.Phi_bb.diagonal()(ii);
    }

    P = Phi * (P * Phi.transpose());
    for (size_t ii = 0; ii < 9; ++ii) {
      P(ii,ii) += Q.diagonal()(ii);
    }

    for (size_t ii = 0; ii < 3; ++ii) {
      X[ii]    = prop.r_imu_inrtl(ii);
      X[ii+3]  = prop.v_imu_inrtl(ii);
      X[ii+6]  = 0.0;
      X[ii+9]  = prop.b_acc(ii);
      X[ii+12] = prop.b_gyro(ii);
    }
    
    time = prop.time;
    // FIXME: Make sure this is symmetric after.
  }

  double time; /* (s)  time of state/covariance validity */
  Cov Phi;     /* (--) state transition matrix */
  Cov P;       /* (--) state covariance */
  State X;     /* (--) state */
  State dX;    /* (--) pending state update */
};

#endif
