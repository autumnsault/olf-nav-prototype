#ifndef UPDATE_HPP
# define UPDATE_HPP

#include <iostream>

#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::DiagonalMatrix;

#include "filter.hpp"


template <size_t MM>
class Update {
public:
  Update(Filter& kf, const Matrix<double,MM,1>& y)
    : z2(0.0)
    , pass(true)
  {}

  virtual void apply(Filter& kf, const Matrix<double,MM,1>& y,
		     float z2_max = 16.0) = 0;

  float z2;  /* (--) square of z-score, which is Mahalanobis distance
	      *      of measurement from expectation */
  bool pass; /* (--) did residual edit check pass? */
};


template <size_t MM>
class ScalarUpdate : public Update<MM> {
public:
  typedef Matrix<double,MM,1> Meas;
  typedef Matrix<double,1,MM> MeasT;
  typedef DiagonalMatrix<double,MM> MeasCov;
  typedef Matrix<double,MM,NN> MeasSens;
  typedef Matrix<double,NN,MM> MeasSensT;
  
  ScalarUpdate(Filter& kf, const Matrix<double,MM,1>& y)
    : Update<MM>::Update(kf, y)
  {}

  void apply(Filter& kf, const Matrix<double,MM,1>& y,
		     float z2_max = 16.0)
  {
    MeasSensT PHt;
    // 3. Perform residual edit check. Perform underweighting if
    //    needed.
    for (size_t kk = 0; Update<MM>::pass && kk < MM; ++kk) {
      StateT Hk = H.row(kk);
      PHt.col(kk) = kf.P * Hk.transpose();
      W.diagonal()(kk) = Hk * PHt.col(kk);

      // Should we do anything here about dX from other updates during
      // this cycle, to account for it in the residual edit check?

      W.diagonal()(kk) += R.diagonal()(kk);

      // FIXME: Needs measurement underweighting

      // Residual edit check
      if (W.diagonal()[kk] < FILTER_SMALL) {
        Update<MM>::z2 += (dy[kk] * dy[kk]) / W.diagonal()[kk];
      } else {
	Update<MM>::z2 = std::numeric_limits<float>::infinity();
      }

      if (Update<MM>::z2 > z2_max) Update<MM>::pass = false;
    }

    if (Update<MM>::pass) {
      for (size_t kk = 0; kk < MM; ++kk) {
	double w = W.diagonal()[kk];
	
	// Compute Kalman gain column kk
	State K = PHt.col(kk) / w;
	
	kf.dX += K * dy[kk];
	dx    += K * dy[kk]; // local copy just from this update
	
	for (size_t ii = 0; ii < NN; ++ii) {
	  for (size_t jj = ii; jj < NN; ++jj) {
	    kf.P(ii,jj) = kf.P(ii,jj) - K[ii] * PHt(jj,kk)
	                              - K[jj] * PHt(ii,kk)
	                              + K[ii] * K[jj] * w;
	    if (ii != jj)
	      kf.P(jj,ii) = kf.P(ii,jj);
	  }
	}
      }
    }
  }
  
  MeasCov W; /* (--) diagonal innovation covariance */
  Meas dy;   /* (--) measurement residual */
  MeasSens H;/* (--) measurement sensitivity matrix */
  MeasCov R; /* (--) diagonal measurement covariance */

  /* Debugging variables */
  State dx;   /* (--) state update vector, for debugging */

};

#endif
