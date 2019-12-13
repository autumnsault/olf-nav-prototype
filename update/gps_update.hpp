#ifndef GPS_UPDATE_HPP
# define GPS_UPDATE_HPP

#include "filter.hpp"
#include "update.hpp"

/** @brief GPS scalar update
 *
 * We really don't want to do a vector update with GPS since it's got
 * a 6x6 inversion, and that'll probably be too slow. So instead we'll
 * use 6 columns of scalar updates.
 */
class GPSScalarUpdate : public ScalarUpdate<6> {
public:
  GPSScalarUpdate(Filter& kf,
		  const Eigen::Matrix<double,6,1>& y)
    : ScalarUpdate<6>(kf, y)
  {
    // 1. Compute measurement model and use that to get the
    //    measurement residual.
    // FIXME: Some matrix math really ought to happen here.
    for (int ii = 0; ii < 6; ++ii) {
      dy[ii] = y[ii] - kf.X[ii];
    }

    // 2. Compute the measurement sensitivity matrix H.
    MeasSens H_now = MeasSens::Zero();
    for (meas_size_t ii = 0; ii < 6; ++ii)
      H_now(ii,ii) = 1.0;
    
    H = H_now * kf.Phi; // FIXME: Need to use a Phi which is properly
         		//        interpolated

    // 3. Compute the measurement covariance matrix R.
    R.diagonal()(0) = 1.0;
    R.diagonal()(1) = 1.0;
    R.diagonal()(2) = 1.0;
    
    R.diagonal()(3) = 0.1;
    R.diagonal()(4) = 0.1;
    R.diagonal()(5) = 0.1;
  }
};

#endif
