#include "gps_update.hpp"
#include "filter.hpp"
#include "prop.hpp"


int main() {
  GravBody Earth(398600442.0, 6371400.0);
  Eigen::Vector3d w_planet(0.0, 0.0, M_PI / (12 * 3600.0));
  
  // Initial position and velocity with noise
  Eigen::Matrix<double,6,1> rv;
  rv << 1000.0, 1000.0, 1000.0,
        10.0,   10.0,   10.0;
  Eigen::Matrix<double,6,1> wn;
  wn << 0.1,  -0.1,    0.2,
        0.001, 0.001, -0.001;
  
  Filter kf(0.0, rv);
  Prop prop(kf.time, kf.X);

  Eigen::Vector3d dtheta(0.01, 0.02, 0.03);
  Eigen::Vector3d dv(0.0, 0.0, -0.01);

  for (size_t ii = 0; ii < 8; ++ii) {
    prop.propagate(dv, dtheta, Earth, w_planet);
  }
  
  // Setup a measurement
  Eigen::Matrix<double,6,1> y;
  for (size_t ii = 0; ii < 3; ++ii) {
    y[ii]   = prop.r_imu_inrtl[ii] + wn[ii];
    y[ii+3] = prop.v_imu_inrtl[ii] + wn[ii+3];
  }
  
  kf.prepare_for_update(prop);
  GPSScalarUpdate gps(kf, y);
  gps.apply(kf, y);


  return 0;
}
