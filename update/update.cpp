#include "gps_update.hpp"


int main() {
  // Initial position and velocity with noise
  Eigen::Matrix<double,6,1> rv;
  rv << 1000.0, 1000.0, 1000.0,
        10.0,   10.0,   10.0;
  Eigen::Matrix<double,6,1> wn;
  wn << 0.1,  -0.1,    0.2,
        0.001, 0.001, -0.001;

  Eigen::Matrix<double,6,1> y;
  for (size_t ii = 0; ii < 6; ++ii)
    y[ii] = rv[ii] + wn[ii];
  
  Filter kf(rv);
  GPSScalarUpdate(kf, y);
  
}
