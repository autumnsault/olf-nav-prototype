#ifndef NAV_TYPES_HPP
# define NAV_TYPES_HPP

#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::DiagonalMatrix;

#define NN 30

typedef Eigen::Matrix<double,NN,NN> Cov;
typedef Eigen::Matrix<double,NN,1>  State;
typedef Eigen::Matrix<double,1,NN>  StateT;
typedef Eigen::Matrix<double,9,9>   Matrix9d;


const double FILTER_SMALL = 1e-12;  /* (--) small number used for underflow and div by zero */
const double PROP_DT      = 0.0125; /* (s)  propagation time step */
const double TAU          = 600.0;  /* (s)  time constant for ECRVs */
const double Q_ACCEL_PSD  = 1e-7;   /* accelerometer noise power spectral density */
const double Q_GYRO_PSD   = 1e-12;  /* gyroscope noise power spectral density */

#endif
