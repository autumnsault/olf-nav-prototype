#ifndef FILTER_HPP
# define FILTER_HPP

#include <Eigen/Dense>

#define NN 30

const double FILTER_SMALL = 1e-12;

typedef Eigen::Matrix<double,NN,NN> Cov;
typedef Eigen::Matrix<double,NN,1>  State;
typedef Eigen::Matrix<double,1,NN>  StateT;

class Filter {
public:
  Filter(const Eigen::Matrix<double,6,1>& rv)
    : P(Cov::Zero())
    , X(State::Zero())
    , dX(State::Zero())
    , Phi(Cov::Zero())
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

  
  
  Cov P;   /* (--) state covariance */
  State X; /* (--) state */
  State dX; /* (--) pending state update */

  Cov Phi; /* (--) state transition matrix */
};

#endif
