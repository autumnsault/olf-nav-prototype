#include <cmath>


#include "prop.hpp"
#include "gravity.hpp"
#include "nav_types.hpp"

int main() {
  GravBody Earth(398600442.0, 6371400.0);
  Eigen::Vector3d w_planet(0.0, 0.0, M_PI / (12 * 3600.0));
  
  State X(State::Zero());
  Prop prop(0.0, X);

  Eigen::Vector3d dtheta(0.01, 0.02, 0.03);
  Eigen::Vector3d dv(0.0, 0.0, -0.01);

  prop.propagate(dv, dtheta, Earth, w_planet);

  return 0;
}
