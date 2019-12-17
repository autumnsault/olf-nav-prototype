#ifndef FRAMES_HPP
# define FRAMES_HPP

#include "vector_math.hpp"

/** @brief Compute a transformation matrix from the inertial frame to
 **        the planet-centered, planet-fixed frame.
 *
 * @param[in]  time              time since epoch (s)
 * @param[in]  w_pcpf_wrt_inrtl  angular rate of planet (r/s)
 *
 * @returns A transformation matrix from the inertial frame to the PCPF frame.
 */
Eigen::Matrix3d compute_T_inrtl_to_pcpf(const double& time,
					const Eigen::Vector3d& w_pcpf_wrt_inrtl) {
  return rotvec_to_matrix(w_pcpf_wrt_inrtl * time);
}

#endif
