#pragma once
/*
 * Modified for adaptive lookahead + physics-based caps
 * Original authors:
 *   Michael Ferguson, Fetch Robotics Inc., et al.
 */

#include <cmath>

namespace graceful_controller
{

class GracefulController
{
public:
  // NOTE: constructor signature changed: (removed beta/lambda, added a_lat_max, omega_max_sust)
  GracefulController(double k1, double k2,
                     double min_abs_velocity, double max_abs_velocity,
                     double max_decel,
                     double max_abs_angular_velocity,
                     double a_lat_max, double omega_max_sust);

  // x, y, theta are in robot (base) frame coordinates to the target pose
  bool approach(const double x, const double y, const double theta,
                double& vel_x, double& vel_th, bool backward_motion);

  void setVelocityLimits(const double min_abs_velocity,
                         const double max_abs_velocity,
                         const double max_abs_angular_velocity);

private:
  // controller gains
  double k1_{2.0};
  double k2_{1.0};

  // velocity limits
  double min_abs_velocity_{0.05};
  double max_abs_velocity_{0.5};
  double max_decel_{1.0};
  double max_abs_angular_velocity_{1.0};

  // physics caps
  double a_lat_max_{1.0};      // [m/s^2]
  double omega_max_sust_{2.5}; // [rad/s]
};

} // namespace graceful_controller