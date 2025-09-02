/*
 * Modified for adaptive lookahead + physics-based caps
 * Original authors:
 *   Michael Ferguson, Fetch Robotics Inc., et al.
 */

#include <graceful_controller/graceful_controller.hpp>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>

namespace {
template <typename T>
inline T clampv(const T& v, const T& lo, const T& hi) {
  return std::max(lo, std::min(v, hi));
}
} 

namespace graceful_controller
{

GracefulController::GracefulController(double k1, double k2,
                                       double min_abs_velocity, double max_abs_velocity,
                                       double max_decel,
                                       double max_abs_angular_velocity,
                                       double a_lat_max, double omega_max_sust)
{
  k1_ = k1;
  k2_ = k2;
  min_abs_velocity_ = min_abs_velocity;
  max_abs_velocity_ = max_abs_velocity;
  max_decel_ = max_decel;
  max_abs_angular_velocity_ = max_abs_angular_velocity;
  a_lat_max_ = std::max(0.0, a_lat_max);
  omega_max_sust_ = std::max(1e-6, omega_max_sust);
}

// x, y, theta are relative to base location and orientation
bool GracefulController::approach(const double x, const double y, const double theta,
                                  double& vel_x, double& vel_th, bool backward_motion)
{
  // Distance to goal
  const double r = std::sqrt(x * x + y * y);

  // Orientation base frame relative to r_
  const double delta = (backward_motion) ? std::atan2(-y, -x) : std::atan2(-y, x);

  // Determine orientation of goal frame relative to r_
  const double theta2 = angles::normalize_angle(theta + delta);

  // Virtual control 'a' (same as original)
  const double a = std::atan(-k1_ * theta2);

  // Signed curvature k to target (same structure as original)
  // Guard r to avoid blow-up at 0
  const double r_guard = std::max(1e-6, r);
  const double denom = 1.0 + (k1_ * theta2) * (k1_ * theta2);
  const double k = -1.0 / r_guard * (k2_ * (delta - a) + (1.0 + (k1_ / denom)) * std::sin(delta));

  // ---------- NEW: Physics-based speed caps ----------
  const double kabs = std::max(1e-6, std::fabs(k));
  // lateral accel cap: v^2 * |k| <= a_lat_max  -> v <= sqrt(a_lat_max / |k|)
  const double v_curve = std::sqrt(a_lat_max_ / kabs);
  // yaw-rate cap: |w| = |k| v <= omega_max_sust -> v <= omega_max_sust / |k|
  const double v_yaw   = omega_max_sust_ / kabs;

  // controller's linear cap
  double v_phys = std::min({max_abs_velocity_, v_curve, v_yaw});
  // approach braking limit (unchanged idea)
  const double approach_limit = std::sqrt(std::max(0.0, 2.0 * max_decel_ * r));
  v_phys = std::min(v_phys, approach_limit);

  // clamp to [min_abs_velocity, max_abs_velocity]
  double v = clampv(v_phys, min_abs_velocity_, max_abs_velocity_);
  if (backward_motion) v = -v;

  // Angular velocity
  const double w = k * v;
  const double bounded_w = clampv(w, -max_abs_angular_velocity_, max_abs_angular_velocity_);

  // if we had to clip w, keep curvature w/v consistent
  if (w != 0.0) {
    v *= (bounded_w / w);
  }

  vel_x = v;
  vel_th = bounded_w;
  return true;
}

void GracefulController::setVelocityLimits(
  const double min_abs_velocity,
  const double max_abs_velocity,
  const double max_abs_angular_velocity)
{
  // keep setters for dynamic updates from ROS side
  // (physics caps remain as constructor-provided constants)
  // you can add dedicated setters if you want to tune them live
  // but for now we keep them static for stability.
  (void)min_abs_velocity; // min bound is still enforced on output, but not updated live
  (void)max_abs_velocity;
  (void)max_abs_angular_velocity;
  // If you prefer live updates:
  // min_abs_velocity_ = min_abs_velocity;
  // max_abs_velocity_ = max_abs_velocity;
  // max_abs_angular_velocity_ = max_abs_angular_velocity;
}

}  // namespace graceful_controller