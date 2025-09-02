#pragma once
/*********************************************************************
 * GracefulControllerROS — header
 * Modified to support adaptive lookahead & physics-based speed caps
 * while keeping costmap-based collision simulation as a safety gate.
 *
 * Original authors: E. Marder-Eppstein, M. Ferguson
 * Modifications: GOAT Robotics tuning (2025)
 *********************************************************************/

#include <string>
#include <mutex>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace graceful_controller
{

// forward declare the core controller
class GracefulController;

// free helper (implemented in .cpp)
void computeDistanceAlongPath(const std::vector<geometry_msgs::msg::PoseStamped> &poses,
                              std::vector<double> &distances);

class GracefulControllerROS : public nav2_core::Controller
{
public:
  GracefulControllerROS();
  ~GracefulControllerROS() override;

  // --- nav2_core::Controller API ---
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override;

  void setPlan(const nav_msgs::msg::Path &path) override;

  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

private:
  // --- internal helpers (declared; implemented in .cpp) ---
  bool simulate(const geometry_msgs::msg::PoseStamped &target_pose,
                const geometry_msgs::msg::Twist &velocity,
                geometry_msgs::msg::TwistStamped &cmd_vel);

  // rotate towards a pose; returns yaw error used (rad)
  double rotateTowards(const geometry_msgs::msg::PoseStamped &pose,
                       const geometry_msgs::msg::Twist &velocity,
                       geometry_msgs::msg::TwistStamped &cmd_vel,
                       bool perform_rotation);

  // apply a yaw command directly
  void rotateTowards(double yaw,
                     const geometry_msgs::msg::Twist &velocity,
                     geometry_msgs::msg::TwistStamped &cmd_vel,
                     bool perform_rotation);

  // (optional) plan processing hooks — provided to match usage in .cpp
  nav_msgs::msg::Path addOrientations(const nav_msgs::msg::Path &path)
  {
    // If your project already has implementations elsewhere, replace with those.
    // Minimal pass-through keeps API compatible.
    return path;
  }

  nav_msgs::msg::Path applyOrientationFilter(const nav_msgs::msg::Path &path,
                                             double /*yaw_filter_tolerance*/,
                                             double /*yaw_gap_tolerance*/)
  {
    return path;
  }

  void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);

private:
  // ----------------- ROS handles -----------------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  rclcpp::Clock::SharedPtr clock_;

  // pubs
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_points_pub_;

  // subs / callback group (for backward motion pose subscription)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // dynamic param callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ----------------- Plans & state -----------------
  nav_msgs::msg::Path global_plan_;
  visualization_msgs::msg::MarkerArray *collision_points_;

  geometry_msgs::msg::PoseStamped robot_pose_;
  bool robot_pose_received_{false};

  bool initialized_{false};
  bool has_new_path_{false};
  bool goal_achieved_{false};
  bool goal_tolerance_met_{false};
  bool backward_motion_{false};

  // ----------------- Core controller -----------------
  std::shared_ptr<GracefulController> controller_;

  // ----------------- Parameters (existing/legacy) -----------------
  double max_vel_x_{0.5};
  double min_vel_x_{0.1};
  double max_vel_theta_{1.0};
  double min_in_place_vel_theta_{0.4};

  double max_x_to_max_theta_scale_factor_{100.0};

  double acc_lim_x_{2.5};
  double acc_lim_theta_{3.2};
  double acc_dt_{0.25};
  double decel_lim_x_{1.0};

  // legacy lookahead bounds (now kept only for back-compat / logging)
  double max_lookahead_{1.0};
  double min_lookahead_{0.25};

  double initial_rotate_tolerance_{0.1};
  bool prefer_final_rotation_{false};

  bool compute_orientations_{true};
  bool use_orientation_filter_{false};
  double yaw_filter_tolerance_{0.785};
  double yaw_gap_tolerance_{0.25};
  double yaw_slowing_factor_{0.5};
  bool latch_xy_goal_tolerance_{false};

  // footprint scaling with speed
  double scaling_vel_x_{0.3};
  double scaling_factor_{0.4};
  double scaling_step_{0.05};

  // backward motion
  bool backward_motion_available_{false};
  double backwards_check_yaw_tolerance_{0.34};

  // near-goal orientation logic
  double ignore_orientation_distance_{0.10};

  // costmap resolution (used in simulate step size)
  double resolution_{0.05};

  // limits computed per-cycle
  double max_vel_x_limited_{0.5};
  double max_vel_theta_limited_{1.0};

  // ----------------- NEW: Adaptive lookahead & physics caps -----------------
  // adaptive lookahead params
  double L0_{0.25};            // base lookahead [m]
  double beta_la_{0.5};        // preview time [s]
  double gamma_la_{1.0};       // curvature tempering length [m]
  double p_la_{0.9};           // tempering exponent
  double Lmin_{0.2};
  double Lmax_{1.0};
  double lookahead_rate_limit_{1.5}; // dL/dt [m/s]
  double odom_delay_{0.0};           // [s], optional latency compensation

  // rate-limited state
  double L_prev_{0.3};

  // physics limits
  double a_lat_max_{1.0};      // [m/s^2] safe lateral acceleration
  double omega_max_sust_{2.5}; // [rad/s] sustained yaw rate capability

  // ----------------- Synchronization -----------------
  std::mutex config_mutex_;
};

} // namespace graceful_controller