/*********************************************************************
 * Modified GracefulControllerROS with adaptive lookahead & physics caps
 * Keeps original costmap-based collision simulation as a safety gate.
 *
 * Authors: Eitan Marder-Eppstein, Michael Ferguson (original)
 * Modifications: (c) 2025 GOAT Robotics tuning by ChatGPT
 *********************************************************************/

#include <cmath>
#include <mutex>

#include <angles/angles.h>
#include <nav_2d_utils/parameters.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/line_iterator.hpp>
#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "graceful_controller_ros/graceful_controller_ros.hpp"
#include <graceful_controller/graceful_controller.hpp>

using nav2_util::declare_parameter_if_not_declared;
using rclcpp_lifecycle::LifecyclePublisher;

namespace {
template <typename T>
inline T clampv(const T& v, const T& lo, const T& hi) {
  return std::max(lo, std::min(v, hi));
}
} 

namespace graceful_controller
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("graceful_controller");

static inline double sign(double x) { return x < 0.0 ? -1.0 : 1.0; }

// ----------------- Helpers: visualization marker stub (if you already have it elsewhere, keep it)
static void addPointMarker(double /*x*/, double /*y*/, bool /*is_collision*/,
                           visualization_msgs::msg::MarkerArray* /*viz*/) {
  // implement if you need RViz points; left as no-op here.
}

// ----------------- Collision check (unchanged except referencing addPointMarker)
bool isColliding(double x, double y, double theta,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
                 visualization_msgs::msg::MarkerArray *viz, double inflation = 1.0)
{
  unsigned mx, my;
  if (!costmap->getCostmap()->worldToMap(x, y, mx, my))
  {
    RCLCPP_INFO(LOGGER, "Pose is off the costmap bounds: [x: %.2f, y: %.2f]", x, y);
    addPointMarker(x, y, true, viz);
    return true;
  }

  if (inflation < 1.0)
  {
    RCLCPP_INFO(LOGGER, "Inflation ratio %.2f is less than 1.0. Resetting to 1.0.", inflation);
    inflation = 1.0;
  }

  // Get and inflate footprint
  std::vector<geometry_msgs::msg::Point> spec = costmap->getRobotFootprint();
  for (auto &p : spec) { p.x *= inflation; p.y *= inflation; }

  // Transform footprint
  std::vector<geometry_msgs::msg::Point> footprint;
  nav2_costmap_2d::transformFootprint(x, y, theta, spec, footprint);

  if (footprint.size() < 4)
  {
    unsigned cost = costmap->getCostmap()->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      RCLCPP_INFO(LOGGER, "Collision detected at pose: [x: %.2f, y: %.2f]", x, y);
      addPointMarker(x, y, true, viz);
      return true;
    }
    return false;
  }

  // Check each edge
  for (size_t i = 0; i < footprint.size(); ++i)
  {
    unsigned x0, y0, x1, y1;
    if (!costmap->getCostmap()->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
    {
      RCLCPP_INFO(LOGGER, "Footprint point %lu is off the costmap.", i);
      addPointMarker(footprint[i].x, footprint[i].y, true, viz);
      return true;
    }
    addPointMarker(footprint[i].x, footprint[i].y, false, viz);

    size_t next = (i + 1) % footprint.size();
    if (!costmap->getCostmap()->worldToMap(footprint[next].x, footprint[next].y, x1, y1))
    {
      RCLCPP_INFO(LOGGER, "Footprint point %lu is off the costmap.", next);
      addPointMarker(footprint[next].x, footprint[next].y, true, viz);
      return true;
    }
    addPointMarker(footprint[next].x, footprint[next].y, false, viz);

    for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
      unsigned cost = costmap->getCostmap()->getCost(line.getX(), line.getY());
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        RCLCPP_INFO(LOGGER, "Collision along footprint edge at map cell: [%u, %u] cost: %.2u",
                    line.getX(), line.getY(), cost);
        return true;
      }
    }
  }
  return false;
}

// ---------- NEW: curvature estimate (base_link frame poses) ----------
static double estimateKappa(const std::vector<geometry_msgs::msg::PoseStamped>& poses, int i) {
  if (poses.empty()) return 0.0;
  int i0 = std::max(0, i - 1);
  int i2 = std::min((int)poses.size() - 1, i + 1);
  auto p0 = poses[i0].pose.position;
  auto p1 = poses[i].pose.position;
  auto p2 = poses[i2].pose.position;

  double x1 = p0.x, y1 = p0.y, x2 = p1.x, y2 = p1.y, x3 = p2.x, y3 = p2.y;
  double a = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2;

  double d1 = std::hypot(x2 - x1, y2 - y1);
  double d2 = std::hypot(x3 - x2, y3 - y2);
  double d3 = std::hypot(x1 - x3, y1 - y3);
  double s = (d1 + d2 + d3) * 0.5;
  double area_sq = std::max(0.0, s*(s-d1)*(s-d2)*(s-d3));
  double area = std::sqrt(std::max(0.0, area_sq));
  double R = (area > 1e-9) ? (d1*d2*d3) / (4.0*area) : 1e9;
  double kappa = (R > 1e-6) ? (1.0 / R) : 0.0;

  double th1 = std::atan2(y2 - y1, x2 - x1);
  double th2 = std::atan2(y3 - y2, x3 - x2);
  double dth = angles::shortest_angular_distance(th1, th2);
  return (dth >= 0.0 ? kappa : -kappa);
}

// ---------- NEW: adaptive lookahead ----------
static double computeAdaptiveLookahead(double v, double kappa, double dt,
                                       double L0, double beta, double gamma, double p,
                                       double Lmin, double Lmax, double rate_lim,
                                       double L_prev, double odom_delay)
{
  const double L0_eff = L0 + std::max(0.0, v) * std::max(0.0, odom_delay);
  const double Lv = L0_eff + beta * std::max(0.0, v);
  const double fk = 1.0 / std::pow(1.0 + gamma * std::abs(kappa), p);
  double L_des = clampv(Lv * fk, Lmin, Lmax);

  const double max_step = rate_lim * std::max(1e-3, dt);
  if (L_des > L_prev) L_des = std::min(L_des, L_prev + max_step);
  else                L_des = std::max(L_des, L_prev - max_step);
  return L_des;
}

// ========================== GracefulControllerROS ==========================

GracefulControllerROS::GracefulControllerROS()
: initialized_(false), has_new_path_(false), collision_points_(nullptr), goal_achieved_(false)
{
  RCLCPP_INFO(LOGGER, "GracefulControllerROS constructor.");
}

GracefulControllerROS::~GracefulControllerROS()
{
  if (collision_points_) delete collision_points_;
}

void GracefulControllerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &weak_node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  using std::placeholders::_1;
  if (initialized_) return;

  node_ = weak_node;
  buffer_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = std::move(name);

  auto node = node_.lock();
  if (!node) throw std::runtime_error{"Failed to lock node"};

  param_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&GracefulControllerROS::onParameterChange, this, std::placeholders::_1));
  clock_ = node->get_clock();

  // ---- existing params (unchanged excerpts, keep yours) ----
  declare_parameter_if_not_declared(node, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".min_vel_x", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".min_in_place_vel_theta", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name_ + ".min_x_to_max_theta_scale_factor", rclcpp::ParameterValue(100.0));
  declare_parameter_if_not_declared(node, name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(node, name_ + ".acc_dt", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".decel_lim_x", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".max_lookahead", rclcpp::ParameterValue(1.0));   // kept for back-compat (not used for selection)
  declare_parameter_if_not_declared(node, name_ + ".min_lookahead", rclcpp::ParameterValue(0.25));  // kept for back-compat (not used for selection)
  declare_parameter_if_not_declared(node, name_ + ".initial_rotate_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name_ + ".prefer_final_rotation", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".compute_orientations", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".use_orientation_filter", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".yaw_filter_tolerance", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(node, name_ + ".yaw_gap_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".yaw_slowing_factor", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".latch_xy_goal_tolerance", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".publish_collision_points", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".k1", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name_ + ".k2", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".beta", rclcpp::ParameterValue(0.4));   // legacy, unused now
  declare_parameter_if_not_declared(node, name_ + ".lambda", rclcpp::ParameterValue(2.0)); // legacy, unused now
  declare_parameter_if_not_declared(node, name_ + ".scaling_vel_x", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name_ + ".scaling_factor", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name_ + ".scaling_step", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".backward_motion_available", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".backwards_check_yaw_tolerance", rclcpp::ParameterValue(0.34));
  declare_parameter_if_not_declared(node, name_ + ".ignore_orientation_distance", rclcpp::ParameterValue(0.10));

  // ---- NEW params for adaptive lookahead & physics caps ----
  declare_parameter_if_not_declared(node, name_ + ".L0", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".beta_la", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".gamma_la", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".p_la", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, name_ + ".Lmin", rclcpp::ParameterValue(0.20));
  declare_parameter_if_not_declared(node, name_ + ".Lmax", rclcpp::ParameterValue(1.00));
  declare_parameter_if_not_declared(node, name_ + ".lookahead_rate_limit", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name_ + ".a_lat_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".omega_max_sust", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, name_ + ".odom_delay", rclcpp::ParameterValue(0.0));

  // ---- read params ----
  node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(name_ + ".min_in_place_vel_theta", min_in_place_vel_theta_);
  node->get_parameter(name_ + ".min_x_to_max_theta_scale_factor", max_x_to_max_theta_scale_factor_);
  node->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(name_ + ".acc_dt", acc_dt_);
  node->get_parameter(name_ + ".decel_lim_x", decel_lim_x_);
  node->get_parameter(name_ + ".max_lookahead", max_lookahead_);   // legacy
  node->get_parameter(name_ + ".min_lookahead", min_lookahead_);   // legacy
  node->get_parameter(name_ + ".initial_rotate_tolerance", initial_rotate_tolerance_);
  node->get_parameter(name_ + ".prefer_final_rotation", prefer_final_rotation_);
  node->get_parameter(name_ + ".compute_orientations", compute_orientations_);
  node->get_parameter(name_ + ".use_orientation_filter", use_orientation_filter_);
  node->get_parameter(name_ + ".yaw_filter_tolerance", yaw_filter_tolerance_);
  node->get_parameter(name_ + ".yaw_gap_tolerance", yaw_gap_tolerance_);
  node->get_parameter(name_ + ".yaw_slowing_factor", yaw_slowing_factor_);
  node->get_parameter(name_ + ".latch_xy_goal_tolerance", latch_xy_goal_tolerance_);
  node->get_parameter(name_ + ".scaling_vel_x", scaling_vel_x_);
  node->get_parameter(name_ + ".scaling_factor", scaling_factor_);
  node->get_parameter(name_ + ".scaling_step", scaling_step_);
  node->get_parameter(name_ + ".backward_motion_available", backward_motion_available_);
  node->get_parameter(name_ + ".backwards_check_yaw_tolerance", backwards_check_yaw_tolerance_);
  node->get_parameter(name_ + ".ignore_orientation_distance", ignore_orientation_distance_);

  node->get_parameter(name_ + ".L0", L0_);
  node->get_parameter(name_ + ".beta_la", beta_la_);
  node->get_parameter(name_ + ".gamma_la", gamma_la_);
  node->get_parameter(name_ + ".p_la", p_la_);
  node->get_parameter(name_ + ".Lmin", Lmin_);
  node->get_parameter(name_ + ".Lmax", Lmax_);
  node->get_parameter(name_ + ".lookahead_rate_limit", lookahead_rate_limit_);
  node->get_parameter(name_ + ".a_lat_max", a_lat_max_);
  node->get_parameter(name_ + ".omega_max_sust", omega_max_sust_);
  node->get_parameter(name_ + ".odom_delay", odom_delay_);

  resolution_ = costmap_ros_->getCostmap()->getResolution();

  // defaults / guards
  if (max_x_to_max_theta_scale_factor_ < 0.001) max_x_to_max_theta_scale_factor_ = 100.0;
  if (decel_lim_x_ < 0.001) decel_lim_x_ = acc_lim_x_;

  // Limit maximum angular velocity proportional to linear
  max_vel_x_limited_ = max_vel_x_;
  max_vel_theta_limited_ = std::min(max_vel_theta_, max_vel_x_limited_ * max_x_to_max_theta_scale_factor_);

  // Publishers
  global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan", 1);
  target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/target_pose", 1);

  // Backward motion subscriber (unchanged)
  if (backward_motion_available_) {
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option; sub_option.callback_group = callback_group_;
    robot_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot_pose", 10, std::bind(&GracefulControllerROS::robot_pose_callback, this, std::placeholders::_1), sub_option);
  }

  bool publish_collision_points;
  node->get_parameter(name_ + ".publish_collision_points", publish_collision_points);
  if (publish_collision_points) {
    collision_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/collision_points", 1);
    collision_points_ = new visualization_msgs::msg::MarkerArray();
  }

  // ---------- construct core controller with physics caps ----------
  double k1, k2, beta_legacy, lambda_legacy;  // beta/lambda legacy, unused
  node->get_parameter(name_ + ".k1", k1);
  node->get_parameter(name_ + ".k2", k2);
  node->get_parameter(name_ + ".beta", beta_legacy);
  node->get_parameter(name_ + ".lambda", lambda_legacy);

  controller_ = std::make_shared<GracefulController>(
    k1, k2, min_vel_x_, max_vel_x_, decel_lim_x_, max_vel_theta_, a_lat_max_, omega_max_sust_);

  backward_motion_ = false;
  L_prev_ = clampv(L0_, Lmin_, Lmax_);
  initialized_ = true;
}

void GracefulControllerROS::cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  target_pose_pub_.reset();
  collision_points_pub_.reset();
}

void GracefulControllerROS::activate()
{
  global_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  target_pose_pub_->on_activate();
  if (collision_points_) collision_points_pub_->on_activate();
  has_new_path_ = false;
  goal_achieved_ = false;
}

void GracefulControllerROS::deactivate()
{
  global_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  target_pose_pub_->on_deactivate();
  if (collision_points_) collision_points_pub_->on_deactivate();
}

void GracefulControllerROS::robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  robot_pose_ = *msg;
  robot_pose_received_ = true;
}

geometry_msgs::msg::TwistStamped GracefulControllerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  geometry_msgs::msg::TwistStamped cmd_vel;

  if (!initialized_) return cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  // Publish global plan (debug)
  global_plan_pub_->publish(global_plan_);

  // Transforms
  geometry_msgs::msg::TransformStamped plan_to_robot, robot_to_costmap_transform_;
  try {
    plan_to_robot = buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                             global_plan_.header.frame_id,
                                             tf2::TimePointZero);
    robot_to_costmap_transform_ = buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                                           costmap_ros_->getBaseFrameID(),
                                                           tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_INFO(LOGGER, "TF exception: %s", ex.what());
    return cmd_vel;
  }

  // Goal pose in robot frame
  geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
  tf2::doTransform(goal_pose, goal_pose, plan_to_robot);

  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  goal_checker->getTolerances(pose_tol, vel_tol);

  const double dist_to_goal = std::hypot(goal_pose.pose.position.x, goal_pose.pose.position.y);

  // goal handling (unchanged behavior)
  if (dist_to_goal < pose_tol.position.x || goal_tolerance_met_)
  {
    if (!goal_achieved_) {
      if (prefer_final_rotation_) {
        // try in-place rotate with collision check
        goal_tolerance_met_ = latch_xy_goal_tolerance_;
        rotateTowards(tf2::getYaw(goal_pose.pose.orientation), velocity, cmd_vel, true);

        // simulate rotation path for collisions
        double yaw_delta = tf2::getYaw(goal_pose.pose.orientation);
        size_t steps = std::max<size_t>(1, static_cast<size_t>(std::fabs(yaw_delta) / 0.1));
        for (size_t i = 1; i <= steps; ++i) {
          double step = static_cast<double>(i) / static_cast<double>(steps);
          double yaw = step * yaw_delta;
          if (isColliding(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw,
                          costmap_ros_, collision_points_)) {
            cmd_vel.twist = geometry_msgs::msg::Twist();
            return cmd_vel;
          }
        }
        return cmd_vel;
      } else {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        goal_achieved_ = true;
        global_plan_.poses.clear();
        return cmd_vel;
      }
    } else {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }
  } else {
    goal_achieved_ = false;
  }

  // accel/decel limits for current step
  max_vel_x_limited_ = max_vel_x_;
  double max_vel_x;
  if (velocity.linear.x > max_vel_x_limited_) {
    double dec_max = velocity.linear.x - (decel_lim_x_ * acc_dt_);
    max_vel_x = std::max({max_vel_x_limited_, dec_max, min_vel_x_});
  } else {
    double acc_max = velocity.linear.x + (acc_lim_x_ * acc_dt_);
    max_vel_x = std::clamp(acc_max, min_vel_x_, max_vel_x_limited_);
  }
  max_vel_theta_limited_ = std::min(max_vel_theta_, max_vel_x_limited_ * max_x_to_max_theta_scale_factor_);

  // transform plan into base_link
  std::vector<geometry_msgs::msg::PoseStamped> target_poses;
  std::vector<double> target_distances;
  target_poses.reserve(global_plan_.poses.size());
  target_distances.reserve(global_plan_.poses.size());
  for (auto &pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped tp;
    tf2::doTransform(pose, tp, plan_to_robot);
    target_poses.push_back(tp);
  }
  computeDistanceAlongPath(target_poses, target_distances);

  // ---------- ADAPTIVE LOOKAHEAD: choose one target at distance L ----------
  int i_closest = std::min_element(target_distances.begin(), target_distances.end()) - target_distances.begin();
  double kappa_here = estimateKappa(target_poses, i_closest);

  const double dt = acc_dt_;
  const double v_meas = velocity.linear.x;
  double L = computeAdaptiveLookahead(
      v_meas, kappa_here, dt,
      L0_, beta_la_, gamma_la_, p_la_,
      Lmin_, Lmax_, lookahead_rate_limit_,
      L_prev_, odom_delay_);
  L_prev_ = L;

  // pick index closest to L
  int idx = i_closest;
  double best_err = 1e9;
  for (int i = 0; i < (int)target_distances.size(); ++i) {
    double e = std::abs(target_distances[i] - L);
    if (e < best_err) { best_err = e; idx = i; }
  }
  geometry_msgs::msg::PoseStamped target_pose = target_poses[idx];

  // ---------- PHYSICS CAPS for velocity ladder ----------
  const double kappa_for_caps = std::max(1e-6, std::abs(estimateKappa(target_poses, idx)));
  const double v_curve = std::sqrt(a_lat_max_ / (kappa_for_caps + 1e-6));
  const double v_yaw   = omega_max_sust_ / (kappa_for_caps + 1e-6);
  const double v_phys_cap = std::min({ max_vel_x, v_curve, v_yaw });

  // velocity ladder with simulation + costmap safety gate
  double sim_velocity = v_phys_cap;
  do {
    controller_->setVelocityLimits(min_vel_x_, sim_velocity, max_vel_theta_limited_);
    if (simulate(target_pose, velocity, cmd_vel)) {
      if (dist_to_goal < ignore_orientation_distance_) {
        cmd_vel.twist.angular.z = 0.0;
      }
      return cmd_vel;
    }
    sim_velocity = std::max(scaling_vel_x_, sim_velocity - scaling_step_);
  } while (sim_velocity > scaling_vel_x_ - 1e-6);

  // No reachable pose at any speed -> stop & throw for BT
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  throw std::runtime_error("No reachable pose found. Aborting navigation from BT.");
}

bool GracefulControllerROS::simulate(
    const geometry_msgs::msg::PoseStamped &target_pose,
    const geometry_msgs::msg::Twist &velocity,
    geometry_msgs::msg::TwistStamped &cmd_vel)
{
  // (Mostly unchanged; keeps your footprint scaling + costmap collision checks)
  nav_msgs::msg::Path simulated_path;
  simulated_path.header.frame_id = "base_link";
  bool sim_initial_rotation_ = has_new_path_ && initial_rotate_tolerance_ > 0.0;

  if (collision_points_) collision_points_->markers.resize(0);

  geometry_msgs::msg::PoseStamped target = target_pose;
  while (true)
  {
    geometry_msgs::msg::PoseStamped error = target;
    double error_angle = tf2::getYaw(error.pose.orientation);

    if (!simulated_path.poses.empty())
    {
      double x = error.pose.position.x - simulated_path.poses.back().pose.position.x;
      double y = error.pose.position.y - simulated_path.poses.back().pose.position.y;
      double theta = -tf2::getYaw(simulated_path.poses.back().pose.orientation);

      error.pose.position.x = x * std::cos(theta) - y * std::sin(theta);
      error.pose.position.y = y * std::cos(theta) + x * std::sin(theta);
      error_angle += theta;
      error.pose.orientation.z = std::sin(error_angle / 2.0);
      error.pose.orientation.w = std::cos(error_angle / 2.0);
    }

    double vel_x = 0.0, vel_th = 0.0;
    if (sim_initial_rotation_)
    {
      geometry_msgs::msg::TwistStamped rotation;
      double yaw_error = rotateTowards(error, velocity, rotation, prefer_final_rotation_);

      if (std::fabs(yaw_error) < initial_rotate_tolerance_)
      {
        if (simulated_path.poses.empty()) has_new_path_ = false;
        sim_initial_rotation_ = false;
      }
      vel_x = rotation.twist.linear.x;
      vel_th = rotation.twist.angular.z;

      if (prefer_final_rotation_) {
        cmd_vel.twist.linear.x = vel_x;
        cmd_vel.twist.angular.z = vel_th;
      } else {
        cmd_vel.twist.linear.x = vel_x;
        cmd_vel.twist.angular.z = 0.0;
      }
    }

    if (!sim_initial_rotation_)
    {
      if (!controller_->approach(error.pose.position.x, error.pose.position.y, error_angle,
                                 vel_x, vel_th, backward_motion_))
      {
        return false;
      }
    }

    if (simulated_path.poses.empty())
    {
      cmd_vel.twist.linear.x = vel_x;
      cmd_vel.twist.angular.z = vel_th;
    }
    else if (std::hypot(error.pose.position.x, error.pose.position.y) < resolution_)
    {
      local_plan_pub_->publish(simulated_path);
      target_pose_pub_->publish(target);
      if (collision_points_ && !collision_points_->markers.empty()) {
        collision_points_->markers[0].header.stamp = clock_->now();
        collision_points_pub_->publish(*collision_points_);
      }
      return true;
    }

    // Forward simulate one step
    geometry_msgs::msg::PoseStamped next_pose;
    next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
    if (simulated_path.poses.empty()) next_pose.pose.orientation.w = 1.0;
    else next_pose = simulated_path.poses.back();

    const double dt = (vel_x > 0.0) ? (resolution_ / vel_x) : 0.1;
    double yaw = tf2::getYaw(next_pose.pose.orientation);
    next_pose.pose.position.x += dt * vel_x * std::cos(yaw);
    next_pose.pose.position.y += dt * vel_x * std::sin(yaw);
    yaw += dt * vel_th;
    next_pose.pose.orientation.z = std::sin(yaw / 2.0);
    next_pose.pose.orientation.w = std::cos(yaw / 2.0);
    simulated_path.poses.push_back(next_pose);

    // Footprint scaling with speed (unchanged)
    double footprint_scaling = 1.0;
    if (vel_x > scaling_vel_x_) {
      double ratio = max_vel_x_limited_ - scaling_vel_x_;
      if (ratio > 0.0) {
        ratio = (vel_x - scaling_vel_x_) / ratio;
        footprint_scaling += ratio * scaling_factor_;
      }
    }

    // Collision check in costmap frame
    geometry_msgs::msg::PoseStamped costmap_pose = next_pose;
    geometry_msgs::msg::TransformStamped robot_to_costmap_transform_;
    try {
      robot_to_costmap_transform_ = buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                                             costmap_ros_->getBaseFrameID(),
                                                             tf2::TimePointZero);
    } catch (...) {
      return false;
    }
    tf2::doTransform(costmap_pose, costmap_pose, robot_to_costmap_transform_);

    if (isColliding(costmap_pose.pose.position.x, costmap_pose.pose.position.y, tf2::getYaw(costmap_pose.pose.orientation),
                    costmap_ros_, collision_points_, footprint_scaling))
    {
      if (collision_points_ && !collision_points_->markers.empty()) {
        collision_points_->markers[0].header.stamp = clock_->now();
        collision_points_pub_->publish(*collision_points_);
      }
      return false;
    }
  }

  return false; // not reached
}

void GracefulControllerROS::setPlan(const nav_msgs::msg::Path &path)
{
  if (!initialized_) return;

  nav_msgs::msg::Path oriented_plan = compute_orientations_ ? addOrientations(path) : path;
  nav_msgs::msg::Path filtered_plan = use_orientation_filter_ ?
                                      applyOrientationFilter(oriented_plan, yaw_filter_tolerance_, yaw_gap_tolerance_) :
                                      oriented_plan;

  if (backward_motion_available_)
  {
    robot_pose_received_ = false;
    callback_group_executor_.spin_some();
    double robot_orientation = 0.0;
    if (robot_pose_received_) {
      tf2::Quaternion q; tf2::fromMsg(robot_pose_.pose.orientation, q);
      double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_orientation = angles::normalize_angle(yaw);
    }

    tf2::Quaternion qpath; tf2::fromMsg(filtered_plan.poses[0].pose.orientation, qpath);
    double roll, pitch, yaw; tf2::Matrix3x3(qpath).getRPY(roll, pitch, yaw);
    const double path_orientation = angles::normalize_angle(yaw);

    const double diff_angle = std::fabs(angles::normalize_angle(path_orientation - robot_orientation));
    const double angle_tol = std::fabs(backwards_check_yaw_tolerance_);

    backward_motion_ = (diff_angle > (M_PI - angle_tol) && diff_angle < (M_PI + angle_tol));
    global_plan_ = filtered_plan;
  }
  else
  {
    backward_motion_ = false;
    global_plan_ = filtered_plan;
  }

  has_new_path_ = true;
  goal_tolerance_met_ = false;
  goal_achieved_ = false;
}

double GracefulControllerROS::rotateTowards(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    geometry_msgs::msg::TwistStamped &cmd_vel,
    bool perform_rotation)
{
  if (!perform_rotation) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return 0.0;
  }

  double yaw = 0.0;
  if (std::hypot(pose.pose.position.x, pose.pose.position.y) > 0.5)
    yaw = std::atan2(pose.pose.position.y, pose.pose.position.x);
  else
    yaw = tf2::getYaw(pose.pose.orientation);

  rotateTowards(yaw, velocity, cmd_vel, perform_rotation);
  return yaw;
}

void GracefulControllerROS::rotateTowards(
    double yaw,
    const geometry_msgs::msg::Twist &velocity,
    geometry_msgs::msg::TwistStamped &cmd_vel,
    bool perform_rotation)
{
  if (!perform_rotation) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return;
  }

  double max_vel_th = max_vel_theta_limited_;
  if (acc_dt_ > 0.0)
  {
    double abs_vel = std::fabs(velocity.angular.z);
    double acc_limited = abs_vel + (acc_lim_theta_ * acc_dt_);
    max_vel_th = std::min(max_vel_th, acc_limited);
    max_vel_th = std::max(max_vel_th, min_in_place_vel_theta_);
  }

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = sign(yaw) * std::min(max_vel_th,
                              std::max(min_in_place_vel_theta_, std::fabs(yaw * yaw_slowing_factor_)));
}

void GracefulControllerROS::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  if (speed_limit == 0.0) {
    max_vel_x_limited_ = max_vel_x_;
  } else {
    if (percentage)
      max_vel_x_limited_ = std::max((speed_limit / 100.0) * max_vel_x_, min_vel_x_);
    else
      max_vel_x_limited_ = std::max(speed_limit, min_vel_x_);
  }
  max_vel_theta_limited_ =
      std::min(max_vel_theta_, max_vel_x_limited_ * max_x_to_max_theta_scale_factor_);
}

// unchanged
void computeDistanceAlongPath(const std::vector<geometry_msgs::msg::PoseStamped> &poses,
                              std::vector<double> &distances)
{
  distances.resize(poses.size());
  for (size_t i = 0; i < poses.size(); ++i)
    distances[i] = std::hypot(poses[i].pose.position.x, poses[i].pose.position.y);

  auto closest = std::min_element(std::begin(distances), std::end(distances));
  for (size_t i = std::distance(std::begin(distances), closest) + 1; i < distances.size(); ++i) {
    distances[i] = distances[i - 1] +
                   std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x,
                              poses[i].pose.position.y - poses[i - 1].pose.position.y);
  }
}

rcl_interfaces::msg::SetParametersResult GracefulControllerROS::onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  for (const auto & param : parameters) {
    if (param.get_name() == name_ + ".max_vel_x") {
      max_vel_x_ = param.as_double();
      max_vel_x_limited_ = max_vel_x_;
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

} // namespace graceful_controller

PLUGINLIB_EXPORT_CLASS(graceful_controller::GracefulControllerROS, nav2_core::Controller)