/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  ... [License Text] ...
*
* Author: Eitan Marder-Eppstein, Michael Ferguson
*********************************************************************/

#include <algorithm>
#include <cmath>
#include <mutex>

#include <angles/angles.h>
#include <nav_2d_utils/parameters.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/line_iterator.hpp>
#include <rclcpp/logging.hpp>
#include "graceful_controller_ros/graceful_controller_ros.hpp"

using nav2_util::declare_parameter_if_not_declared;
using rclcpp_lifecycle::LifecyclePublisher;
 
namespace graceful_controller
{
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("graceful_controller");

  double sign(double x)
  {
    return x < 0.0 ? -1.0 : 1.0;
  }

  /**
  * @brief Collision check the robot pose
  * @param x The robot x coordinate in costmap.global frame
  * @param y The robot y coordinate in costmap.global frame
  * @param theta The robot rotation in costmap.global frame
  * @param viz Optional message for visualizing collisions
  * @param inflation Ratio to expand the footprint
  */
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
    for (size_t i = 0; i < spec.size(); ++i)
    {
      spec[i].x *= inflation;
      spec[i].y *= inflation;
    }

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

    // Check each edge of the footprint for collisions
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
          RCLCPP_INFO(LOGGER, "Collision detected along footprint edge at map cell: [%u, %u] with cost: %.2u",
                      line.getX(), line.getY(), cost);
          return true;
        }
      }
    }

    return false;
  }
 
  GracefulControllerROS::GracefulControllerROS() : initialized_(false), has_new_path_(false), collision_points_(nullptr), goal_achieved_(false)
  {
    RCLCPP_INFO(LOGGER, "GracefulControllerROS constructor called.");
  }

  GracefulControllerROS::~GracefulControllerROS()
  {
    RCLCPP_INFO(LOGGER, "GracefulControllerROS destructor called.");
    if (collision_points_)
    {
      delete collision_points_;
    }
  }
 
  void GracefulControllerROS::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &weak_node,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    RCLCPP_INFO(LOGGER, "Configuring GracefulControllerROS with name: %s", name.c_str());

    using std::placeholders::_1;
    if (initialized_)
    {
      RCLCPP_INFO(LOGGER, "This planner has already been initialized, doing nothing.");
      return;
    }

    // Save important things
    node_ = weak_node;
    buffer_ = tf;
    costmap_ros_ = costmap_ros;
    name_ = name;

    auto node = node_.lock();
    if (!node)
    {
      RCLCPP_INFO(LOGGER, "Failed to lock node in configure.");
      throw std::runtime_error{"Failed to lock node"};
    }

    param_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&GracefulControllerROS::onParameterChange, this, std::placeholders::_1)
    );

    clock_ = node->get_clock();

    // Setup parameters
    declare_parameter_if_not_declared(node, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, name_ + ".min_vel_x", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, name_ + ".min_in_place_vel_theta", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, name_ + ".min_x_to_max_theta_scale_factor", rclcpp::ParameterValue(100.0));
    declare_parameter_if_not_declared(node, name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
    declare_parameter_if_not_declared(node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
    declare_parameter_if_not_declared(node, name_ + ".acc_dt", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(node, name_ + ".decel_lim_x", rclcpp::ParameterValue(1.0)); // Set a positive value
    declare_parameter_if_not_declared(node, name_ + ".max_lookahead", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, name_ + ".min_lookahead", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(node, name_ + ".curve_detection_distance", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, name_ + ".curve_turn_threshold", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(node, name_ + ".curve_turn_full_threshold", rclcpp::ParameterValue(1.05));
    declare_parameter_if_not_declared(node, name_ + ".curve_min_lookahead_distance", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, name_ + ".curve_min_speed", rclcpp::ParameterValue(0.25));
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
    declare_parameter_if_not_declared(node, name_ + ".beta", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, name_ + ".lambda", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, name_ + ".scaling_vel_x", rclcpp::ParameterValue(0.3)); // Lower to initiate slowdown earlier
    declare_parameter_if_not_declared(node, name_ + ".scaling_factor", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, name_ + ".scaling_step", rclcpp::ParameterValue(0.05));

    declare_parameter_if_not_declared(node, name_ + ".curve_lookahead_decrease_rate", rclcpp::ParameterValue(0.8));
    declare_parameter_if_not_declared(node, name_ + ".curve_lookahead_increase_rate", rclcpp::ParameterValue(0.4));

    // Params for backwards motion
    declare_parameter_if_not_declared(node, name_ + ".backward_motion_available", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, name_ + ".backwards_check_yaw_tolerance", rclcpp::ParameterValue(0.34));

    declare_parameter_if_not_declared(node, name_ + ".ignore_orientation_distance", rclcpp::ParameterValue(0.10));

    // Retrieve parameters
    node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
    node->get_parameter(name_ + ".min_vel_x", min_vel_x_);
    node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
    node->get_parameter(name_ + ".min_in_place_vel_theta", min_in_place_vel_theta_);
    node->get_parameter(name_ + ".min_x_to_max_theta_scale_factor", max_x_to_max_theta_scale_factor_);
    node->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
    node->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
    node->get_parameter(name_ + ".acc_dt", acc_dt_);
    node->get_parameter(name_ + ".decel_lim_x", decel_lim_x_);
    node->get_parameter(name_ + ".max_lookahead", max_lookahead_);
    node->get_parameter(name_ + ".min_lookahead", min_lookahead_);
    node->get_parameter(name_ + ".curve_detection_distance", curve_detection_distance_);
    node->get_parameter(name_ + ".curve_turn_threshold", curve_turn_threshold_);
    node->get_parameter(name_ + ".curve_turn_full_threshold", curve_turn_full_threshold_);
    node->get_parameter(name_ + ".curve_min_lookahead_distance", curve_min_lookahead_distance_);
    node->get_parameter(name_ + ".curve_min_speed", curve_min_speed_);
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

    node->get_parameter(name_ + ".curve_lookahead_decrease_rate", curve_lookahead_decrease_rate_);
    node->get_parameter(name_ + ".curve_lookahead_increase_rate", curve_lookahead_increase_rate_);

    // Params for backwards motion
    node->get_parameter(name_ + ".backward_motion_available", backward_motion_available_);
    node->get_parameter(name_ + ".backwards_check_yaw_tolerance", backwards_check_yaw_tolerance_);

    node->get_parameter(name_ + ".ignore_orientation_distance", ignore_orientation_distance_);

    // Log loaded parameters
    RCLCPP_INFO(LOGGER, "Parameters loaded:");
    RCLCPP_INFO(LOGGER, "  max_vel_x: %.2f", max_vel_x_);
    RCLCPP_INFO(LOGGER, "  min_vel_x: %.2f", min_vel_x_);
    RCLCPP_INFO(LOGGER, "  max_vel_theta: %.2f", max_vel_theta_);
    RCLCPP_INFO(LOGGER, "  min_in_place_vel_theta: %.2f", min_in_place_vel_theta_);
    RCLCPP_INFO(LOGGER, "  max_x_to_max_theta_scale_factor: %.2f", max_x_to_max_theta_scale_factor_);
    RCLCPP_INFO(LOGGER, "  acc_lim_x: %.2f", acc_lim_x_);
    RCLCPP_INFO(LOGGER, "  acc_lim_theta: %.2f", acc_lim_theta_);
    RCLCPP_INFO(LOGGER, "  acc_dt: %.2f", acc_dt_);
    RCLCPP_INFO(LOGGER, "  decel_lim_x: %.2f", decel_lim_x_);
    RCLCPP_INFO(LOGGER, "  max_lookahead: %.2f", max_lookahead_);
    RCLCPP_INFO(LOGGER, "  min_lookahead: %.2f", min_lookahead_);
    RCLCPP_INFO(LOGGER, "  curve_detection_distance: %.2f", curve_detection_distance_);
    RCLCPP_INFO(LOGGER, "  curve_turn_threshold: %.2f", curve_turn_threshold_);
    RCLCPP_INFO(LOGGER, "  curve_turn_full_threshold: %.2f", curve_turn_full_threshold_);
    RCLCPP_INFO(LOGGER, "  curve_min_lookahead_distance: %.2f", curve_min_lookahead_distance_);
    RCLCPP_INFO(LOGGER, "  curve_min_speed: %.2f", curve_min_speed_);
    RCLCPP_INFO(LOGGER, "  initial_rotate_tolerance: %.2f", initial_rotate_tolerance_);
    RCLCPP_INFO(LOGGER, "  prefer_final_rotation: %s", prefer_final_rotation_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(LOGGER, "  compute_orientations: %s", compute_orientations_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(LOGGER, "  use_orientation_filter: %s", use_orientation_filter_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(LOGGER, "  yaw_filter_tolerance: %.2f", yaw_filter_tolerance_);
    RCLCPP_INFO(LOGGER, "  yaw_gap_tolerance: %.2f", yaw_gap_tolerance_);
    RCLCPP_INFO(LOGGER, "  yaw_slowing_factor: %.2f", yaw_slowing_factor_);
    RCLCPP_INFO(LOGGER, "  latch_xy_goal_tolerance: %s", latch_xy_goal_tolerance_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(LOGGER, "  scaling_vel_x: %.2f", scaling_vel_x_);
    RCLCPP_INFO(LOGGER, "  scaling_factor: %.2f", scaling_factor_);
    RCLCPP_INFO(LOGGER, "  scaling_step: %.2f", scaling_step_);
    RCLCPP_INFO(LOGGER, "  backward_motion_available: %s", backward_motion_available_ ? "TRUE" : "FALSE");
    RCLCPP_INFO(LOGGER, "  backwards_check_yaw_tolerance: %.2f", backwards_check_yaw_tolerance_);
    RCLCPP_INFO(LOGGER, "  ignore_orientation_distance: %.2f", ignore_orientation_distance_);

    // Retrieve additional parameters
    resolution_ = costmap_ros_->getCostmap()->getResolution();
    double k1, k2, beta, lambda;
    node->get_parameter(name_ + ".k1", k1);
    node->get_parameter(name_ + ".k2", k2);
    node->get_parameter(name_ + ".beta", beta);
    node->get_parameter(name_ + ".lambda", lambda);

    RCLCPP_INFO(LOGGER, "Controller parameters: k1=%.2f, k2=%.2f, beta=%.2f, lambda=%.2f", k1, k2, beta, lambda);
                
    //Parameter sanity checks, if not satisfied, throw and prevent from getting to READY state
    if (k1 == 0.0 && k2 == 0.0)
    {
    RCLCPP_ERROR(LOGGER, "GracefulController -> configure failed: k1 and k2 cannot both be zero.");
    throw std::runtime_error("Invalid parameters: k1 and k2 both zero");
    return;
    }

    // Set backward motion
    backward_motion_ = false;
    RCLCPP_INFO(LOGGER, "Initial backward_motion_ set to FALSE.");

    // Set initial velocity limit
    max_vel_x_limited_ = max_vel_x_;
    RCLCPP_INFO(LOGGER, "Initial max_vel_x_limited set to %.2f", max_vel_x_limited_);

    if (max_x_to_max_theta_scale_factor_ < 0.001)
    {
      // If max_x_to_max_theta_scale_factor not specified, use a high value so it has no functional impact
      max_x_to_max_theta_scale_factor_ = 100.0;
      RCLCPP_INFO(LOGGER, "max_x_to_max_theta_scale_factor was too low. Resetting to 100.0.");
    }

    // Limit maximum angular velocity proportional to maximum linear velocity
    max_vel_theta_limited_ = max_vel_x_limited_ * max_x_to_max_theta_scale_factor_;
    max_vel_theta_limited_ = std::min(max_vel_theta_limited_, max_vel_theta_);
    RCLCPP_INFO(LOGGER, "max_vel_theta_limited set to %.2f", max_vel_theta_limited_);

    // Publishers (same topics as DWA/TrajRollout)
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/global_plan", 1);
    local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan", 1);
    target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/target_pose", 1);
    RCLCPP_INFO(LOGGER, "Publishers for global_plan, local_plan, and target_pose initialized.");

    // Subscriber Robot pose if backward motion is needed
    if (backward_motion_available_)
    {
      RCLCPP_INFO(LOGGER, "Backward motion is available. Setting up robot_pose subscriber.");
      callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

      callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

      rclcpp::SubscriptionOptions sub_option;
      sub_option.callback_group = callback_group_;

      robot_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          "robot_pose", 10, std::bind(&GracefulControllerROS::robot_pose_callback, this, _1), sub_option);
      RCLCPP_INFO(LOGGER, "robot_pose subscriber initialized.");
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Backward motion is not available. Skipping robot_pose subscriber setup.");
    }

    bool publish_collision_points;
    node->get_parameter(name_ + ".publish_collision_points", publish_collision_points);
    if (publish_collision_points)
    {
      // Create publisher
      collision_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/collision_points", 1);
      RCLCPP_INFO(LOGGER, "Collision points publisher initialized.");

      // Create message to publish
      collision_points_ = new visualization_msgs::msg::MarkerArray();
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Collision points publishing is disabled.");
    }

    if (decel_lim_x_ < 0.001)
    {
      // If decel limit not specified, use accel_limit
      decel_lim_x_ = acc_lim_x_;
      RCLCPP_INFO(LOGGER, "decel_lim_x was not set. Using acc_lim_x: %.2f", decel_lim_x_);
    }

    // Initialize the controller
    controller_ = std::make_shared<GracefulController>(k1,
                                                      k2,
                                                      min_vel_x_,
                                                      max_vel_x_,
                                                      decel_lim_x_,
                                                      max_vel_theta_,
                                                      beta,
                                                      lambda);
    RCLCPP_INFO(LOGGER, "GracefulController instance created.");

    initialized_ = true;
    RCLCPP_INFO(LOGGER, "GracefulControllerROS successfully configured and initialized.");
  }
 
  void GracefulControllerROS::cleanup()
  {
    RCLCPP_INFO(LOGGER, "Cleaning up GracefulControllerROS.");
    global_plan_pub_.reset();
    local_plan_pub_.reset();
    target_pose_pub_.reset();
    collision_points_pub_.reset();
    RCLCPP_INFO(LOGGER, "Publishers reset.");
  }

  void GracefulControllerROS::activate()
  {
    RCLCPP_INFO(LOGGER, "Activating GracefulControllerROS.");
    global_plan_pub_->on_activate();
    local_plan_pub_->on_activate();
    target_pose_pub_->on_activate();
    if (collision_points_)
    {
      collision_points_pub_->on_activate();
      RCLCPP_INFO(LOGGER, "Collision points publisher activated.");
    }
    has_new_path_ = false;
    goal_achieved_ = false;

    filtered_lookahead_initialized_ = false;
    filtered_max_lookahead_ = max_lookahead_;
    RCLCPP_INFO(LOGGER, "GracefulControllerROS activated.");
  }

  void GracefulControllerROS::deactivate()
  {
    RCLCPP_INFO(LOGGER, "Deactivating GracefulControllerROS.");
    global_plan_pub_->on_deactivate();
    local_plan_pub_->on_deactivate();
    target_pose_pub_->on_deactivate();
    if (collision_points_)
    {
      collision_points_pub_->on_deactivate();
      RCLCPP_INFO(LOGGER, "Collision points publisher deactivated.");
    }
    RCLCPP_INFO(LOGGER, "GracefulControllerROS deactivated.");
  }
 
   // Callback for robot pose (used for backward motion)
  void GracefulControllerROS::robot_pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    robot_pose_ = *msg;
    robot_pose_received_ = true;
    RCLCPP_DEBUG(LOGGER, "Received robot pose callback.");
  }
 
  geometry_msgs::msg::TwistStamped GracefulControllerROS::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker)
  {
    //  RCLCPP_INFO(LOGGER, "Goal has already been achieved. Stopping the robot.");
    std::lock_guard<std::mutex> lock(config_mutex_);

    geometry_msgs::msg::TwistStamped cmd_vel;

    max_vel_x_limited_ = max_vel_x_;
    if (!initialized_)
    {
      RCLCPP_WARN(LOGGER, "Controller is not initialized, call configure() before using this planner");
      return cmd_vel;
    }

    // If goal is already achieved, ensure the robot is stopped
    if (goal_achieved_)
    {
      RCLCPP_INFO(LOGGER, "Goal has already been achieved. Stopping the robot.");
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;

      // Log the cmd_vel
      RCLCPP_INFO(LOGGER, "cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                  cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
      return cmd_vel;
    }

    // Set header
    cmd_vel.header.frame_id = robot_pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if (global_plan_.poses.empty())
    {
      RCLCPP_WARN(LOGGER, "computeVelocityCommands skipped -> global_plan_ is empty.");
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }

    // Publish the global plan
    global_plan_pub_->publish(global_plan_);

    // Get transforms
    geometry_msgs::msg::TransformStamped plan_to_robot;
    try
    {
      plan_to_robot = buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                              global_plan_.header.frame_id,
                                              tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_INFO(LOGGER, "Could not transform to %s: %s", costmap_ros_->getBaseFrameID().c_str(), ex.what());
      return cmd_vel;
    }

    try
    {
      robot_to_costmap_transform_ = buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                                            costmap_ros_->getBaseFrameID(),
                                                            tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_INFO(LOGGER, "Could not transform to %s: %s", costmap_ros_->getGlobalFrameID().c_str(), ex.what());
      return cmd_vel;
    }
 
    // Get the overall goal (in the robot frame)
    geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
    tf2::doTransform(goal_pose, goal_pose, plan_to_robot);

    // Get goal tolerances
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const double yaw_goal_tolerance = std::fabs(tf2::getYaw(pose_tolerance.orientation));

    // Compute distance to goal
    double dist_to_goal = std::hypot(goal_pose.pose.position.x, goal_pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "dist_to_goal: %.2f", dist_to_goal);

    const double final_yaw_error = angles::shortest_angular_distance(0.0, tf2::getYaw(goal_pose.pose.orientation));
    const double goal_yaw_alignment_distance = 1.0;
    const double max_moving_yaw_alignment_error = 0.3490658504;  // 20 deg

    // If we've reached the XY goal tolerance, decide whether to rotate or stop
    if (dist_to_goal < pose_tolerance.position.x || goal_tolerance_met_)
    {
      if (!goal_achieved_)
      {
        if (prefer_final_rotation_)
        {
          RCLCPP_INFO(LOGGER, "prefer_final_rotation is enabled. Attempting in-place rotation.");

          // Reached goal, latch if desired
          goal_tolerance_met_ = latch_xy_goal_tolerance_;

          // FINAL YAW TOLERANCE CHECK FROM GOAL CHECKER
          if (std::abs(final_yaw_error) <= yaw_goal_tolerance)
          {
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;

            goal_achieved_ = true;
            goal_tolerance_met_ = false;
            global_plan_.poses.clear();

            RCLCPP_INFO(LOGGER, "Goal achieved after final rotation. final_yaw_error=%.3f yaw_goal_tolerance=%.3f",
              final_yaw_error, yaw_goal_tolerance);

            return cmd_vel;
          }

          // Rotate only if outside yaw tolerance
          rotateTowards(final_yaw_error, velocity, cmd_vel, true);

          // Check for collisions between our current pose and goal
          double yaw_delta = final_yaw_error;
          size_t num_steps = static_cast<size_t>(fabs(yaw_delta) / 0.1);
          num_steps = std::max(static_cast<size_t>(1), num_steps);

          bool collision_free = true;
          for (size_t i = 1; i <= num_steps; ++i)
          {
            double step = static_cast<double>(i) / static_cast<double>(num_steps);
            double yaw = step * yaw_delta;

            if (isColliding(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw, costmap_ros_, collision_points_))
            {
              RCLCPP_INFO(LOGGER, "Unable to rotate in place due to collision at step %lu.", i);
              if (collision_points_ && !collision_points_->markers.empty())
              {
                collision_points_->markers[0].header.stamp = clock_->now();
                collision_points_pub_->publish(*collision_points_);
              }
              // Reset to zero velocity
              cmd_vel.twist = geometry_msgs::msg::Twist();
              collision_free = false;

              // Log the cmd_vel after collision detection
              RCLCPP_INFO(LOGGER, "Collision detected. cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                          cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
              break;
            }
          }
          if (collision_free)
          {
            RCLCPP_INFO(LOGGER, "No collisions detected during rotation. Executing rotation command.");
            RCLCPP_INFO(LOGGER, "cmd_vel Output -> linear.x: %.2f, angular.z: %.2f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
            return cmd_vel;
          }
          // Otherwise, fall through and try to get closer to goal in XY
          RCLCPP_INFO(LOGGER, "Collision detected during in-place rotation. Falling back to stopping.");
        }
        else
        {
          RCLCPP_INFO(LOGGER, "prefer_final_rotation is disabled. Stopping without rotating.");
          // Stop the robot by setting velocities to zero
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          goal_achieved_ = true; // Set the flag
          RCLCPP_INFO(LOGGER, "cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
          RCLCPP_INFO(LOGGER, "---- GOAL ACHIEVED ----");

          // Optionally clear the global plan to prevent further iterations
          global_plan_.poses.clear();
          return cmd_vel;
        }
      }
      else
      {
        // Goal already achieved, ensure velocities are zero
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        RCLCPP_INFO(LOGGER, "cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                    cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        return cmd_vel;
      }
    }
    else
    {
      // Goal not yet achieved
      goal_achieved_ = false; // Reset if goal is not met
    }
 
    // Get controller max velocity based on current speed
    double max_vel_x = max_vel_x_limited_;

    if (velocity.linear.x > max_vel_x)
    {
      // If our velocity limit has recently changed,
      // decelerate towards desired max_vel_x while still respecting acceleration limits
      double decelerating_max_vel_x = velocity.linear.x - (decel_lim_x_ * acc_dt_);
      max_vel_x = std::max(max_vel_x, decelerating_max_vel_x);
      max_vel_x = std::max(max_vel_x, min_vel_x_);
    }
    else
    {
      // Otherwise, allow up to max acceleration
      max_vel_x = velocity.linear.x + (acc_lim_x_ * acc_dt_);
      max_vel_x = std::max(min_vel_x_, std::min(max_vel_x, max_vel_x_limited_));
    }

    // Compute distance along path
    // Transform global plan into robot frame.
    // This is used both for curve detection and target-pose selection.
    std::vector<geometry_msgs::msg::PoseStamped> transformed_global_plan_poses;
    std::vector<double> target_distances;

    transformed_global_plan_poses.reserve(global_plan_.poses.size());

    for (const auto &pose : global_plan_.poses)
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(pose, transformed_pose, plan_to_robot);
      transformed_global_plan_poses.push_back(transformed_pose);
    }

    // Distances are still needed later for selecting the target/lookahead pose.
    computeDistanceAlongPath(transformed_global_plan_poses, target_distances);

    // Curve detection is now explicitly from the transformed global plan,
    const double curve_factor = computeCurveFactorFromGlobalPlan(transformed_global_plan_poses);
    
    // LOOKAHEAD WITH GRADUAL ENTER + EXIT
  
    const double curve_min_lookahead = std::max(resolution_, std::min(max_lookahead_, curve_min_lookahead_distance_));

    // Raw desired lookahead from current curve severity.
    // curve_factor = 0.0 -> max_lookahead_
    // curve_factor = 1.0 -> curve_min_lookahead
    const double target_max_lookahead = max_lookahead_ - curve_factor * (max_lookahead_ - curve_min_lookahead);

    // Time delta
    const rclcpp::Time now = clock_->now();

    double dt = acc_dt_;

    if (filtered_lookahead_initialized_)
    {
      const double measured_dt = (now - last_lookahead_update_time_).seconds();

      if (std::isfinite(measured_dt) && measured_dt > 0.0 && measured_dt < 1.0)
      {
        dt = measured_dt;
      }
    }

    last_lookahead_update_time_ = now;

    // Initialize from full lookahead so even first curve entry reduces gradually
    if (!filtered_lookahead_initialized_)
    {
      filtered_max_lookahead_ = max_lookahead_;
      filtered_lookahead_initialized_ = true;
    }

    // Move filtered_max_lookahead_ gradually toward target_max_lookahead
    if (target_max_lookahead < filtered_max_lookahead_)
    {
      // Entering curve: reduce gradually
      const double max_decrease = std::max(0.0, curve_lookahead_decrease_rate_) * dt;

      filtered_max_lookahead_ = std::max(target_max_lookahead, filtered_max_lookahead_ - max_decrease);
    }
    else if (target_max_lookahead > filtered_max_lookahead_)
    {
      // Exiting curve: increase gradually
      const double max_increase = std::max(0.0, curve_lookahead_increase_rate_) * dt;

      filtered_max_lookahead_ = std::min(target_max_lookahead, filtered_max_lookahead_ + max_increase);
    }

    const double active_max_lookahead = std::clamp(filtered_max_lookahead_, curve_min_lookahead, max_lookahead_);

    // ensure min constraint
    const double active_min_lookahead = std::max(resolution_, std::min(min_lookahead_, active_max_lookahead));

    // SPEED 
    const double curve_min_speed = std::max(min_vel_x_, std::min(max_vel_x, curve_min_speed_));

    max_vel_x = max_vel_x - curve_factor * (max_vel_x - curve_min_speed);

    RCLCPP_INFO(LOGGER, "curve_factor=%.3f target_max_lookahead=%.3f filtered_max_lookahead=%.3f " "active_min_lookahead=%.3f active_max_lookahead=%.3f "
      "lookahead_decrease_rate=%.3f lookahead_increase_rate=%.3f " "curve_limited_max_vel_x=%.3f transformed_global_plan_poses=%zu",
      curve_factor, target_max_lookahead, filtered_max_lookahead_, active_min_lookahead, active_max_lookahead,
      curve_lookahead_decrease_rate_, curve_lookahead_increase_rate_, max_vel_x, transformed_global_plan_poses.size());

    // Work back from the end of plan to find valid target pose
    for (int i = static_cast<int>(transformed_global_plan_poses.size()) - 1; i >= 0; --i)
    {
      geometry_msgs::msg::PoseStamped target_pose = transformed_global_plan_poses[static_cast<std::size_t>(i)];

      double dist_to_target = target_distances[static_cast<std::size_t>(i)];

      // Continue if target_pose is too far away from robot
      if (dist_to_target > active_max_lookahead)
      {
        continue;
      }

      if (prefer_final_rotation_)
      {
        const double carrot_yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);

        const bool moving_goal_yaw_alignment_allowed =
            dist_to_goal < goal_yaw_alignment_distance &&
            std::abs(final_yaw_error) <= max_moving_yaw_alignment_error;

        if (moving_goal_yaw_alignment_allowed)
        {
          // Within 1 m of goal and yaw error is less than 20 deg.
          // Start aligning to final goal yaw while moving.
          target_pose.pose.orientation.x = 0.0;
          target_pose.pose.orientation.y = 0.0;
          target_pose.pose.orientation.z = std::sin(final_yaw_error / 2.0);
          target_pose.pose.orientation.w = std::cos(final_yaw_error / 2.0);

          const double current_yaw_error = final_yaw_error;
          const double current_yaw_error_deg = current_yaw_error * 180.0 / M_PI;
          const double abs_current_yaw_error_deg = std::abs(current_yaw_error) * 180.0 / M_PI;

          RCLCPP_INFO(LOGGER, "Moving goal yaw alignment active -> dist_to_goal=%.3f "
              "current_yaw_error=%.3f current_yaw_error_deg=%.2f abs_yaw_error_deg=%.2f deg max_allowed=%.3f rad",
              dist_to_goal, current_yaw_error, current_yaw_error_deg, abs_current_yaw_error_deg, max_moving_yaw_alignment_error);
        }
        else if (dist_to_goal < active_max_lookahead)
        {
          // If yaw error is too large, do not align while moving.
          // Keep target orientation facing the carrot/path direction.
          target_pose.pose.orientation.x = 0.0;
          target_pose.pose.orientation.y = 0.0;
          target_pose.pose.orientation.z = std::sin(carrot_yaw / 2.0);
          target_pose.pose.orientation.w = std::cos(carrot_yaw / 2.0);

          if (dist_to_goal < goal_yaw_alignment_distance && std::abs(final_yaw_error) > max_moving_yaw_alignment_error)
          {
            RCLCPP_INFO(LOGGER, "Moving goal yaw alignment skipped -> yaw error too large. "
                "dist_to_goal=%.3f final_yaw_error=%.3f max_allowed=%.3f",
                dist_to_goal, final_yaw_error, max_moving_yaw_alignment_error);
          }
        }
      }

      // Preserve old minimum-lookahead behavior.
      // Only break when we are not already close to the goal.
      if (dist_to_goal >= active_max_lookahead && dist_to_target < active_min_lookahead)
      {
        // Make sure target is far enough away to avoid instability
        break;
      }

      // Iteratively try to find a path, incrementally reducing the velocity
      double sim_velocity = max_vel_x;
      do
      {
        controller_->setVelocityLimits(min_vel_x_, sim_velocity, max_vel_theta_limited_);

        if (simulate(target_pose, velocity, cmd_vel))
        {
          RCLCPP_INFO(LOGGER, "After Simulation -> cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        
          // MOVING FINAL YAW ALIGNMENT CORRECTION / HOLD
          const bool moving_goal_yaw_alignment_zone = prefer_final_rotation_ &&
            dist_to_goal < goal_yaw_alignment_distance && std::abs(final_yaw_error) <= max_moving_yaw_alignment_error;

          if (moving_goal_yaw_alignment_zone)
          {
            const double yaw_align_kp = 1.2;
            const double max_yaw_align_correction = 0.20;

            const double before_angular_z = cmd_vel.twist.angular.z;

            if (std::abs(final_yaw_error) <= yaw_goal_tolerance)
            {
              // Yaw is already inside tolerance. Do not allow path/carrot angular command to pull yaw away again.
              cmd_vel.twist.angular.z = 0.0;

              RCLCPP_INFO(LOGGER, "Moving yaw hold -> final_yaw_error=%.3f rad %.2f deg "
                  "yaw_goal_tolerance=%.3f before_angular_z=%.3f after_angular_z=%.3f",
                  final_yaw_error, final_yaw_error * 180.0 / M_PI, yaw_goal_tolerance, before_angular_z, cmd_vel.twist.angular.z);
            }
            else
            {
              const double yaw_correction = std::clamp(yaw_align_kp * final_yaw_error,
                      -max_yaw_align_correction, max_yaw_align_correction);

              if (cmd_vel.twist.angular.z * final_yaw_error < 0.0)
              {
                cmd_vel.twist.angular.z = yaw_correction;
              }
              else
              {
                cmd_vel.twist.angular.z += yaw_correction;
              }

              cmd_vel.twist.angular.z = std::clamp(cmd_vel.twist.angular.z, -max_vel_theta_limited_, max_vel_theta_limited_);

              RCLCPP_INFO(LOGGER, "Moving yaw correction -> final_yaw_error=%.3f rad %.2f deg "
                  "before_angular_z=%.3f yaw_correction=%.3f after_angular_z=%.3f",
                  final_yaw_error, final_yaw_error * 180.0 / M_PI, before_angular_z, yaw_correction, cmd_vel.twist.angular.z);
            }
          }

          if (dist_to_goal < ignore_orientation_distance_ && !moving_goal_yaw_alignment_zone)
          {
            RCLCPP_INFO(LOGGER, "Robot is within %.2f meters of the goal. Ignoring orientation changes.", dist_to_goal);

            cmd_vel.twist.angular.z = 0.0;
          }
          return cmd_vel;
        }

        sim_velocity -= scaling_step_;
      } while (sim_velocity >= scaling_vel_x_);
    }

    RCLCPP_WARN(LOGGER, "No reachable pose found in the plan. Stopping the robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    // Do NOT set goal_achieved_ = true here
    // Instead, throw or return a failure for the BT to handle
    throw std::runtime_error("No reachable pose found. Aborting navigation from BT.");
  }
 
  bool GracefulControllerROS::simulate(
      const geometry_msgs::msg::PoseStamped &target_pose,
      const geometry_msgs::msg::Twist &velocity,
      geometry_msgs::msg::TwistStamped &cmd_vel)
  {
 
    // Simulated path (for debugging/visualization)
    nav_msgs::msg::Path simulated_path;
    simulated_path.header.frame_id = costmap_ros_->getBaseFrameID();
    simulated_path.header.stamp = clock_->now();
    // Should we simulate rotation initially
    bool sim_initial_rotation_ = has_new_path_ && initial_rotate_tolerance_ > 0.0;
 
    // Clear any previous visualizations
    if (collision_points_)
    {
      collision_points_->markers.resize(0);
    }
 
    // Get control and path, iteratively
    while (true)
    {
      // The error between current robot pose and the target pose
      geometry_msgs::msg::PoseStamped error = target_pose;
      double error_angle = tf2::getYaw(error.pose.orientation);

      // Move origin to our current simulated pose
      if (!simulated_path.poses.empty())
      {
        double x = error.pose.position.x - simulated_path.poses.back().pose.position.x;
        double y = error.pose.position.y - simulated_path.poses.back().pose.position.y;
        double theta = -tf2::getYaw(simulated_path.poses.back().pose.orientation);

        error.pose.position.x = x * cos(theta) - y * sin(theta);
        error.pose.position.y = y * cos(theta) + x * sin(theta);

        error_angle += theta;
        error.pose.orientation.z = sin(error_angle / 2.0);
        error.pose.orientation.w = cos(error_angle / 2.0);
      }

      // Compute commands
      double vel_x = 0.0, vel_th = 0.0;
      if (sim_initial_rotation_)
      {
        geometry_msgs::msg::TwistStamped rotation;
        double yaw_error = rotateTowards(error, velocity, rotation, prefer_final_rotation_);

        if (fabs(yaw_error) < initial_rotate_tolerance_)
        {
          if (simulated_path.poses.empty())
          {
            has_new_path_ = false;
          }
          sim_initial_rotation_ = false;
        }

        vel_x = rotation.twist.linear.x;
        vel_th = rotation.twist.angular.z;

        // Only set angular velocity if prefer_final_rotation_ is enabled
        if (prefer_final_rotation_)
        {
          cmd_vel.twist.linear.x = vel_x;
          cmd_vel.twist.angular.z = vel_th;

          // Log the cmd_vel after rotation command
          RCLCPP_INFO(LOGGER, "Simulation Rotation -> cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        }
        else
        {
          cmd_vel.twist.linear.x = vel_x;
          cmd_vel.twist.angular.z = 0.0; // Force angular.z to zero

          // Log the cmd_vel after rotation command
          RCLCPP_INFO(LOGGER, "Simulation Rotation without angular velocity -> cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        }
      }

      if (!sim_initial_rotation_)
      {
        if (!controller_->approach(error.pose.position.x, error.pose.position.y, error_angle,
                                  vel_x, vel_th, backward_motion_))
        {
          RCLCPP_INFO(LOGGER, "Controller approach failed during simulation.");
          return false;
        }
      }

      if (simulated_path.poses.empty())
      {
        // First iteration of simulation, store our commands to the robot
        cmd_vel.twist.linear.x = vel_x;
        cmd_vel.twist.angular.z = vel_th;
      }
      else if (std::hypot(error.pose.position.x, error.pose.position.y) < resolution_)
      {
        // We've simulated to the desired pose, can return this result
        local_plan_pub_->publish(simulated_path);
        target_pose_pub_->publish(target_pose);

        // Publish visualization if desired
        if (collision_points_ && !collision_points_->markers.empty())
        {
          collision_points_->markers[0].header.stamp = clock_->now();
          collision_points_pub_->publish(*collision_points_);
        }

        return true;
      }

      // Forward simulate command
      geometry_msgs::msg::PoseStamped next_pose;
      next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
      if (simulated_path.poses.empty())
      {
        // Initialize at origin
        next_pose.pose.orientation.w = 1.0;
      }
      else
      {
        // Start at last pose
        next_pose = simulated_path.poses.back();
      }

      // Generate next pose
      double dt = (vel_x > 0.0) ? resolution_ / vel_x : 0.1;
      double yaw = tf2::getYaw(next_pose.pose.orientation);
      next_pose.pose.position.x += dt * vel_x * cos(yaw);
      next_pose.pose.position.y += dt * vel_x * sin(yaw);
      yaw += dt * vel_th;
      next_pose.pose.orientation.z = sin(yaw / 2.0);
      next_pose.pose.orientation.w = cos(yaw / 2.0);

      next_pose.header.stamp = clock_->now();
      next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
      simulated_path.poses.push_back(next_pose);

      // Compute footprint scaling
      double footprint_scaling = 1.0;
      if (vel_x > scaling_vel_x_)
      {
        double ratio = max_vel_x_limited_ - scaling_vel_x_;
        if (ratio > 0)
        {
          ratio = (vel_x - scaling_vel_x_) / ratio;
          footprint_scaling += ratio * scaling_factor_;
        }
      }

      // Check next pose for collision
      tf2::doTransform(next_pose, next_pose, robot_to_costmap_transform_);
      bool collision = isColliding(next_pose.pose.position.x, next_pose.pose.position.y, tf2::getYaw(next_pose.pose.orientation),
                                  costmap_ros_, collision_points_, footprint_scaling);
      if (collision)
      {
        RCLCPP_INFO(LOGGER, "Collision detected at simulated pose: [x: %.2f, y: %.2f]",
                    next_pose.pose.position.x, next_pose.pose.position.y);

        if (collision_points_ && !collision_points_->markers.empty())
        {
          collision_points_->markers[0].header.stamp = clock_->now();
          collision_points_pub_->publish(*collision_points_);
        }

        // Log the cmd_vel before returning
        RCLCPP_INFO(LOGGER, "Collision detected during simulation. cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                    cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        return false;
      }
    }
 
    // Really shouldn't hit this
    RCLCPP_INFO(LOGGER, "Simulation loop exited unexpectedly without reaching target.");
    return false;
  }
 
  void GracefulControllerROS::setPlan(const nav_msgs::msg::Path &path)
  {
    RCLCPP_INFO(LOGGER, "Received new plan with %lu poses.", path.poses.size());

    if (!initialized_)
    {
      RCLCPP_INFO(LOGGER, "Controller is not initialized, call initialize() before using this controller");
      return;
    }

    // We need orientations on our poses
    nav_msgs::msg::Path oriented_plan;
    if (compute_orientations_)
    {
      RCLCPP_INFO(LOGGER, "Adding orientations to the plan.");
      oriented_plan = addOrientations(path);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Using plan without adding orientations.");
      oriented_plan = path;
    }

    // Filter noisy orientations (if desired)
    nav_msgs::msg::Path filtered_plan;
    if (use_orientation_filter_)
    {
      RCLCPP_INFO(LOGGER, "Applying orientation filter to the plan.");
      filtered_plan = applyOrientationFilter(oriented_plan, yaw_filter_tolerance_, yaw_gap_tolerance_);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Using plan without orientation filtering.");
      filtered_plan = oriented_plan;
    }

    // Backward motion logic
    if (backward_motion_available_)
    {
      RCLCPP_INFO(LOGGER, "Backward motion is available. Processing robot pose for backward motion determination.");

      // Callback spin to get robot pose
      robot_pose_received_ = false;
      callback_group_executor_.spin_some();
      double robot_orientation;

      // Check and set backward motion / forward motion
      global_plan_ = filtered_plan;

      if (robot_pose_received_)
      {
        // Extract yaw from orientation
        tf2::Quaternion orientation;
        tf2::fromMsg(robot_pose_.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        robot_orientation = normalizeAngle(yaw);
      }
      else
      {
        robot_orientation = 0.0;
        RCLCPP_INFO(LOGGER, "Robot Pose Unavailable for Backwards motion.");
      }

      tf2::Quaternion orientation;
      tf2::fromMsg(global_plan_.poses[0].pose.orientation, orientation);
      double roll, pitch, yaw;
      tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      double path_orientation = normalizeAngle(yaw);
      backward_motion_ = false;

      double diff_angle = normalizeAngle(path_orientation - robot_orientation);
      diff_angle = fabs(diff_angle);
      double angle_tolerance = fabs(backwards_check_yaw_tolerance_);

      // Check if the angle difference is approximately 180 degrees
      if (diff_angle > (M_PI - angle_tolerance) && diff_angle < (M_PI + angle_tolerance))
      {
        backward_motion_ = true;
        global_plan_ = filtered_plan;
        RCLCPP_INFO(LOGGER, "Backward motion initiated due to angle difference: %.2f radians.", diff_angle);
      }
      else if (diff_angle > (-angle_tolerance) && diff_angle < (angle_tolerance))
      {
        backward_motion_ = false;
        global_plan_ = filtered_plan;
        RCLCPP_INFO(LOGGER, "Backward motion not required. Angle difference: %.2f radians.", diff_angle);
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Angle difference %.2f radians does not qualify for backward motion.", diff_angle);
      }
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Backward motion is not available. Setting backward_motion_ to FALSE.");
      backward_motion_ = false;
      global_plan_ = filtered_plan;
    }

    // Store the plan for computeVelocityCommands
    has_new_path_ = true;
    goal_tolerance_met_ = false;
    goal_achieved_ = false; // Reset the flag

    filtered_lookahead_initialized_ = false;
    filtered_max_lookahead_ = max_lookahead_;
    RCLCPP_INFO(LOGGER, "Plan set successfully in frame: %s", filtered_plan.header.frame_id.c_str());
  }

  double GracefulControllerROS::computeCurveFactorFromGlobalPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> &plan_poses) const
  {
    if (plan_poses.size() < 3)
    {
      RCLCPP_WARN(LOGGER, "Global plan curve detection skipped -> plan_poses too small: %zu", plan_poses.size());
      return 0.0;
    }

    // Since poses are transformed into base_link, robot is near (0, 0).
    std::size_t closest_index = 0;
    double closest_dist = std::hypot(plan_poses[0].pose.position.x, plan_poses[0].pose.position.y);

    for (std::size_t i = 1; i < plan_poses.size(); ++i)
    {
      const double dist = std::hypot(plan_poses[i].pose.position.x, plan_poses[i].pose.position.y);

      if (dist < closest_dist)
      {
        closest_dist = dist;
        closest_index = i;
      }
    }

    if (closest_index + 2 >= plan_poses.size())
    {
      RCLCPP_WARN(LOGGER, "Global plan curve detection skipped -> closest_index near end. "
          "closest_index=%zu plan_size=%zu closest_dist=%.3f",
          closest_index, plan_poses.size(), closest_dist);
      return 0.0;
    }

    const double horizon = std::max(curve_detection_distance_, max_lookahead_);

    // Important:
    // Do not use 0.013 here. Your global plan spacing is around 0.010 m.
    // Use a larger sampling distance and accumulate small segments.
    const double heading_sample_distance = std::max(0.05, resolution_);

    double travelled = 0.0;
    double sample_accumulated = 0.0;
    double accumulated_turn = 0.0;

    double last_heading = 0.0;
    bool have_heading = false;

    std::size_t scanned_segments = 0;
    std::size_t valid_heading_segments = 0;

    geometry_msgs::msg::Point sample_start = plan_poses[closest_index].pose.position;

    for (std::size_t i = closest_index; i + 1 < plan_poses.size(); ++i)
    {
      const auto &p0 = plan_poses[i].pose.position;
      const auto &p1 = plan_poses[i + 1].pose.position;

      const double dx_seg = p1.x - p0.x;
      const double dy_seg = p1.y - p0.y;
      const double segment_length = std::hypot(dx_seg, dy_seg);

      travelled += segment_length;
      sample_accumulated += segment_length;
      scanned_segments++;

      if (travelled > horizon)
      {
        break;
      }

      // Wait until enough tiny plan segments are accumulated.
      if (sample_accumulated < heading_sample_distance)
      {
        continue;
      }

      const double dx = p1.x - sample_start.x;
      const double dy = p1.y - sample_start.y;
      const double sample_length = std::hypot(dx, dy);

      if (sample_length < 1e-4)
      {
        sample_start = p1;
        sample_accumulated = 0.0;
        continue;
      }

      const double heading = std::atan2(dy, dx);

      if (have_heading)
      {
        accumulated_turn += std::abs(angles::shortest_angular_distance(last_heading, heading));
      }

      last_heading = heading;
      have_heading = true;
      valid_heading_segments++;

      sample_start = p1;
      sample_accumulated = 0.0;
    }

    const double turn_range = curve_turn_full_threshold_ - curve_turn_threshold_;

    if (!have_heading || valid_heading_segments < 2)
    {
      RCLCPP_WARN(LOGGER, "Global plan curve detection skipped -> not enough sampled heading segments. "
          "plan_size=%zu closest_index=%zu closest_dist=%.3f travelled=%.3f " "scanned_segments=%zu valid_heading_segments=%zu sample_distance=%.3f",
          plan_poses.size(), closest_index, closest_dist, travelled, scanned_segments, valid_heading_segments, heading_sample_distance);
      return 0.0;
    }

    if (turn_range <= 0.0)
    {
      RCLCPP_WARN(LOGGER, "Global plan curve detection skipped -> invalid turn thresholds. "
          "curve_turn_threshold=%.3f curve_turn_full_threshold=%.3f",
          curve_turn_threshold_, curve_turn_full_threshold_);
      return 0.0;
    }

    const double curve_factor = std::clamp(
        (accumulated_turn - curve_turn_threshold_) / turn_range,
        0.0,
        1.0);

    return curve_factor;
  }

  double GracefulControllerROS::rotateTowards(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      geometry_msgs::msg::TwistStamped &cmd_vel,
      bool perform_rotation) // New parameter
  {
    if (!perform_rotation)
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      RCLCPP_DEBUG(LOGGER, "Rotation disabled. cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                  cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
      return 0.0;
    }

    RCLCPP_INFO(LOGGER, "------ Rotating 1 -------");
 
    // Determine error
    double yaw = 0.0;
    if (std::hypot(pose.pose.position.x, pose.pose.position.y) > 0.5)
    {
      // Goal is far away, point towards goal
      yaw = std::atan2(pose.pose.position.y, pose.pose.position.x);
    }
    else
    {
      // Goal is nearby, align heading
      yaw = tf2::getYaw(pose.pose.orientation);
    }
 
    // Compute command velocity
    rotateTowards(yaw, velocity, cmd_vel, perform_rotation);

    // Return error
    return yaw;
  }
 
  void GracefulControllerROS::rotateTowards(
      double yaw,
      const geometry_msgs::msg::Twist &velocity,
      geometry_msgs::msg::TwistStamped &cmd_vel,
      bool perform_rotation) // New parameter
  {
    if (!perform_rotation)
    {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      RCLCPP_DEBUG(LOGGER, "Rotation disabled in helper. cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                  cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
      return;
    }

    RCLCPP_INFO(LOGGER, "------ Rotating 2 -------");

    // Determine max velocity based on current speed
    double max_vel_th = max_vel_theta_limited_;
    if (acc_dt_ > 0.0)
    {
      double abs_vel = fabs(velocity.angular.z);
      double acc_limited = abs_vel + (acc_lim_theta_ * acc_dt_);
      max_vel_th = std::min(max_vel_th, acc_limited);
      max_vel_th = std::max(max_vel_th, min_in_place_vel_theta_);
    }

    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = sign(yaw) * std::min(max_vel_th, std::max(min_in_place_vel_theta_, fabs(yaw * yaw_slowing_factor_)));
    RCLCPP_INFO(LOGGER, "Rotation command set: angular.z=%.2f", cmd_vel.twist.angular.z);
    RCLCPP_INFO(LOGGER, "cmd_vel Output -> linear.x: %.2f, angular.z: %.2f",
                cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  }
 
  void GracefulControllerROS::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    RCLCPP_INFO(LOGGER, "Setting speed limit: %.2f (%s)", speed_limit, percentage ? "Percentage" : "Absolute");

    if (speed_limit == 0.0)
    {
      max_vel_x_limited_ = max_vel_x_;
      RCLCPP_INFO(LOGGER, "Speed limit is 0.0. Resetting max_vel_x_limited_ to max_vel_x: %.2f", max_vel_x_);
    }
    else
    {
      if (percentage)
      {
        max_vel_x_limited_ = std::max((speed_limit / 100.0) * max_vel_x_, min_vel_x_);
        RCLCPP_INFO(LOGGER, "Speed limit set to %.2f%% of max_vel_x: %.2f", speed_limit, max_vel_x_limited_);
      }
      else
      {
        max_vel_x_limited_ = std::max(speed_limit, min_vel_x_);
        RCLCPP_INFO(LOGGER, "Speed limit set to absolute value: %.2f", max_vel_x_limited_);
      }
    }

    // Limit maximum angular velocity proportional to maximum linear velocity
    max_vel_theta_limited_ = max_vel_x_limited_ * max_x_to_max_theta_scale_factor_;
    max_vel_theta_limited_ = std::min(max_vel_theta_limited_, max_vel_theta_);
    RCLCPP_INFO(LOGGER, "max_vel_theta_limited_ set to %.2f", max_vel_theta_limited_);
  }
 
  void computeDistanceAlongPath(const std::vector<geometry_msgs::msg::PoseStamped> &poses,
                                std::vector<double> &distances)
  {
    distances.resize(poses.size());

    // First compute distance from robot to pose
    for (size_t i = 0; i < poses.size(); ++i)
    {
      // Determine distance from robot to pose
      distances[i] = std::hypot(poses[i].pose.position.x, poses[i].pose.position.y);
    }

    // Find the closest target pose
    auto closest = std::min_element(std::begin(distances), std::end(distances));

    // Sum distances between poses, starting with the closest pose
    for (size_t i = std::distance(std::begin(distances), closest) + 1; i < distances.size(); ++i)
    {
      distances[i] = distances[i - 1] +
                    std::hypot(poses[i].pose.position.x - poses[i - 1].pose.position.x,
                                poses[i].pose.position.y - poses[i - 1].pose.position.y);
    }
  }
 
  rcl_interfaces::msg::SetParametersResult GracefulControllerROS::onParameterChange(
      const std::vector<rclcpp::Parameter> & parameters)
  {

      std::lock_guard<std::mutex> lock(config_mutex_);  // Ensure thread safety

      for (const auto & param : parameters)
        if (param.get_name() == name_ + ".max_vel_x")
        {
            max_vel_x_ = param.as_double();
            RCLCPP_INFO(LOGGER, "Current Updated max_speed to: %f", max_vel_x_);
        }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
  }

} // namespace graceful_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(graceful_controller::GracefulControllerROS, nav2_core::Controller)