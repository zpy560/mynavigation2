#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

namespace nav2_regulated_modules
{

RegulatedNavigator::RegulatedNavigator(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("regulated_navigator", "", options) {
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("operation_mode", "autonomous");
  declare_parameter("goal_topic", "goal_pose");
  declare_parameter("navigation_service_action", "navigation_service");
  declare_parameter("fixed_path_visualization_topic", "fixed_path_plan");
  declare_parameter("fixed_path_boundaries_topic", "fixed_path_boundaries");
  declare_parameter("fixed_path_boundary_half_width", 0.4);
  declare_parameter("navigate_to_pose_action", "navigate_to_pose");
  declare_parameter("navigate_through_poses_action", "navigate_through_poses");
  declare_parameter("compute_path_to_pose_action", "compute_path_to_pose");
  declare_parameter("compute_path_through_poses_action", "compute_path_through_poses");
  declare_parameter("smooth_path_action", "smooth_path");
  declare_parameter("follow_path_action", "follow_path");
  declare_parameter("planner_id", "GridBasedAstar");
  declare_parameter("controller_id", "RPP");
  declare_parameter("goal_checker_id", "stopped_goal_checker");
  declare_parameter("smoother_id", "simple_smoother");
  declare_parameter("use_smoother", true);
  declare_parameter("check_smoother_collisions", true);
  declare_parameter("server_timeout", 5.0);
  declare_parameter("cancel_timeout", 2.0);
  declare_parameter("max_smoothing_duration", 2.0);
  declare_parameter("feedback_frequency", 5.0);
  declare_parameter("replan_frequency", 1.0);
  declare_parameter("max_consecutive_planning_failures", 3);
  declare_parameter("max_recovery_rounds", 2);
  declare_parameter("costmap_update_wait_duration", 0.8);
  declare_parameter("passed_goal_radius", 0.7);
  declare_parameter("localization_timeout", 0.3);
  declare_parameter("max_localization_translation_jump", 0.3);
  declare_parameter("max_localization_rotation_jump", 0.35);
  declare_parameter("progress_timeout", 10.0);
  declare_parameter("progress_min_translation", 0.1);
  declare_parameter("localization_recovery_timeout", 10.0);
  declare_parameter("localization_stable_duration", 0.5);
  declare_parameter("fixed_path_step", 0.1);
  declare_parameter("stop_cmd_vel_topic", "cmd_vel_nav");
  declare_parameter("controller_cmd_vel_topic", "cmd_vel_nav");
  declare_parameter("smoothed_cmd_vel_topic", "cmd_vel");
  declare_parameter("velocity_odom_topic", "/odometry");
  declare_parameter("velocity_log_frequency", 1.0);
  declare_parameter<std::string>("speed_limit_topic", "speed_limit");
}

nav2_util::CallbackReturn RegulatedNavigator::on_configure(const rclcpp_lifecycle::State &) {
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  const auto operation_mode = get_parameter("operation_mode").as_string();
  if (operation_mode == "remote") {
    operation_mode_ = NavigationMode::REMOTE;
  } else if (operation_mode == "autonomous") {
    operation_mode_ = NavigationMode::AUTONOMOUS;
  } else if (operation_mode == "fixed_path") {
    operation_mode_ = NavigationMode::FIXED_PATH;
  } else {
    LOG_ERROR("不支持的 operation_mode：{}", operation_mode);
    return nav2_util::CallbackReturn::FAILURE;
  }
  goal_topic_ = get_parameter("goal_topic").as_string();
  navigation_service_action_ = get_parameter("navigation_service_action").as_string();
  fixed_path_visualization_topic_ = get_parameter("fixed_path_visualization_topic").as_string();
  fixed_path_boundaries_topic_ = get_parameter("fixed_path_boundaries_topic").as_string();
  server_timeout_ = get_parameter("server_timeout").as_double();
  cancel_timeout_ = get_parameter("cancel_timeout").as_double();
  smoothing_duration_ = get_parameter("max_smoothing_duration").as_double();
  check_smoother_collisions_ = get_parameter("check_smoother_collisions").as_bool();
  feedback_frequency_ = get_parameter("feedback_frequency").as_double();
  max_recovery_rounds_ = get_parameter("max_recovery_rounds").as_int();
  costmap_wait_duration_ = get_parameter("costmap_update_wait_duration").as_double();
  passed_goal_radius_ = get_parameter("passed_goal_radius").as_double();
  localization_timeout_ = get_parameter("localization_timeout").as_double();
  max_translation_jump_ = get_parameter("max_localization_translation_jump").as_double();
  max_rotation_jump_ = get_parameter("max_localization_rotation_jump").as_double();
  progress_min_translation_ = get_parameter("progress_min_translation").as_double();
  localization_recovery_timeout_ = get_parameter("localization_recovery_timeout").as_double();
  localization_stable_duration_ = get_parameter("localization_stable_duration").as_double();
  fixed_path_step_ = get_parameter("fixed_path_step").as_double();
  fixed_path_boundary_half_width_ = get_parameter("fixed_path_boundary_half_width").as_double();
  controller_cmd_vel_topic_ = get_parameter("controller_cmd_vel_topic").as_string();
  smoothed_cmd_vel_topic_ = get_parameter("smoothed_cmd_vel_topic").as_string();
  velocity_odom_topic_ = get_parameter("velocity_odom_topic").as_string();
  speed_limit_topic_ = get_parameter("speed_limit_topic").as_string();
  velocity_log_frequency_ = get_parameter("velocity_log_frequency").as_double();
  if (!std::isfinite(fixed_path_step_) || fixed_path_step_ <= 0.0) {
    LOG_ERROR("fixed_path_step 必须为有限正数，当前值={}", fixed_path_step_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!std::isfinite(fixed_path_boundary_half_width_) || fixed_path_boundary_half_width_ <= 0.0) {
    LOG_ERROR("fixed_path_boundary_half_width 必须为有限正数，当前值={}", fixed_path_boundary_half_width_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!std::isfinite(velocity_log_frequency_) || velocity_log_frequency_ <= 0.0) {
    LOG_ERROR("velocity_log_frequency 必须为有限正数，当前值={}", velocity_log_frequency_);
    return nav2_util::CallbackReturn::FAILURE;
  }

  planning_module_.configure(get_parameter("planner_id").as_string(), get_parameter("smoother_id").as_string(), get_parameter("use_smoother").as_bool(), get_parameter("replan_frequency").as_double(), get_parameter("max_consecutive_planning_failures").as_int());
  control_module_.configure(get_parameter("controller_id").as_string(), get_parameter("goal_checker_id").as_string(), get_parameter("progress_timeout").as_double());

  compute_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, get_parameter("compute_path_to_pose_action").as_string());
  compute_poses_client_ = rclcpp_action::create_client<ComputePathThroughPoses>(this, get_parameter("compute_path_through_poses_action").as_string());
  smooth_client_ = rclcpp_action::create_client<SmoothPath>(this, get_parameter("smooth_path_action").as_string());
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, get_parameter("follow_path_action").as_string());
  clear_local_client_ = create_client<ClearCostmap>("local_costmap/clear_entirely_local_costmap");
  clear_global_client_ = create_client<ClearCostmap>("global_costmap/clear_entirely_global_costmap");

  navigate_pose_server_ = rclcpp_action::create_server<NavigateToPose>(this, get_parameter("navigate_to_pose_action").as_string(), std::bind(&RegulatedNavigator::handlePoseGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handlePoseCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handlePoseAccepted, this, std::placeholders::_1));
  navigate_poses_server_ = rclcpp_action::create_server<NavigateThroughPoses>(this, get_parameter("navigate_through_poses_action").as_string(), std::bind(&RegulatedNavigator::handlePosesGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handlePosesCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handlePosesAccepted, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&RegulatedNavigator::onTopicGoal, this, std::placeholders::_1));
  if (operation_mode_ == NavigationMode::FIXED_PATH) {
    navigation_service_server_ = rclcpp_action::create_server<NavigationService>(this, navigation_service_action_, std::bind(&RegulatedNavigator::handleNavigationServiceGoal, this, std::placeholders::_1, std::placeholders::_2), std::bind(&RegulatedNavigator::handleNavigationServiceCancel, this, std::placeholders::_1), std::bind(&RegulatedNavigator::handleNavigationServiceAccepted, this, std::placeholders::_1));
    fixed_path_pub_ = create_publisher<nav_msgs::msg::Path>(fixed_path_visualization_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    fixed_path_boundaries_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(fixed_path_boundaries_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }
  stop_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("stop_cmd_vel_topic").as_string(), rclcpp::SystemDefaultsQoS());
  const auto velocity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  controller_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(controller_cmd_vel_topic_, velocity_qos, std::bind(&RegulatedNavigator::onControllerVelocity, this, std::placeholders::_1));
  smoothed_velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(smoothed_cmd_vel_topic_, velocity_qos, std::bind(&RegulatedNavigator::onSmoothedVelocity, this, std::placeholders::_1));
  velocity_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(velocity_odom_topic_, velocity_qos, std::bind(&RegulatedNavigator::onVelocityOdometry, this, std::placeholders::_1));
  speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>( speed_limit_topic_, rclcpp::QoS(10));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto feedback_period = std::chrono::duration<double>(1.0 / feedback_frequency_);
  feedback_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(feedback_period), std::bind(&RegulatedNavigator::publishFeedback, this));
  monitor_timer_ = create_wall_timer(200ms, std::bind(&RegulatedNavigator::monitorTask, this));
  const auto velocity_log_period = std::chrono::duration<double>(1.0 / velocity_log_frequency_);
  velocity_log_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(velocity_log_period), std::bind(&RegulatedNavigator::logVelocityChain, this));

  configured_ = true;
  LOG_INFO("独立规控导航器配置完成，operation_mode={}，速度日志={}Hz，反馈={}，控制器输出={}，平滑器输出={}", operation_mode, velocity_log_frequency_, velocity_odom_topic_, controller_cmd_vel_topic_, smoothed_cmd_vel_topic_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RegulatedNavigator::on_activate(const rclcpp_lifecycle::State &) {
  if (fixed_path_pub_) {fixed_path_pub_->on_activate();}
  if (fixed_path_boundaries_pub_) {fixed_path_boundaries_pub_->on_activate();}
  active_ = true;
  createBond();
  LOG_INFO("独立规控导航器已激活");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RegulatedNavigator::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  cancelTask("节点停用");
  if (fixed_path_pub_) {fixed_path_pub_->on_deactivate();}
  if (fixed_path_boundaries_pub_) {fixed_path_boundaries_pub_->on_deactivate();}
  destroyBond();
  LOG_INFO("独立规控导航器已停用");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RegulatedNavigator::on_cleanup(const rclcpp_lifecycle::State &) {
  cancelTask("节点清理");
  navigate_pose_server_.reset();
  navigate_poses_server_.reset();
  navigation_service_server_.reset();
  compute_pose_client_.reset();
  compute_poses_client_.reset();
  smooth_client_.reset();
  follow_client_.reset();
  clear_local_client_.reset();
  clear_global_client_.reset();
  goal_sub_.reset();
  controller_velocity_sub_.reset();
  smoothed_velocity_sub_.reset();
  velocity_odom_sub_.reset();
  stop_cmd_pub_.reset();
  fixed_path_pub_.reset();
  fixed_path_boundaries_pub_.reset();
  feedback_timer_.reset();
  monitor_timer_.reset();
  velocity_log_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    has_controller_velocity_ = false;
    has_smoothed_velocity_ = false;
    has_velocity_odometry_ = false;
  }
  tf_listener_.reset();
  tf_buffer_.reset();
  configured_ = false;
  LOG_INFO("独立规控导航器已清理");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn RegulatedNavigator::on_shutdown(const rclcpp_lifecycle::State &) {
  cancelTask("节点关闭");
  LOG_INFO("独立规控导航器已关闭");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_regulated_modules
