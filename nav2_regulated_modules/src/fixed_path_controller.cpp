// 固定路径控制器实现。按运动方向计算起点对齐、前视跟踪和终点停车指令。

#include "nav2_regulated_modules/fixed_path_controller.hpp"
#include "nav2_regulated_modules/detail/terminal_position.hpp"
#include "nav2_regulated_modules/detail/braking_margin.hpp"
#include "nav2_regulated_modules/fixed_path_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 读取参数并建立控制器运行所需的 ROS 接口或插件资源。
void FixedPathController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  // 生命周期父节点已销毁时无法读取参数或创建 TF 接口，直接终止插件配置。
  if (!node)
  {
    throw nav2_core::PlannerException("FixedPathController cannot lock lifecycle node");
  }
  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_position_tolerance", rclcpp::ParameterValue(1.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_lateral_tolerance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_offset_speed_limit", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_limit_distance", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_release_yaw_tolerance", rclcpp::ParameterValue(0.3490658503988659));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".start_speed_release_stable_cycles", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".direct_tracking_max_yaw_error", rclcpp::ParameterValue(0.2617993877991494));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".initial_yaw_tolerance", rclcpp::ParameterValue(0.05235987755982989));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_kp", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.005));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_linear_deceleration", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_final_approach_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_stop_entry_tolerance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_reaction_time", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_distance_margin", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".dynamic_goal_braking_margin_enabled", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_min_distance_margin", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_braking_margin_transition_speed", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_braking_enabled", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_max_deceleration", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_jerk_limit", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_approach_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_response_time", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".adaptive_goal_distance_margin", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_dist", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_reference_speed", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_speed_gain", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_min_dist", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_terminal_lookahead_max_dist", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_error_log_frequency", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".alignment_stable_cycles", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  double controller_frequency = 100.0;
  node->get_parameter(plugin_name_ + ".desired_linear_vel", base_linear_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".start_position_tolerance", start_position_tolerance_);
  node->get_parameter(plugin_name_ + ".direct_tracking_lateral_tolerance", direct_tracking_lateral_tolerance_);
  node->get_parameter(plugin_name_ + ".start_offset_speed_limit", start_offset_speed_limit_);
  node->get_parameter(plugin_name_ + ".start_speed_limit_distance", start_speed_limit_distance_);
  node->get_parameter(plugin_name_ + ".start_speed_release_yaw_tolerance", start_speed_release_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".start_speed_release_stable_cycles", start_speed_release_stable_cycles_);
  node->get_parameter(plugin_name_ + ".direct_tracking_max_yaw_error", direct_tracking_max_yaw_error_);
  node->get_parameter(plugin_name_ + ".initial_yaw_tolerance", initial_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_kp", rotate_to_heading_kp_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".goal_linear_deceleration", goal_linear_deceleration_);
  node->get_parameter(plugin_name_ + ".goal_final_approach_velocity", goal_final_approach_velocity_);
  node->get_parameter(plugin_name_ + ".goal_stop_entry_tolerance", goal_stop_entry_tolerance_);
  node->get_parameter(plugin_name_ + ".goal_braking_reaction_time", goal_braking_reaction_time_);
  node->get_parameter(plugin_name_ + ".goal_braking_distance_margin", goal_braking_distance_margin_);
  node->get_parameter(plugin_name_ + ".dynamic_goal_braking_margin_enabled", dynamic_goal_braking_margin_enabled_);
  node->get_parameter(plugin_name_ + ".goal_braking_min_distance_margin", goal_braking_min_distance_margin_);
  node->get_parameter(plugin_name_ + ".goal_braking_margin_transition_speed", goal_braking_margin_transition_speed_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_braking_enabled", adaptive_goal_braking_enabled_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_max_deceleration", adaptive_goal_max_deceleration_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_jerk_limit", adaptive_goal_jerk_limit_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_approach_speed", adaptive_goal_approach_speed_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_response_time", adaptive_goal_response_time_);
  node->get_parameter(plugin_name_ + ".adaptive_goal_distance_margin", adaptive_goal_distance_margin_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_dist", goal_terminal_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_reference_speed", goal_terminal_lookahead_reference_speed_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_speed_gain", goal_terminal_lookahead_speed_gain_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_min_dist", goal_terminal_lookahead_min_dist_);
  node->get_parameter(plugin_name_ + ".goal_terminal_lookahead_max_dist", goal_terminal_lookahead_max_dist_);
  node->get_parameter(plugin_name_ + ".goal_error_log_frequency", goal_error_log_frequency_);
  node->get_parameter(plugin_name_ + ".alignment_stable_cycles", alignment_stable_cycles_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter("controller_frequency", controller_frequency);
  // 任一旧模式共用参数不满足正值、有限值或上下界关系时拒绝配置；终点前视也不能超出基础前视。
  if (base_linear_velocity_ <= 0.0 || lookahead_dist_ <= 0.0 || min_lookahead_dist_ <= 0.0 || max_lookahead_dist_ < min_lookahead_dist_ || start_position_tolerance_ <= 0.0 || direct_tracking_lateral_tolerance_ < 0.0 || direct_tracking_max_yaw_error_ <= 0.0 || direct_tracking_max_yaw_error_ > M_PI_2 || initial_yaw_tolerance_ <= 0.0 || initial_yaw_tolerance_ >= direct_tracking_max_yaw_error_ || rotate_to_heading_angular_vel_ <= 0.0 || max_angular_accel_ <= 0.0 || rotate_to_heading_kp_ <= 0.0 || min_approach_linear_velocity_ <= 0.0 || min_approach_linear_velocity_ > base_linear_velocity_ || approach_velocity_scaling_dist_ <= 0.0 || !std::isfinite(goal_linear_deceleration_) || goal_linear_deceleration_ <= 0.0 || !std::isfinite(goal_final_approach_velocity_) || goal_final_approach_velocity_ < 0.0 || !std::isfinite(goal_braking_reaction_time_) || goal_braking_reaction_time_ < 0.0 || !std::isfinite(goal_braking_distance_margin_) || goal_braking_distance_margin_ < 0.0 || !std::isfinite(goal_terminal_lookahead_dist_) || goal_terminal_lookahead_dist_ <= 0.0 || goal_terminal_lookahead_dist_ > lookahead_dist_ || !std::isfinite(goal_terminal_lookahead_reference_speed_) || goal_terminal_lookahead_reference_speed_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_speed_gain_) || goal_terminal_lookahead_speed_gain_ < 0.0 || !std::isfinite(goal_terminal_lookahead_min_dist_) || goal_terminal_lookahead_min_dist_ <= 0.0 || !std::isfinite(goal_terminal_lookahead_max_dist_) || goal_terminal_lookahead_max_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ < goal_terminal_lookahead_min_dist_ || goal_terminal_lookahead_dist_ > goal_terminal_lookahead_max_dist_ || goal_error_log_frequency_ <= 0.0 || alignment_stable_cycles_ < 1 || transform_tolerance_ < 0.0 || controller_frequency <= 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController parameters are invalid");
  }
  if (!std::isfinite(direct_tracking_lateral_tolerance_) || !std::isfinite(start_offset_speed_limit_) || start_offset_speed_limit_ <= 0.0 || !std::isfinite(start_speed_limit_distance_) || start_speed_limit_distance_ <= 0.0 || !std::isfinite(start_speed_release_yaw_tolerance_) || start_speed_release_yaw_tolerance_ <= 0.0 || start_speed_release_yaw_tolerance_ >= M_PI || start_speed_release_stable_cycles_ < 1)
  {
    throw nav2_core::PlannerException("FixedPathController start speed guard parameters are invalid");
  }
  speed_limit_ = base_linear_velocity_;
  if (!std::isfinite(goal_stop_entry_tolerance_) || goal_stop_entry_tolerance_ < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController stop entry tolerance must be finite and nonnegative");
  }
  if (dynamic_goal_braking_margin_enabled_ && !detail::validBrakingMargin(goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_))
  {
    throw nav2_core::PlannerException("FixedPathController dynamic braking margin parameters are invalid");
  }
  LOG_INFO("连续制动裕量：enabled={}，minimum={:.3f}m，high={:.3f}m，transition={:.3f}m/s，adaptive_branch={}", dynamic_goal_braking_margin_enabled_, goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_, adaptive_goal_braking_enabled_);
  control_duration_ = 1.0 / controller_frequency;
  // 仅在显式启用新制动路径时校验专属参数；关闭时不改变旧配置的接受条件。
  if (adaptive_goal_braking_enabled_ && (!std::isfinite(adaptive_goal_max_deceleration_) || adaptive_goal_max_deceleration_ <= 0.0 || adaptive_goal_max_deceleration_ > 1.0 || !std::isfinite(adaptive_goal_jerk_limit_) || adaptive_goal_jerk_limit_ <= 0.0 || !std::isfinite(adaptive_goal_approach_speed_) || adaptive_goal_approach_speed_ <= 0.0 || !std::isfinite(adaptive_goal_response_time_) || adaptive_goal_response_time_ < 0.0 || !std::isfinite(adaptive_goal_distance_margin_) || adaptive_goal_distance_margin_ < 0.0))
  {
    throw nav2_core::PlannerException("FixedPathController adaptive braking parameters are invalid");
  }
  LOG_INFO("固定路径控制器配置完成，plugin={}，最大线速度={:.3f}m/s，终点减速度={:.3f}m/s^2，终点微量接近速度={:.3f}m/s，制动反应时间={:.3f}s，制动距离裕量={:.3f}m，终点基础前视={:.3f}m，前视参考速度={:.3f}m/s，前视速度增益={:.3f}s，前视范围=[{:.3f},{:.3f}]m，起点位置容差={:.3f}m，直接跟踪横向容差={:.3f}m，直接跟踪航向门限={:.3f}rad，严格对齐航向容差={:.3f}rad，终点按位置或越界锁存停车", plugin_name_, base_linear_velocity_, goal_linear_deceleration_, goal_final_approach_velocity_, goal_braking_reaction_time_, goal_braking_distance_margin_, goal_terminal_lookahead_dist_, goal_terminal_lookahead_reference_speed_, goal_terminal_lookahead_speed_gain_, goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_, start_position_tolerance_, direct_tracking_lateral_tolerance_, direct_tracking_max_yaw_error_, initial_yaw_tolerance_);
  LOG_INFO("固定路径起步限速配置：每条路径保护距离={:.3f}m，限速={:.3f}m/s，解除横向误差≤{:.3f}m，航向误差≤{:.3f}rad，连续周期={}", start_speed_limit_distance_, start_offset_speed_limit_, direct_tracking_lateral_tolerance_, start_speed_release_yaw_tolerance_, start_speed_release_stable_cycles_);
}

// 释放插件资源并复位控制器内部阶段与路径状态。
void FixedPathController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = nav_msgs::msg::Path();
  start_path_points_.clear();
  start_speed_guard_.reset();
  nearest_index_ = 0;
  start_speed_prebraking_ = false;
  goal_tangent_index_ = 0;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  last_braking_deceleration_ = 0.0;
  last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  terminal_tangent_x_ = 0.0;
  terminal_tangent_y_ = 0.0;
  error_log_initialized_ = false;
}

// 记录插件激活事件；此函数不单独建立订阅或输出速度。
void FixedPathController::activate()
{
  LOG_INFO("固定路径控制器已激活，plugin={}", plugin_name_);
}

// 将控制阶段设为停止并锁存终点停车，后续计算不再继续跟踪旧路径。
void FixedPathController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::STOPPED;
  terminal_stop_latched_ = true;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  stable_cycles_ = 0;
  LOG_INFO("固定路径控制器已停用，plugin={}", plugin_name_);
}

// 装入新的跟踪路径；重置此前路径索引、终点锁存及进度数据。
void FixedPathController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 缺少坐标系或不足两个点时无法定义起点、终点及运动切线。
  if (path.poses.size() < 2 || path.header.frame_id.empty())
  {
    throw nav2_core::PlannerException("Fixed path requires a frame and at least two poses");
  }
  std::size_t tangent_index = 1;
  // 跳过与起点重合的前缀点，寻找首段真正具有长度的路径切线。
  while (tangent_index < path.poses.size() && poseDistance(path.poses.front(), path.poses[tangent_index]) <= 1e-6)
  {
    ++tangent_index;
  }
  // 所有路径点均与起点重合时，无法计算前进／后退的初始方向。
  if (tangent_index >= path.poses.size())
  {
    throw nav2_core::PlannerException("Fixed path has no valid tangent");
  }
  std::size_t goal_tangent_index = path.poses.size() - 2;
  // 从终点向前跳过重复点，选出最后一段有效切线用于越界停车判定。
  while (goal_tangent_index > 0 && poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    --goal_tangent_index;
  }
  // 回溯后仍无有效末段，终点平面和末段延长方向都无法定义。
  if (poseDistance(path.poses[goal_tangent_index], path.poses.back()) <= 1e-6)
  {
    throw nav2_core::PlannerException("Fixed path has no valid goal tangent");
  }
  const double start_path_yaw = std::atan2(path.poses[tangent_index].pose.position.y - path.poses.front().pose.position.y, path.poses[tangent_index].pose.position.x - path.poses.front().pose.position.x);
  const double goal_tangent_x = path.poses.back().pose.position.x - path.poses[goal_tangent_index].pose.position.x;
  const double goal_tangent_y = path.poses.back().pose.position.y - path.poses[goal_tangent_index].pose.position.y;
  const double goal_tangent_length = std::hypot(goal_tangent_x, goal_tangent_y);
  const double goal_path_yaw = std::atan2(goal_tangent_y, goal_tangent_x);
  const double direction_cosine = std::cos(normalizeAngle(poseYaw(path.poses.front()) - start_path_yaw));
  // 首点车头朝向必须能明确区分沿切线前进还是逆切线倒车，近乎垂直则拒绝。
  if (!std::isfinite(direction_cosine) || std::abs(direction_cosine) < 0.5)
  {
    throw nav2_core::PlannerException("Fixed path orientation does not encode a clear direction");
  }
  global_plan_ = path;
  start_path_points_.clear();
  start_path_points_.reserve(path.poses.size());
  for (const auto & path_pose : path.poses)
  {
    start_path_points_.emplace_back(path_pose.pose.position.x, path_pose.pose.position.y);
  }
  start_speed_guard_.reset();
  nearest_index_ = 0;
  start_speed_prebraking_ = true;
  goal_tangent_index_ = goal_tangent_index;
  // 车头与起点切线同向记为前进，反向记为倒车；后续速度输出使用此符号。
  direction_sign_ = direction_cosine > 0.0 ? 1 : -1;
  start_path_yaw_ = start_path_yaw;
  goal_path_yaw_ = goal_path_yaw;
  terminal_tangent_x_ = goal_tangent_x / goal_tangent_length;
  terminal_tangent_y_ = goal_tangent_y / goal_tangent_length;
  stable_cycles_ = 0;
  start_strategy_evaluated_ = false;
  direct_start_tracking_ = false;
  goal_braking_active_ = false;
  terminal_stop_latched_ = false;
  terminal_position_accurate_ = false;
  terminal_tracking_yaw_error_ = 0.0;
  last_braking_command_magnitude_ = 0.0;
  last_braking_deceleration_ = 0.0;
  last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  phase_ = Phase::ALIGN_START;
  error_log_initialized_ = false;
  LOG_INFO("固定路径已装载，frame={}，路径点数={}，方向={}，起点切线={:.6f}rad，终点切线={:.6f}rad", global_plan_.header.frame_id, global_plan_.poses.size(), direction_sign_ > 0 ? "forward" : "backward", start_path_yaw_, goal_path_yaw_);
}

// 根据当前位姿与路径阶段生成速度指令；各安全边界可使输出归零。
geometry_msgs::msg::TwistStamped FixedPathController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 路径和 GoalChecker 是本周期求速度与到点容差的前提，缺失时不能继续控制。
  if (global_plan_.poses.size() < 2 || !goal_checker)
  {
    throw nav2_core::PlannerException("Fixed path controller has no valid plan or goal checker");
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  // 所有路径距离均在路径坐标系中计算，TF 失败不能用原始位姿代替。
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into fixed path frame");
  }
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  // 目标检查器提供当前位置和停稳速度门槛；读取失败时无法安全判定终点。
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance))
  {
    throw nav2_core::PlannerException("FixedPathController failed to read StoppedGoalChecker tolerances");
  }
  const double goal_xy_tolerance = pose_tolerance.position.x;
  const double linear_stopped_velocity = velocity_tolerance.linear.x;
  const double angular_stopped_velocity = velocity_tolerance.angular.z;
  if (goal_stop_entry_tolerance_ > goal_xy_tolerance)
  {
    throw nav2_core::PlannerException("FixedPathController stop entry tolerance exceeds goal tolerance");
  }
  const double stop_tolerance = goal_stop_entry_tolerance_ > 0.0 ? goal_stop_entry_tolerance_ : goal_xy_tolerance;
  // 拒绝非有限或负容差，避免后续停车锁存和停稳判定失去边界。
  if (!std::isfinite(goal_xy_tolerance) || !std::isfinite(linear_stopped_velocity) || !std::isfinite(angular_stopped_velocity) || goal_xy_tolerance <= 0.0 || linear_stopped_velocity < 0.0 || angular_stopped_velocity < 0.0)
  {
    throw nav2_core::PlannerException("FixedPathController requires valid position and stopped-velocity tolerances");
  }
  // 新制动路径依赖实测速度；只有启用该路径时才对无效反馈执行零速保护。
  if (adaptive_goal_braking_enabled_ && (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z)))
  {
    LOG_ERROR("固定路径里程计速度无效，输出零速保护");
    return zeroCommand();
  }
  const double start_distance = poseDistance(robot_pose, global_plan_.poses.front());
  // 移动中接收新路径时先平滑减速，不把减速和原地转向计入保护距离。
  if (start_speed_prebraking_)
  {
    if (detail::requiresStartPrebraking(velocity.linear.x, std::min(start_offset_speed_limit_, speed_limit_)))
    {
      LOG_DEBUG("固定路径起步预减速：实测速度={:.6f}，起步上限={:.6f}", velocity.linear.x, std::min(start_offset_speed_limit_, speed_limit_));
      return zeroCommand();
    }
    start_speed_prebraking_ = false;
  }
  // 起点阶段先决定是否需要原地转向；进入 TRACK_PATH 后不再重新选择策略。
  if (phase_ == Phase::ALIGN_START)
  {
    // 机器人离首点过远时，不允许直接驶向路径以补偿起点偏差。
    if (start_distance > start_position_tolerance_)
    {
      throw nav2_core::PlannerException("Robot is outside fixed path start position tolerance");
    }
    // 倒车时以车体反向轴作为运动方向，统一与路径切线比较航向误差。
    const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
    const double yaw_error = normalizeAngle(start_path_yaw_ - vehicle_motion_yaw);
    const double start_delta_x = robot_pose.pose.position.x - global_plan_.poses.front().pose.position.x;
    const double start_delta_y = robot_pose.pose.position.y - global_plan_.poses.front().pose.position.y;
    const double lateral_error = std::abs(-std::sin(start_path_yaw_) * start_delta_x + std::cos(start_path_yaw_) * start_delta_y);
    // 每条新路径只根据首次横向误差锁定一次起点策略，避免阈值附近来回切换。
    if (!start_strategy_evaluated_)
    {
      direct_start_tracking_ = lateral_error <= direct_tracking_lateral_tolerance_;
      start_strategy_evaluated_ = true;
      if (start_speed_guard_.evaluateStart())
      {
        LOG_INFO("固定路径起步限速已进入：横向偏差={:.3f}m，航向误差={:.3f}rad，纵向上限={:.3f}m/s，保护距离={:.3f}m", lateral_error, yaw_error, start_offset_speed_limit_, start_speed_limit_distance_);
      }
      LOG_INFO("固定路径起点策略已锁定，方向={}，起点距离={:.3f}m，横向误差={:.3f}m，运动方向航向误差={:.3f}rad，策略={}", direction_sign_ > 0 ? "forward" : "backward", start_distance, lateral_error, yaw_error, direct_start_tracking_ ? "direct_tracking" : "strict_alignment");
    }
    // 横向误差较小时采用较宽的航向门槛，尽量直接进入路径跟踪。
    if (direct_start_tracking_)
    {
      // 已满足直接跟踪门槛，切换状态并继续执行本周期的路径控制。
      if (std::abs(yaw_error) <= direct_tracking_max_yaw_error_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径小横向误差直接进入跟踪，方向={}，横向误差={:.3f}m，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", lateral_error, yaw_error);
      }
      else
      {
        // 航向尚未进入宽门槛，本周期只输出原地转向，不输出纵向速度。
        return rotateCommand(yaw_error, direct_tracking_max_yaw_error_, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
      }
    }
    else
    {
      // 横向偏差较大时，只按更严的航向容差决定何时结束原地旋转，不再等待车速停稳。
      if (std::abs(yaw_error) <= initial_yaw_tolerance_)
      {
        ++stable_cycles_;
      }
      else
      {
        // 航向重新超出容差时清零计数，避免累计不连续的对齐周期。
        stable_cycles_ = 0;
      }
      // 默认只需当前周期达标：立即切到跟踪，并沿本周期后续逻辑输出受限纵向速度。
      if (stable_cycles_ >= alignment_stable_cycles_)
      {
        phase_ = Phase::TRACK_PATH;
        stable_cycles_ = 0;
        LOG_INFO("固定路径严格起点航向对齐完成并进入低速跟踪，方向={}，运动方向航向误差={:.3f}rad", direction_sign_ > 0 ? "forward" : "backward", yaw_error);
      }
      else
      {
        // 严格分支已单独判断 3 度切换门槛；转向指令按完整误差计算，避免在门槛外速度趋零而迟迟无法进入跟踪。
        return rotateCommand(yaw_error, 0.0, rotate_to_heading_angular_vel_, max_angular_accel_, rotate_to_heading_kp_, velocity);
      }
    }
  }
  const double goal_distance = poseDistance(robot_pose, global_plan_.poses.back());
  // 终点航向只用于跟踪诊断；倒车仍按车体反向轴计算，不参与终点成功条件。
  const double vehicle_motion_yaw = normalizeAngle(poseYaw(robot_pose) + (direction_sign_ < 0 ? M_PI : 0.0));
  const double goal_yaw_error = normalizeAngle(goal_path_yaw_ - vehicle_motion_yaw);
  nearest_index_ = findNearestIndex(robot_pose);
  // 仅在已触发起步限速时复核当前最近有效路径段；连续达标后单向解除。
  if (start_speed_guard_.active())
  {
    double local_lateral_error = 0.0;
    double local_heading_error = 0.0;
    double start_progress = 0.0;
    if (detail::calculateStartRecoveryErrors(start_path_points_, nearest_index_, robot_pose.pose.position.x, robot_pose.pose.position.y, poseYaw(robot_pose), direction_sign_, local_lateral_error, local_heading_error, &start_progress))
    {
      LOG_DEBUG("固定路径起步保护：路径进度={:.6f}m，保护距离={:.6f}m，横向误差={:.6f}m，航向误差={:.6f}rad", start_progress, start_speed_limit_distance_, local_lateral_error, local_heading_error);
      if (start_speed_guard_.observe(local_lateral_error, local_heading_error, direct_tracking_lateral_tolerance_, start_speed_release_yaw_tolerance_, start_speed_release_stable_cycles_, start_progress, start_speed_limit_distance_))
      {
        LOG_INFO("固定路径起步限速已解除，路径进度={:.3f}m，横向误差={:.3f}m，运动方向航向误差={:.3f}rad，连续达标周期={}", start_progress, local_lateral_error, local_heading_error, start_speed_release_stable_cycles_);
      }
    }
    else
    {
      start_speed_guard_.resetStability();
    }
  }
  const double effective_linear_limit = std::min(std::min(base_linear_velocity_, speed_limit_), start_speed_guard_.active() ? start_offset_speed_limit_ : base_linear_velocity_);
  LOG_DEBUG("固定路径起步速度链：保护={}，任务限速={:.6f}，有效上限={:.6f}", start_speed_guard_.active(), speed_limit_, effective_linear_limit);
  const auto & goal_position = global_plan_.poses.back().pose.position;
  const double goal_delta_x = robot_pose.pose.position.x - goal_position.x;
  const double goal_delta_y = robot_pose.pose.position.y - goal_position.y;
  const double terminal_projection = goal_delta_x * terminal_tangent_x_ + goal_delta_y * terminal_tangent_y_;
  // 只有已跟踪到末段且沿末段切线越过终点平面，才视为纵向越界。
  const bool goal_plane_crossed = nearest_index_ >= goal_tangent_index_ && terminal_projection >= 0.0;
  const double remaining = remainingDistance(nearest_index_, robot_pose);
  const double effective_remaining = std::max(remaining - stop_tolerance, 0.0);
  // 旧模式只取运动方向上的正速度；新模式取绝对值，避免反向残余速度低估停车距离。
  const double current_linear_velocity = adaptive_goal_braking_enabled_ ? std::abs(velocity.linear.x) : std::max(0.0, static_cast<double>(direction_sign_) * velocity.linear.x);
  // 自适应关闭时不调用新包络；旧模式仍以固定减速度计算制动距离。
  const auto speed_profile = adaptive_goal_braking_enabled_ ? calculateFixedPathSpeedProfile(current_linear_velocity, remaining, effective_linear_limit, adaptive_goal_max_deceleration_, adaptive_goal_jerk_limit_, adaptive_goal_response_time_, adaptive_goal_distance_margin_, stop_tolerance, adaptive_goal_approach_speed_) : FixedPathSpeedProfile{0.0, 0.0, 0.0};
  const double stopping_distance = adaptive_goal_braking_enabled_ ? speed_profile.stopping_distance : current_linear_velocity * current_linear_velocity / (2.0 * goal_linear_deceleration_);
  const double expected_linear_velocity = std::max(0.0, effective_linear_limit);
  // 末段前视随有效速度变化，但始终限制在配置的最小和最大距离之间。
  const double dynamic_terminal_lookahead_distance = std::clamp(goal_terminal_lookahead_dist_ + goal_terminal_lookahead_speed_gain_ * (expected_linear_velocity - goal_terminal_lookahead_reference_speed_), goal_terminal_lookahead_min_dist_, goal_terminal_lookahead_max_dist_);
  // 进入 XY 容差或越过终点平面都会触发零速锁存，避免继续向前驶离终点。
  const auto terminal_position = detail::terminalPosition(goal_distance, goal_xy_tolerance, goal_stop_entry_tolerance_, goal_plane_crossed, terminal_stop_latched_, terminal_position_accurate_);
  const bool terminal_condition_reached = terminal_position.stopped;
  // 首次触发终点条件时锁存停车；位置是否准确单独记录，不把越界误报为到点成功。
  if (!terminal_stop_latched_ && terminal_condition_reached)
  {
    // 新模式若在明显非低速状态触发终点保护，记录异常以区分正常制动与安全停车。
    if (adaptive_goal_braking_enabled_ && current_linear_velocity > 0.02)
    {
      LOG_WARN("固定路径终点在非低速状态下触发安全停车，实测线速度={:.4f}m/s", current_linear_velocity);
    }
    terminal_stop_latched_ = true;
    terminal_position_accurate_ = terminal_position.accurate;
    terminal_tracking_yaw_error_ = goal_yaw_error;
    phase_ = Phase::STOPPED;
    LOG_INFO("固定路径终点停车已锁存，原因={}，位置误差={:.4f}m，终点纵向投影={:.4f}m，锁存跟踪航向误差={:.3f}rad（{:.2f}deg），停车后不再旋转", goal_distance <= stop_tolerance ? "within_tolerance" : "goal_plane_crossed", goal_distance, terminal_projection, terminal_tracking_yaw_error_, terminal_tracking_yaw_error_ * 180.0 / M_PI);
  }
  // 旧模式允许停车后再次进入位置容差时单向锁存精度，兼容已有成功判定。
  else if (terminal_stop_latched_ && goal_stop_entry_tolerance_ > 0.0)
  {
    // 实验入口启用时逐周期复核当前验收半径，不保留历史准确状态。
    terminal_position_accurate_ = terminal_position.accurate;
  }
  else if (terminal_stop_latched_ && !terminal_position_accurate_ && goal_distance <= goal_xy_tolerance)
  {
    terminal_position_accurate_ = true;
    LOG_INFO("固定路径停车后位置精度已进入容差并单向锁存，位置误差={:.4f}m，位置容差={:.4f}m", goal_distance, goal_xy_tolerance);
  }
  // 已停车锁存后不再计算前视、曲率或前进指令；只有停稳且位置达标才可能成功。
  if (terminal_stop_latched_)
  {
    // 新模式逐周期复核当前 XY 误差，不沿用曾短暂经过容差时的历史位置。
    if (adaptive_goal_braking_enabled_)
    {
      // 位置精度使用当前位姿，而非越过终点瞬间的锁存结果。
      terminal_position_accurate_ = goal_distance <= goal_xy_tolerance;
      // 已停稳仍在容差外时明确报告失败，由控制服务器终止当前 FollowPath。
      if (!terminal_position_accurate_ && current_linear_velocity <= linear_stopped_velocity)
      {
        throw nav2_core::PlannerException("Fixed path stopped beyond terminal position tolerance");
      }
    }
    logGoalErrors(goal_distance, terminal_tracking_yaw_error_, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, 0.0, stopping_distance, dynamic_terminal_lookahead_distance);
    return zeroCommand();
  }
  const double nominal_lookahead_distance = std::clamp(std::max(lookahead_dist_, std::abs(velocity.linear.x) * lookahead_time_), min_lookahead_dist_, max_lookahead_dist_);
  // 远离终点按速度选常规前视；接近终点时收缩到不小于末段动态前视的距离。
  const double lookahead_distance = std::min(nominal_lookahead_distance, std::max(dynamic_terminal_lookahead_distance, goal_distance));
  auto carrot = selectCarrot(nearest_index_, lookahead_distance);
  carrot.header.frame_id = global_plan_.header.frame_id;
  carrot.header.stamp = pose.header.stamp;
  geometry_msgs::msg::PoseStamped local_carrot;
  // 曲率需要车体坐标系下的前视点；转换失败时不能用错误坐标计算转向。
  if (!transformPose(costmap_ros_->getBaseFrameID(), carrot, local_carrot))
  {
    throw nav2_core::PlannerException("Unable to transform fixed path lookahead point into robot frame");
  }
  const double carrot_distance_squared = local_carrot.pose.position.x * local_carrot.pose.position.x + local_carrot.pose.position.y * local_carrot.pose.position.y;
  // 前视点几乎与车体重合时曲率分母趋零，改用零曲率避免速度命令发散。
  const double curvature = carrot_distance_squared > 1e-6 ? 2.0 * local_carrot.pose.position.y / carrot_distance_squared : 0.0;
  double linear_magnitude = effective_linear_limit;
  // 弯道曲率非零时再受最大角速度约束，直线段保持原线速度上限。
  if (std::abs(curvature) > 1e-6)
  {
    linear_magnitude = std::min(linear_magnitude, rotate_to_heading_angular_vel_ / std::abs(curvature));
  }
  const double effective_braking_margin = detail::brakingMargin(dynamic_goal_braking_margin_enabled_, adaptive_goal_braking_enabled_, velocity.linear.x, goal_braking_min_distance_margin_, goal_braking_distance_margin_, goal_braking_margin_transition_speed_);
  const double braking_reaction_distance = current_linear_velocity * goal_braking_reaction_time_ + effective_braking_margin;
  // 新模式使用含响应延迟和 jerk 预留的停车距离；旧模式沿用固定减速度加反应裕量。
  const double braking_activation_distance = adaptive_goal_braking_enabled_ ? std::max(approach_velocity_scaling_dist_, stopping_distance) : std::max(approach_velocity_scaling_dist_, stopping_distance + braking_reaction_distance);
  // 首次进入减速区才锁定制动状态与当前命令，防止剩余距离波动反复切换阶段。
  if (!goal_braking_active_ && effective_remaining <= braking_activation_distance)
  {
    goal_braking_active_ = true;
    last_braking_command_magnitude_ = linear_magnitude;
    // 新路径初始化 jerk 限制器的减速度和时间戳，后续周期连续推进。
    if (adaptive_goal_braking_enabled_)
    {
      last_braking_deceleration_ = 0.0;
      last_braking_update_time_ = clock_->now();
      LOG_INFO("固定路径进入自适应终点制动，剩余距离={:.4f}m，当前速度={:.4f}m/s，预留停车距离={:.4f}m，本周期减速度上限={:.3f}m/s^2", remaining, current_linear_velocity, stopping_distance, speed_profile.deceleration);
    }
    else
    {
      // 旧路径不维护 jerk 状态，继续使用既有制动参数和日志语义。
      LOG_INFO("固定路径进入终点平滑制动，剩余距离={:.4f}m，当前速度={:.4f}m/s，制动距离={:.4f}m，目标减速度={:.3f}m/s^2", remaining, current_linear_velocity, stopping_distance, goal_linear_deceleration_);
    }
  }
  // 只有制动阶段才改变纵向目标速度；此前由限速及曲率约束控制。
  if (goal_braking_active_)
  {
    // 显式启用时采用实测车速包络和逐周期 jerk 限制。
    if (adaptive_goal_braking_enabled_)
    {
      const auto update_time = clock_->now();
      double dt = control_duration_;
      // 有上次更新时间时使用真实周期；首周期使用配置的控制周期。
      if (last_braking_update_time_.nanoseconds() != 0)
      {
        dt = (update_time - last_braking_update_time_).seconds();
        // 仿真时钟倒退会使历史制动积分失效，重置状态并输出零速。
        if (dt < 0.0)
        {
          goal_braking_active_ = false;
          last_braking_deceleration_ = 0.0;
          last_braking_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          LOG_WARN("固定路径控制时钟回跳，重置制动状态并输出零速");
          return zeroCommand();
        }
        // 同一仿真时间戳可能覆盖多个控制回调，使用名义周期避免除零或停滞。
        if (dt == 0.0)
        {
          dt = control_duration_;
        }
      }
      // 限定时间步长，防止长时间调度间隔一次性放大命令变化。
      dt = std::clamp(dt, 0.005, 0.1);
      const double braking_target = std::min(linear_magnitude, speed_profile.speed_limit);
      double next_deceleration = 0.0;
      // 按减速度及 jerk 双重约束逼近目标；输出再与曲率速度上限取小值。
      const double braking_command = advanceFixedPathBrakingSpeed(last_braking_command_magnitude_, braking_target, last_braking_deceleration_, adaptive_goal_max_deceleration_, adaptive_goal_jerk_limit_, dt, next_deceleration);
      linear_magnitude = std::min(linear_magnitude, braking_command);
      last_braking_command_magnitude_ = linear_magnitude;
      last_braking_deceleration_ = next_deceleration;
      last_braking_update_time_ = update_time;
    }
    else
    {
      // 旧路径先扣除反应距离，再按 v²=2ad 求允许的终点线速度。
      const double braking_distance_remaining = std::max(effective_remaining - braking_reaction_distance, 0.0);
      double braking_target = std::min(linear_magnitude, std::sqrt(2.0 * goal_linear_deceleration_ * braking_distance_remaining));
      // 尚未进入位置容差时保留配置的微量接近速度，避免提前完全停住。
      if (goal_distance > stop_tolerance)
      {
        braking_target = std::min(linear_magnitude, std::max(braking_target, goal_final_approach_velocity_));
      }
      // 旧制动阶段只允许目标速度单调下降，避免定位抖动导致二次加速。
      linear_magnitude = detail::boundedBrakingCommand(last_braking_command_magnitude_, braking_target);
      last_braking_command_magnitude_ = linear_magnitude;
    }
  }
  logGoalErrors(goal_distance, goal_yaw_error, goal_xy_tolerance, terminal_projection, goal_plane_crossed, remaining, current_linear_velocity, linear_magnitude, stopping_distance, dynamic_terminal_lookahead_distance);
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  // 路径方向决定纵向命令正负；曲率再映射为受上限约束的角速度。
  command.twist.linear.x = static_cast<double>(direction_sign_) * linear_magnitude;
  command.twist.angular.z = std::clamp(command.twist.linear.x * curvature, -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);
  return command;
}

// 更新控制器允许速度；区分百分比限制与绝对速度限制。
void FixedPathController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2 的“无限速”哨兵值表示恢复插件基础速度，而不是要求停车。
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
  {
    speed_limit_ = base_linear_velocity_;
    return;
  }
  // 非有限或负限速不覆盖当前有效值，避免异常消息污染速度链。
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
  {
    LOG_WARN("忽略非法固定路径限速：{}", speed_limit);
    return;
  }
  // 百分比以基础速度为基数，绝对值直接按 m/s 限幅；两者均不能超过基础速度。
  speed_limit_ = percentage ? base_linear_velocity_ * std::clamp(speed_limit / 100.0, 0.0, 1.0) : std::min(base_linear_velocity_, speed_limit);
}

// 读取终点停车锁存状态；锁存后不再输出继续跟踪命令。
bool FixedPathController::isTerminalStopLatched()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_stop_latched_;
}

// 只检查终点位置精度，供任务成功判定使用。
bool FixedPathController::isTerminalPositionAccurate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return terminal_position_accurate_;
}

// 通过 TF 把输入位姿转换到指定坐标系；转换异常时返回 false。
bool FixedPathController::transformPose(const std::string & frame, const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output) const
{
  // 坐标系已一致时直接复制，避免不必要的 TF 查询及时间外推失败。
  if (input.header.frame_id == frame)
  {
    output = input;
    return true;
  }
  // 坐标系不同才在配置的 TF 容差内请求变换。
  try
  {
    tf_->transform(input, output, frame, tf2::durationFromSec(transform_tolerance_));
    output.header.frame_id = frame;
    return true;
  }
  // TF 不可用时返回失败，由上层拒绝用未转换的位姿计算控制命令。
  catch (const tf2::TransformException & error)
  {
    RCLCPP_ERROR(logger_, "Fixed path transform failed: %s", error.what());
    return false;
  }
}

// 从当前路径搜索区间选出距机器人最近的轨迹点索引。
std::size_t FixedPathController::findNearestIndex(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::size_t best_index = nearest_index_;
  double best_distance = std::numeric_limits<double>::max();
  // 仅向前搜索，防止定位抖动让最近点索引回退并重复跟踪已通过路径。
  for (std::size_t index = nearest_index_; index < global_plan_.poses.size(); ++index)
  {
    const double distance = poseDistance(robot_pose, global_plan_.poses[index]);
    // 发现更近的候选点才更新索引；相等时保留先遇到的路径点。
    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

// 估算当前位姿至路径终点的剩余行程。
double FixedPathController::remainingDistance(const std::size_t start_index, const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  double distance = poseDistance(robot_pose, global_plan_.poses[start_index]);
  // 当前位姿到最近路径点的距离加上其后各段弧长，得到控制器使用的剩余行程。
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    distance += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
  }
  return distance;
}

// 按前视距离选择路径目标点，必要时对末段进行受限延长。
geometry_msgs::msg::PoseStamped FixedPathController::selectCarrot(const std::size_t start_index, const double lookahead_distance) const
{
  double accumulated = 0.0;
  // 沿离散路径累计弧长，优先选第一个达到前视距离的实际路径点。
  for (std::size_t index = start_index + 1; index < global_plan_.poses.size(); ++index)
  {
    accumulated += poseDistance(global_plan_.poses[index - 1], global_plan_.poses[index]);
    // 距离已满足时直接返回路径点，不对中途段进行外推。
    if (accumulated >= lookahead_distance)
    {
      return global_plan_.poses[index];
    }
  }
  // 路径末段不足前视距离时沿终点切线延长 carrot，避免终点前视塌缩。
  auto carrot = global_plan_.poses.back();
  const double extension_distance = std::max(lookahead_distance - accumulated, 0.0);
  carrot.pose.position.x += terminal_tangent_x_ * extension_distance;
  carrot.pose.position.y += terminal_tangent_y_ * extension_distance;
  return carrot;
}

// 构造全零 Twist，统一用于停车或无有效路径场景。
geometry_msgs::msg::TwistStamped FixedPathController::zeroCommand() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();
  return command;
}

// 将航向误差换算为受角速度与加速度约束的原地转向指令。
geometry_msgs::msg::TwistStamped FixedPathController::rotateCommand(const double yaw_error, const double yaw_tolerance, const double max_angular_velocity, const double max_angular_accel, const double angular_kp, const geometry_msgs::msg::Twist & velocity) const
{
  auto command = zeroCommand();
  const double remaining_error = std::abs(yaw_error) - yaw_tolerance;
  // 航向已进入容差时不继续旋转，直接保持零速度。
  if (remaining_error <= 0.0)
  {
    return command;
  }
  // 航向误差正负决定转向方向；幅值同时受比例项、最大角速度和制动能力约束。
  const double direction = yaw_error > 0.0 ? 1.0 : -1.0;
  const double proportional_velocity = angular_kp * remaining_error;
  const double stopping_velocity = std::sqrt(2.0 * max_angular_accel * remaining_error);
  const double target_velocity = direction * std::min({max_angular_velocity, proportional_velocity, stopping_velocity});
  command.twist.angular.z = std::clamp(target_velocity, velocity.angular.z - max_angular_accel * control_duration_, velocity.angular.z + max_angular_accel * control_duration_);
  return command;
}

// 按设定频率记录终点位置和航向误差；不改变停车判定。
void FixedPathController::logGoalErrors(const double position_error, const double yaw_error, const double goal_xy_tolerance, const double terminal_projection, const bool goal_plane_crossed, const double remaining_distance, const double current_linear_velocity, const double target_linear_velocity, const double stopping_distance, const double terminal_lookahead_distance)
{
  const auto now = clock_->now();
  const double log_period = 1.0 / goal_error_log_frequency_;
  // 仿真时钟未回跳且距上次记录不足周期时跳过日志，不影响速度计算。
  if (error_log_initialized_ && now.nanoseconds() >= last_error_log_time_.nanoseconds() && (now - last_error_log_time_).seconds() < log_period)
  {
    return;
  }
  last_error_log_time_ = now;
  error_log_initialized_ = true;
  LOG_INFO("固定路径终点误差：phase={}，位置误差={:.4f}m，路径剩余={:.4f}m，位置容差={:.4f}m，当前速度={:.4f}m/s，目标速度={:.4f}m/s，终点前视={:.4f}m，制动距离={:.4f}m，平滑制动={}，终点纵向投影={:.4f}m，越界={}，停车锁存={}，位置精度锁存={}，跟踪航向误差仅供诊断={:.3f}rad（{:.2f}deg）", phaseName(phase_), position_error, remaining_distance, goal_xy_tolerance, current_linear_velocity, target_linear_velocity, terminal_lookahead_distance, stopping_distance, goal_braking_active_, terminal_projection, goal_plane_crossed, terminal_stop_latched_, terminal_position_accurate_, yaw_error, yaw_error * 180.0 / M_PI);
}

// 返回当前控制阶段名称，供诊断日志辨识。
const char * FixedPathController::phaseName(const Phase phase)
{
  // 三个已定义控制阶段分别映射为稳定的诊断名称。
  switch (phase)
  {
    // 起点位置／航向检查阶段。
    case Phase::ALIGN_START:
      return "ALIGN_START";
    // 前视路径跟踪与终点制动阶段。
    case Phase::TRACK_PATH:
      return "TRACK_PATH";
    // 终点或停用后的零速锁存阶段。
    case Phase::STOPPED:
      return "STOPPED";
  }
  // 防御性兜底：遇到未列举枚举值时仍返回可读文本。
  return "UNKNOWN";
}

// 将角度规约到约定区间，避免跨越正负 π 时误判。
double FixedPathController::normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 从姿态四元数提取平面航向角，单位弧度。
double FixedPathController::poseYaw(const geometry_msgs::msg::PoseStamped & pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

// 计算两帧位姿的平面距离，单位米。
double FixedPathController::poseDistance(const geometry_msgs::msg::PoseStamped & first, const geometry_msgs::msg::PoseStamped & second)
{
  return std::hypot(first.pose.position.x - second.pose.position.x, first.pose.position.y - second.pose.position.y);
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::FixedPathController, nav2_core::Controller)
