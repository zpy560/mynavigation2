#include "nav2_regulated_modules/autonomous_fixed_controller.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <sstream>
#include <iomanip>

namespace nav2_regulated_modules
{
void AutonomousFixedPathController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  node_ = parent.lock();
  if (!node_)
  {
    throw nav2_core::PlannerException("Autonomous fixed controller has no parent");
  }
  tf_ = tf;
  costmap_ = costmap;
  tracker_.configure(parent, name + ".tracker", tf, costmap);
  tracker_.limitNearestSearch(0.5);
  const auto parameter = [&](const std::string & key, const double value)
  {
    nav2_util::declare_parameter_if_not_declared(node_, name + "." + key, rclcpp::ParameterValue(value));
    const double result = node_->get_parameter(name + "." + key).as_double();
    if (!std::isfinite(result) || result <= 0.0)
    {
      throw std::invalid_argument("Autonomous fixed controller parameter must be positive: " + key);
    }
    return result;
  };
  settle_duration_ = parameter("settle_duration", 0.5);
  brake_stable_duration_ = parameter("brake_stable_duration", 0.2);
  terminal_review_timeout_ = parameter("terminal_review_timeout", 2.0);
  terminal_capture_margin_ = parameter("terminal_capture_margin", 0.02);
  trim_limits_.max_speed = parameter("terminal_trim_max_speed", 0.02);
  trim_limits_.max_step = parameter("terminal_trim_max_step", 0.02);
  trim_limits_.max_distance = parameter("terminal_trim_max_distance", 0.03);
  trim_limits_.timeout = parameter("terminal_trim_timeout", 5.0);
  trim_confirm_duration_ = parameter("terminal_trim_confirm_duration", 0.2);
  nav2_util::declare_parameter_if_not_declared(node_, name + ".terminal_trim_max_attempts", rclcpp::ParameterValue(2));
  const auto attempts = node_->get_parameter(name + ".terminal_trim_max_attempts").as_int();
  if (attempts < 1 || attempts > 2 || trim_limits_.max_speed > 0.02 || trim_limits_.max_step > 0.02 || trim_limits_.max_distance > 0.03 || trim_limits_.max_step > trim_limits_.max_distance || trim_limits_.timeout > 5.0 || trim_confirm_duration_ >= terminal_review_timeout_)
  {
    throw std::invalid_argument("Autonomous fixed trim exceeds bounded speed, distance, attempts or review limits");
  }
  trim_limits_.max_attempts = static_cast<unsigned int>(attempts);
  base_linear_speed_ = node_->get_parameter(name + ".tracker.desired_linear_vel").as_double();
  trim_speed_limit_ = std::min(base_linear_speed_, trim_limits_.max_speed);
  if (terminal_review_timeout_ <= settle_duration_)
  {
    throw std::invalid_argument("Terminal position review timeout must exceed settle duration");
  }
  max_angular_ = parameter("max_angular_velocity", 0.4);
  max_accel_ = parameter("max_angular_acceleration", 0.8);
  obstacle_timeout_ = parameter("obstacle_timeout", 60.0);
  localization_timeout_ = parameter("localization_timeout", 0.3);
  tracking_localization_timeout_ = parameter("tracking_localization_timeout", std::max(0.6, localization_timeout_));
  terminal_localization_distance_ = parameter("terminal_localization_distance", 0.5);
  short_pause_timeout_ = parameter("short_pause_timeout", 0.5);
  resume_stable_duration_ = parameter("resume_stable_duration", 0.15);
  resume_distance_ = parameter("resume_distance", 0.2);
  resume_linear_acceleration_ = parameter("resume_linear_acceleration", 0.5);
  if (resume_stable_duration_ >= short_pause_timeout_)
  {
    throw std::invalid_argument("Autonomous fixed resume stability must fit inside the short pause window");
  }
  if (tracking_localization_timeout_ < localization_timeout_)
  {
    throw std::invalid_argument("Autonomous fixed tracking localization timeout must cover the strict timeout");
  }
  period_ = 1.0 / node_->get_parameter("controller_frequency").as_double();
  collision_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_->getCostmap());
  status_pub_ = node_->create_publisher<std_msgs::msg::String>("autonomous_fixed/execution_status", rclcpp::QoS(10));
  // Controller Server 在加载插件后才构造 OdomSubscriber，故在这里先声明同一参数。
  nav2_util::declare_parameter_if_not_declared(node_, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
  const auto odom_topic = node_->get_parameter("odom_topic").as_string();
  // 与 Controller Server 一致：motion_state 使用底盘反馈，其余话题保留 Odometry 兼容。
  if (odom_topic.substr(odom_topic.find_last_of('/') + 1) == "motion_state")
  {
    odom_sub_ = node_->create_subscription<byd_custom_msgs::msg::MotionState>(odom_topic, rclcpp::SensorDataQoS(), [this](byd_custom_msgs::msg::MotionState::ConstSharedPtr sample)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_ = nav_msgs::msg::Odometry();
      odom_.header = sample->header;
      odom_.twist.twist.linear.x = sample->v_car;
      odom_.twist.twist.angular.z = sample->w_car;
      odom_received_ = std::chrono::steady_clock::now();
      has_odom_ = true;
    });
  }
  else
  {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr sample)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_ = *sample;
      odom_received_ = std::chrono::steady_clock::now();
      has_odom_ = true;
    });
  }
}

void AutonomousFixedPathController::activate()
{
  tracker_.activate();
}

void AutonomousFixedPathController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = false;
  terminal_review_.reset();
  trim_.reset();
  localization_pause_.reset();
  last_localized_pose_.reset();
  resume_ramp_active_ = false;
  tracker_.deactivate();
}

void AutonomousFixedPathController::cleanup()
{
  tracker_.cleanup();
  odom_sub_.reset();
  status_pub_.reset();
  collision_.reset();
  costmap_.reset();
  tf_.reset();
  node_.reset();
}

void AutonomousFixedPathController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto segments = detail::splitFixedPath(path);
  if (segments.size() != 1)
  {
    throw nav2_core::PlannerException("Autonomous fixed controller requires one direction segment");
  }
  path_ = path;
  cursor_ = 0;
  rotation_only_ = segments.front().rotation;
  trim_direction_ = segments.front().direction;
  if (!rotation_only_)
  {
    tracker_.setPlan(path);
  }
  tracker_loaded_ = !rotation_only_;
  terminal_ = rotation_only_;
  ready_ = false;
  terminal_review_.reset();
  trim_.reset();
  trim_paused_ = false;
  trim_confirm_stamp_.reset();
  trim_resume_since_.reset();
  blocked_.reset();
  clear_.reset();
  localization_hard_wait_ = false;
  localization_hard_reason_.clear();
  localization_pause_.reset();
  localization_fresh_since_.reset();
  localization_fresh_frames_ = 0;
  localization_latest_stamp_ = 0.0;
  last_localized_pose_.reset();
  last_localized_stamp_ = 0.0;
  resume_ramp_active_ = false;
  localization_reason_ = "NONE";
  last_status_reason_.clear();
  diagnostic_pause_duration_ = 0.0;
  diagnostic_tf_stamp_ = 0;
  diagnostic_tf_age_ = 1e9;
  diagnostic_odom_age_ = 1e9;
  diagnostic_receive_age_ = 1e9;
  collision_log_time_ = -1.0;
  health_log_time_ = -1.0;
  collision_reason_.clear();
  last_phase_.clear();
  terminal_log_phase_.clear();
  terminal_log_time_ = -1.0;
  status("TRACKING");
}

void AutonomousFixedPathController::setSpeedLimit(const double & speed, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.setSpeedLimit(speed, percentage);
  if (std::isfinite(speed) && speed >= 0.0)
  {
    const double limit = speed == nav2_costmap_2d::NO_SPEED_LIMIT ? base_linear_speed_ : percentage ? base_linear_speed_ * std::clamp(speed / 100.0, 0.0, 1.0) : speed;
    trim_speed_limit_ = std::min(base_linear_speed_, std::min(trim_limits_.max_speed, limit));
  }
}

bool AutonomousFixedPathController::isTerminalStopLatched()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

bool AutonomousFixedPathController::isTerminalPositionAccurate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

void AutonomousFixedPathController::status(const std::string & phase)
{
  const double time = node_->now().seconds();
  const std::string reason = phase == "WAIT_OBSTACLE" ? (collision_reason_.empty() ? "OBSTACLE_CLEARING" : collision_reason_) : localization_reason_;
  if (phase == last_phase_ && reason == last_status_reason_ && time >= last_status_time_ && time - last_status_time_ < 0.2)
  {
    return;
  }
  std::ostringstream stream;
  const auto finite_age = [](const double value)
  {
    return std::isfinite(value) ? value : 1e9;
  };
  stream << std::fixed << std::setprecision(6);
  stream << "{\"execution_id\":" << rclcpp::Time(path_.header.stamp).nanoseconds() << ",\"point_index\":" << cursor_ << ",\"phase\":\"" << phase << "\",\"reason\":\"" << reason << "\",\"tf_stamp_ns\":" << diagnostic_tf_stamp_ << ",\"tf_age\":" << finite_age(diagnostic_tf_age_) << ",\"odom_age\":" << finite_age(diagnostic_odom_age_) << ",\"receive_age\":" << finite_age(diagnostic_receive_age_) << ",\"pause_duration\":" << diagnostic_pause_duration_ << "}";
  std_msgs::msg::String message;
  message.data = stream.str();
  status_pub_->publish(message);
  const bool hard_transition = phase == "WAIT_LOCALIZATION" && phase != last_phase_;
  if (hard_transition || ((reason != last_status_reason_ || reason != "NONE") && (health_log_time_ < 0.0 || time < health_log_time_ || time - health_log_time_ >= 1.0)))
  {
    RCLCPP_INFO(node_->get_logger(), "Autonomous fixed execution health: execution=%lld phase=%s reason=%s tf_age=%.6f odom_age=%.6f receive_age=%.6f pause=%.3f", static_cast<long long>(rclcpp::Time(path_.header.stamp).nanoseconds()), phase.c_str(), reason.c_str(), diagnostic_tf_age_, diagnostic_odom_age_, diagnostic_receive_age_, diagnostic_pause_duration_);
    health_log_time_ = time;
  }
  last_phase_ = phase;
  last_status_reason_ = reason;
  last_status_time_ = time;
}

bool AutonomousFixedPathController::pauseForLocalization(const bool valid, const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const double stamp, const double steady_time)
{
  if (!valid || localization_pause_.paused_since || localization_hard_wait_)
  {
    const bool invalid_expired = localization_pause_.update(valid, steady_time, short_pause_timeout_);
    diagnostic_pause_duration_ = localization_pause_.duration(steady_time);
    // 只清连续验收；已消耗的位置复核时间、微调次数／位移／截止时间继续保留。
    ready_ = false;
    terminal_review_.brake_complete = false;
    terminal_review_.brake_since.reset();
    terminal_review_.settled_since.reset();
    terminal_review_.last_time.reset();
    trim_paused_ = trim_.active;
    trim_confirm_stamp_.reset();
    trim_resume_since_.reset();
    clear_.reset();
    if (!localization_hard_wait_ && invalid_expired)
    {
      localization_hard_wait_ = true;
      localization_hard_reason_ = "SHORT_PAUSE_EXPIRED";
    }
    if (valid && !localization_hard_wait_)
    {
      // 首次启动曾缺数据时，用第一帧新鲜定位建立恢复基准，仍等待后续源帧稳定。
      if (!last_localized_pose_)
      {
        last_localized_pose_ = pose;
        last_localized_time_ = steady_time;
        last_localized_stamp_ = stamp;
      }
      // 旧位姿可能已滞后接近 TF 门限；按源时间间隔允许真实运动，不能只用本地接收间隔。
      const double elapsed = std::max(std::max(0.0, steady_time - last_localized_time_), stamp - last_localized_stamp_);
      detail::FixedSegment segment;
      segment.last = path_.poses.size() - 1;
      const auto nearest = detail::advanceFixedCursor(path_, segment, cursor_, pose);
      if (stamp < last_localized_stamp_ || (localization_fresh_since_ && stamp < localization_latest_stamp_) || (!rotation_only_ && measured.linear.x * trim_direction_ < -0.01) || detail::fixedDistance(pose, *last_localized_pose_) > base_linear_speed_ * elapsed + 0.05 || std::abs(detail::fixedAngle(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(last_localized_pose_->pose.orientation))) > max_angular_ * elapsed + 0.1 || detail::fixedDistance(pose, path_.poses[nearest]) > resume_distance_)
      {
        localization_hard_wait_ = true;
        localization_hard_reason_ = "RESUME_POSE_DISCONTINUITY";
      }
      else if (stamp > last_localized_stamp_)
      {
        if (!localization_fresh_since_)
        {
          localization_fresh_since_ = steady_time;
          localization_first_fresh_stamp_ = stamp;
          localization_latest_stamp_ = stamp;
          localization_fresh_frames_ = 1;
        }
        else if (stamp > localization_latest_stamp_)
        {
          localization_latest_stamp_ = stamp;
          ++localization_fresh_frames_;
        }
        if (localization_fresh_frames_ >= 2 && stamp > localization_first_fresh_stamp_ && steady_time - *localization_fresh_since_ >= resume_stable_duration_)
        {
          localization_pause_.reset();
          localization_fresh_since_.reset();
          resume_command_ = measured;
          resume_command_time_ = steady_time;
          resume_ramp_active_ = true;
          localization_reason_ = "RESUMED";
          last_localized_pose_ = pose;
          last_localized_time_ = steady_time;
          last_localized_stamp_ = stamp;
          return false;
        }
        localization_reason_ = "RESUME_STABILIZING";
      }
      else
      {
        localization_fresh_since_.reset();
        localization_fresh_frames_ = 0;
        localization_reason_ = "RESUME_WAIT_NEW_TF";
      }
    }
    else if (!valid)
    {
      localization_fresh_since_.reset();
      localization_fresh_frames_ = 0;
    }
    if (localization_hard_wait_)
    {
      localization_reason_ = localization_hard_reason_;
    }
    status(localization_hard_wait_ ? "WAIT_LOCALIZATION" : "PAUSED_LOCALIZATION");
    return true;
  }
  diagnostic_pause_duration_ = 0.0;
  if (!last_localized_pose_ || stamp > last_localized_stamp_)
  {
    last_localized_pose_ = pose;
    last_localized_time_ = steady_time;
    last_localized_stamp_ = stamp;
  }
  return false;
}

geometry_msgs::msg::Twist AutonomousFixedPathController::limitResumeCommand(const geometry_msgs::msg::Twist & command, const double steady_time)
{
  if (!resume_ramp_active_)
  {
    return command;
  }
  const double dt = std::clamp(steady_time - resume_command_time_, 0.0, 0.05);
  const auto increase = [dt](const double previous, const double target, const double acceleration)
  {
    // 安全停车／降低限速立即生效；仅对恢复时幅值增加限加速度。
    if (previous * target < 0.0)
    {
      return 0.0;
    }
    return std::copysign(std::min(std::abs(target), std::abs(previous) + acceleration * dt), target);
  };
  auto result = command;
  result.linear.x = increase(resume_command_.linear.x, command.linear.x, resume_linear_acceleration_);
  result.angular.z = increase(resume_command_.angular.z, command.angular.z, max_accel_);
  resume_command_ = result;
  resume_command_time_ = steady_time;
  if (std::abs(result.linear.x - command.linear.x) < 1e-6 && std::abs(result.angular.z - command.angular.z) < 1e-6)
  {
    resume_ramp_active_ = false;
  }
  return result;
}

void AutonomousFixedPathController::logTerminalReview(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const double xy, const double yaw, const double xy_tolerance, const bool stationary, const double tf_stamp, const double tf_age, const double odom_age, const double receive_age)
{
  const double time = node_->now().seconds();
  const std::string phase = detail::fixedReviewPhaseName(terminal_review_.phase);
  if (phase == terminal_log_phase_ && time >= terminal_log_time_ && time - terminal_log_time_ < 1.0)
  {
    return;
  }
  const auto elapsed = [time](const std::optional<double> & since)
  {
    return since ? std::max(0.0, time - *since) : 0.0;
  };
  const double dx = pose.pose.position.x - path_.poses.back().pose.position.x;
  const double dy = pose.pose.position.y - path_.poses.back().pose.position.y;
  RCLCPP_INFO(node_->get_logger(), "Autonomous fixed terminal review: execution=%lld phase=%s xy=%.6fm tolerance=%.6fm dx=%.6fm dy=%.6fm yaw_error=%.6frad vx=%.6fm/s wz=%.6frad/s stopped=%s brake=%.3fs settle=%.3fs position_wait=%.3fs trim_attempts=%u trim_step=%.6fm trim_total=%.6fm pose_stamp=%.9f tf_stamp=%.9f tf_age=%.6fs odom_stamp=%.9f odom_age=%.6fs receive_age=%.6fs", static_cast<long long>(rclcpp::Time(path_.header.stamp).nanoseconds()), phase.c_str(), xy, xy_tolerance, dx, dy, yaw, measured.linear.x, measured.angular.z, stationary ? "true" : "false", elapsed(terminal_review_.brake_since), elapsed(terminal_review_.settled_since), elapsed(terminal_review_.position_since), trim_.attempts, trim_.step_distance, trim_.distance, rclcpp::Time(pose.header.stamp).seconds(), tf_stamp, tf_age, rclcpp::Time(odom_.header.stamp).seconds(), odom_age, receive_age);
  terminal_log_phase_ = phase;
  terminal_log_time_ = time;
}

bool AutonomousFixedPathController::trajectoryFree(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const geometry_msgs::msg::Twist & command)
{
  const auto reject = [&](const std::string & reason, const double x, const double y, const double yaw, const double cost, const int cell_x, const int cell_y, const int center_cost, const double predicted_time)
  {
    collision_reason_ = reason;
    const double time = node_->now().seconds();
    if (collision_log_time_ < 0.0 || time < collision_log_time_ || time - collision_log_time_ >= 1.0)
    {
      // 栅格是拒绝足迹的中心索引，不能把它等同于足迹边界的具体障碍点。
      RCLCPP_WARN(node_->get_logger(), "Autonomous fixed trajectory blocked: execution=%lld reason=%s frame=%s x=%.6f y=%.6f yaw=%.6f predicted_time=%.3f footprint_cost=%.1f center_cell=(%d,%d) center_cost=%d map_current=%s measured=(%.6f,%.6f) command=(%.6f,%.6f)", static_cast<long long>(rclcpp::Time(path_.header.stamp).nanoseconds()), reason.c_str(), pose.header.frame_id.c_str(), x, y, yaw, predicted_time, cost, cell_x, cell_y, center_cost, costmap_->isCurrent() ? "true" : "false", measured.linear.x, measured.angular.z, command.linear.x, command.angular.z);
      collision_log_time_ = time;
    }
    return false;
  };
  if (!costmap_->isCurrent())
  {
    return reject("COSTMAP_STALE", pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation), -1.0, -1, -1, -1, 0.0);
  }
  auto * map = costmap_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
  const auto footprint = costmap_->getRobotFootprint();
  if (footprint.size() < 3)
  {
    throw nav2_core::PlannerException("Autonomous fixed footprint unavailable");
  }
  double path_x = 0.0;
  double path_y = 0.0;
  double path_yaw = 0.0;
  const bool trim_motion = trim_.active || (trim_.started && std::abs(measured.linear.x) > 0.001);
  if (trim_motion && pose.header.frame_id != path_.header.frame_id)
  {
    const auto transform = tf_->lookupTransform(path_.header.frame_id, pose.header.frame_id, tf2::TimePointZero);
    path_x = transform.transform.translation.x;
    path_y = transform.transform.translation.y;
    path_yaw = tf2::getYaw(transform.transform.rotation);
  }
  std::size_t samples = 0;
  const auto free = [&](const double x, const double y, const double yaw)
  {
    const double predicted_time = static_cast<double>(samples++) * std::min(period_, 0.02);
    unsigned int mx = 0;
    unsigned int my = 0;
    const bool inside = map->worldToMap(x, y, mx, my);
    const int cell_x = inside ? static_cast<int>(mx) : -1;
    const int cell_y = inside ? static_cast<int>(my) : -1;
    const int center_cost = inside ? static_cast<int>(map->getCost(mx, my)) : -1;
    if (trim_motion && !trimPoseAllowed(path_x + x * std::cos(path_yaw) - y * std::sin(path_yaw), path_y + x * std::sin(path_yaw) + y * std::cos(path_yaw)))
    {
      return reject("TRIM_CORRIDOR", x, y, yaw, -1.0, cell_x, cell_y, center_cost, predicted_time);
    }
    const double cost = collision_->footprintCostAtPose(x, y, yaw, footprint);
    if (cost >= 0.0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      return true;
    }
    const double reaction = 0.1 + std::max(0.25 / 8.0, max_accel_ / 6.0);
    const std::string reason = predicted_time == 0.0 ? "CURRENT_FOOTPRINT" : predicted_time < reaction ? "REACTION_FOOTPRINT" : predicted_time < 0.5 + reaction ? "COMMAND_FOOTPRINT" : "BRAKING_FOOTPRINT";
    return reject(reason, x, y, yaw, cost, cell_x, cell_y, center_cost, predicted_time);
  };
  const bool safe = detail::autonomousTerminalTrajectory(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation), measured.linear.x, measured.angular.z, command.linear.x, command.angular.z, max_accel_, period_, free);
  if (safe)
  {
    collision_reason_.clear();
  }
  return safe;
}

bool AutonomousFixedPathController::trimPoseAllowed(const double x, const double y) const
{
  if (rotation_only_ || path_.poses.size() < 2)
  {
    return false;
  }
  const auto & goal = path_.poses.back().pose.position;
  double tail = 0.0;
  double nearest = 1e9;
  bool end_plane = false;
  for (std::size_t i = path_.poses.size() - 1; i > 0 && tail < 0.2; --i)
  {
    const auto & a = path_.poses[i - 1].pose.position;
    const auto & b = path_.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6)
    {
      continue;
    }
    if (!end_plane)
    {
      // 不越过本段终点平面；微调也不能提前进入下一换向段。
      if ((x - goal.x) * dx / length + (y - goal.y) * dy / length > 0.0001)
      {
        return false;
      }
      end_plane = true;
    }
    const double first = std::max(0.0, 1.0 - (0.2 - tail) / length);
    const double projection = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / (length * length), first, 1.0);
    nearest = std::min(nearest, std::hypot(x - a.x - projection * dx, y - a.y - projection * dy));
    tail += length;
  }
  return end_plane && nearest <= std::min(0.02, terminal_capture_margin_);
}

geometry_msgs::msg::Twist AutonomousFixedPathController::trimCommand(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & measured, const double xy_tolerance)
{
  geometry_msgs::msg::Twist command;
  const auto & goal = path_.poses.back().pose.position;
  const double dx = goal.x - pose.pose.position.x;
  const double dy = goal.y - pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  // 在成功半径内留出停车余量；完成后必须重新执行 BRAKE 和完整精度验收。
  if (distance <= 0.5 * xy_tolerance + 0.2 * std::abs(measured.linear.x))
  {
    trim_.active = false;
    terminal_review_.reset();
    trim_confirm_stamp_.reset();
    return command;
  }
  const double bearing = std::atan2(dy, dx) + (trim_direction_ < 0 ? M_PI : 0.0);
  const double error = detail::fixedAngle(bearing - tf2::getYaw(pose.pose.orientation));
  command.angular.z = detail::autonomousHeadingCommand(error, measured.angular.z, max_angular_, max_accel_, period_, 0.04);
  command.linear.x = detail::fixedTrimLinear(trim_, trim_limits_, distance, xy_tolerance, error, measured.linear.x, measured.angular.z, trim_direction_, trim_speed_limit_, period_);
  terminal_review_.phase = std::abs(error) > 0.08 || std::abs(measured.angular.z) > 0.08 ? detail::FixedReviewPhase::TRIM_ALIGN : detail::FixedReviewPhase::POSITION_TRIM;
  return command;
}

geometry_msgs::msg::TwistStamped AutonomousFixedPathController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = false;
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_->getBaseFrameID();
  command.header.stamp = node_->now();
  if (path_.poses.empty() || !checker)
  {
    throw nav2_core::PlannerException("Autonomous fixed path or goal checker unavailable");
  }
  const double time = node_->now().seconds();
  const double odom_age = has_odom_ ? time - rclcpp::Time(odom_.header.stamp).seconds() : 1e9;
  const double receive_age = has_odom_ ? std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received_).count() : 1e9;
  const auto & measured = odom_.twist.twist;
  const bool fresh = odom_age >= -0.05 && odom_age <= 0.2 && receive_age <= 0.2 && std::isfinite(measured.linear.x) && std::isfinite(measured.linear.y) && std::isfinite(measured.angular.z);
  const double steady_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if (fresh && !detail::accountFixedTrim(trim_, steady_time, measured.linear.x))
  {
    status("FAILED");
    throw nav2_core::PlannerException("Autonomous fixed trim feedback gap or invalid budget timing");
  }
  if (trim_.started && (trim_.distance >= trim_limits_.max_distance || trim_.step_distance >= trim_limits_.max_step))
  {
    status("FAILED");
    throw nav2_core::PlannerException("Autonomous fixed trim displacement including brake coast exceeded budget");
  }
  double tf_stamp = 0.0;
  double tf_age = 1e9;
  bool transform_available = false;
  // 定位门控使用源时间戳，不能把查询缓存成功当作定位仍在更新。
  try
  {
    const auto transform = tf_->lookupTransform(path_.header.frame_id, costmap_->getBaseFrameID(), tf2::TimePointZero);
    tf_stamp = rclcpp::Time(transform.header.stamp).seconds();
    tf_age = time - tf_stamp;
    diagnostic_tf_stamp_ = std::max<int64_t>(0, rclcpp::Time(transform.header.stamp).nanoseconds());
    transform_available = tf_stamp > 0.0;
  }
  catch (const tf2::TransformException &)
  {
    diagnostic_tf_stamp_ = 0;
  }
  diagnostic_tf_age_ = tf_age;
  diagnostic_odom_age_ = odom_age;
  diagnostic_receive_age_ = receive_age;
  localization_reason_ = "NONE";
  if (!fresh)
  {
    localization_reason_ = !has_odom_ ? "ODOM_MISSING" : !std::isfinite(measured.linear.x) || !std::isfinite(measured.linear.y) || !std::isfinite(measured.angular.z) ? "ODOM_INVALID" : odom_age < -0.05 ? "ODOM_FUTURE" : odom_age > 0.2 ? "ODOM_STAMP_STALE" : "ODOM_RECEIVE_STALE";
  }
  else if (!transform_available)
  {
    localization_reason_ = "TF_UNAVAILABLE";
  }
  geometry_msgs::msg::PoseStamped robot;
  if (localization_reason_ != "NONE" || pose.header.frame_id == path_.header.frame_id)
  {
    robot = pose;
  }
  else
  {
    tf_->transform(pose, robot, path_.header.frame_id, tf2::durationFromSec(0.0));
  }
  const double goal_distance = detail::fixedDistance(robot, path_.poses.back());
  const double allowed_tf_age = detail::fixedLocalizationTimeout(last_localized_pose_.has_value(), terminal_, goal_distance, localization_timeout_, tracking_localization_timeout_, terminal_localization_distance_, base_linear_speed_, terminal_capture_margin_);
  if (localization_reason_ == "NONE" && !detail::freshAutonomousTransform(tf_age, allowed_tf_age))
  {
    localization_reason_ = tf_age < -0.05 ? "TF_FUTURE" : "TF_STALE";
  }
  const bool localized = localization_reason_ == "NONE";
  if (localized && tf_age > localization_timeout_)
  {
    localization_reason_ = "TF_DELAY_TOLERATED";
  }
  if (pauseForLocalization(localized, robot, measured, tf_stamp, steady_time))
  {
    resume_ramp_active_ = false;
    return command;
  }
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist stopped;
  if (!checker->getTolerances(tolerance, stopped))
  {
    throw nav2_core::PlannerException("Autonomous fixed goal tolerances unavailable");
  }
  const double xy = detail::fixedDistance(robot, path_.poses.back());
  const double yaw = detail::fixedAngle(tf2::getYaw(path_.poses.back().pose.orientation) - tf2::getYaw(robot.pose.orientation));
  const double yaw_tolerance = tf2::getYaw(tolerance.orientation);
  const bool stationary = std::hypot(measured.linear.x, measured.linear.y) <= stopped.linear.x && std::abs(measured.angular.z) <= stopped.angular.z;
  detail::FixedSegment segments;
  segments.last = path_.poses.size() - 1;
  segments.rotation = rotation_only_;
  cursor_ = detail::advanceFixedCursor(path_, segments, cursor_, robot, std::max(0.10, std::abs(measured.linear.x) * period_ * 3.0 + 0.05));
  if (!terminal_ && tracker_loaded_)
  {
    command = tracker_.computeVelocityCommands(pose, velocity, checker);
    terminal_ = tracker_.isTerminalStopLatched();
  }
  if (terminal_)
  {
    command.twist = geometry_msgs::msg::Twist();
    if (trim_.active)
    {
      if (xy > tolerance.position.x + terminal_capture_margin_ || !trimPoseAllowed(robot.pose.position.x, robot.pose.position.y) || !detail::fixedTrimWithinBudget(trim_, trim_limits_, steady_time))
      {
        status("FAILED");
        throw nav2_core::PlannerException("Autonomous fixed trim capture, corridor, distance or time budget exceeded");
      }
      if (trim_paused_ && !blocked_ && detail::autonomousSettled(stationary, steady_time, brake_stable_duration_, trim_resume_since_))
      {
        trim_paused_ = false;
      }
      if (!blocked_ && !trim_paused_)
      {
        command.twist = trimCommand(robot, measured, tolerance.position.x);
      }
    }
    else
    {
      detail::reviewFixedTerminal(terminal_review_, xy, yaw, stationary, time, tolerance.position.x, yaw_tolerance, brake_stable_duration_, settle_duration_, terminal_review_timeout_, terminal_capture_margin_);
      if (terminal_review_.phase == detail::FixedReviewPhase::WAIT_POSITION && !rotation_only_ && !blocked_)
      {
        if (!trim_confirm_stamp_)
        {
          trim_confirm_stamp_ = tf_stamp;
        }
        // 至少两个不同源时间戳，且持续超差 0.2 s，才允许新微调；短暂回圈仍按原规则验收。
        if (terminal_review_.position_since && time - *terminal_review_.position_since >= trim_confirm_duration_ && tf_stamp > *trim_confirm_stamp_)
        {
          if (!trimPoseAllowed(robot.pose.position.x, robot.pose.position.y) || !detail::beginFixedTrim(trim_, trim_limits_, steady_time))
          {
            status("FAILED");
            throw nav2_core::PlannerException("Autonomous fixed trim cannot start within original segment and budgets");
          }
          terminal_review_.reset();
          trim_paused_ = false;
          trim_resume_since_.reset();
          command.twist = trimCommand(robot, measured, tolerance.position.x);
        }
      }
      else
      {
        trim_confirm_stamp_.reset();
      }
    }
    const auto phase = terminal_review_.phase;
    logTerminalReview(robot, measured, xy, yaw, tolerance.position.x, stationary, tf_stamp, tf_age, odom_age, receive_age);
    if (terminal_review_.failed())
    {
      status("FAILED");
      throw nav2_core::PlannerException(std::string("Autonomous fixed terminal review failed: ") + detail::fixedReviewPhaseName(phase));
    }
    if (phase == detail::FixedReviewPhase::ALIGN_GOAL)
    {
      command.twist.angular.z = detail::autonomousHeadingCommand(yaw, measured.angular.z, max_angular_, max_accel_, period_, yaw_tolerance * 0.5);
    }
    ready_ = phase == detail::FixedReviewPhase::READY;
    status(detail::fixedReviewPhaseName(phase));
  }
  command.twist = limitResumeCommand(command.twist, steady_time);
  if (!trajectoryFree(pose, measured, command.twist))
  {
    ready_ = false;
    terminal_review_.reset();
    trim_paused_ = trim_.active;
    trim_confirm_stamp_.reset();
    trim_resume_since_.reset();
    clear_.reset();
    if (!blocked_)
    {
      blocked_ = time;
    }
    if (time - *blocked_ > obstacle_timeout_)
    {
      status("FAILED");
      throw nav2_core::PlannerException("Autonomous fixed obstacle wait expired");
    }
    command.twist = geometry_msgs::msg::Twist();
    status("WAIT_OBSTACLE");
  }
  else if (blocked_)
  {
    ready_ = false;
    terminal_review_.reset();
    trim_paused_ = trim_.active;
    trim_confirm_stamp_.reset();
    trim_resume_since_.reset();
    command.twist = geometry_msgs::msg::Twist();
    if (detail::autonomousSettled(true, time, 0.5, clear_))
    {
      blocked_.reset();
      clear_.reset();
    }
    status("WAIT_OBSTACLE");
  }
  else if (!terminal_)
  {
    status("TRACKING");
  }
  if (resume_ramp_active_)
  {
    // 障碍等待产生的零输出也进入恢复历史，不能积累未实际下发的加速量。
    resume_command_ = command.twist;
  }
  return command;
}
}
PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::AutonomousFixedPathController, nav2_core::Controller)
