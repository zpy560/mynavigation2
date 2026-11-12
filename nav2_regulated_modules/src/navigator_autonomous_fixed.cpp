#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "nav2_regulated_modules/navigation_utils.hpp"
#include "nav2_regulated_modules/detail/autonomous_navigation.hpp"
#include <regex>

namespace nav2_regulated_modules
{
bool RegulatedNavigator::fixedVelocityStopped()
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  if (!has_velocity_odometry_)
  {
    return false;
  }
  const double age = (now() - rclcpp::Time(latest_velocity_odometry_.header.stamp)).seconds();
  const auto & velocity = latest_velocity_odometry_.twist.twist;
  return age >= -0.05 && age <= 0.2 && std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) && std::isfinite(velocity.angular.z) && std::hypot(velocity.linear.x, velocity.linear.y) <= 0.01 && std::abs(velocity.angular.z) <= 0.05;
}

void RegulatedNavigator::beginFixedAutonomy()
{
  frozen_phase_ = "WAIT_START";
  frozen_wait_since_ = now().seconds();
  frozen_stable_since_.reset();
  frozen_localization_wait_ = false;
  frozen_soft_pause_since_.reset();
  frozen_localization_reason_.clear();
  task_.requested_speed = get_parameter("autonomous_fixed_max_speed").as_double();
  stopRobot();
  LOG_INFO("自主固定路径任务等待定位、旧执行终态与连续停稳，generation={}", task_.generation);
}

void RegulatedNavigator::freezeAutonomousPath(const nav_msgs::msg::Path & path)
{
  try
  {
    if (!frozen_path_.poses.empty() || path.header.frame_id != global_frame_)
    {
      throw std::invalid_argument("Frozen path already exists or has an unexpected frame");
    }
    const auto segments = detail::splitFixedPath(path);
    const double endpoint_budget = std::min(0.003, 0.3 * get_parameter("autonomous_fixed_xy_tolerance").as_double());
    if (detail::fixedDistance(path.poses.back(), task_.goal) > endpoint_budget || std::abs(detail::fixedAngle(tf2::getYaw(path.poses.back().pose.orientation) - tf2::getYaw(task_.goal.pose.orientation))) > get_parameter("autonomous_fixed_yaw_tolerance").as_double())
    {
      throw std::invalid_argument("Hybrid path endpoint exceeds the original goal error budget");
    }
    frozen_path_ = path;
    frozen_segments_ = segments;
    frozen_fingerprint_ = detail::fixedPathFingerprint(path);
    frozen_segment_ = 0;
    frozen_cursor_ = segments.front().first;
    task_.total_path_length = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
      task_.total_path_length += detail::fixedDistance(path.poses[i - 1], path.poses[i]);
    }
    frozen_path_pub_->publish(frozen_path_);
    LOG_INFO("自主全局路径已冻结，generation={}，规划请求=1，fingerprint={}，点数={}，分段={}", task_.generation, frozen_fingerprint_, path.poses.size(), segments.size());
    frozen_phase_ = "WAIT_RESUME";
    frozen_wait_since_ = now().seconds();
    frozen_stable_since_.reset();
    task_.state = NavigationState::CONTROLLING;
  }
  catch (const std::exception & error)
  {
    failTask(error.what());
  }
}

void RegulatedNavigator::followFrozenSegment(const bool resume)
{
  if (frozen_segments_.empty() || frozen_segment_ >= frozen_segments_.size() || detail::fixedPathFingerprint(frozen_path_) != frozen_fingerprint_)
  {
    failTask("冻结路径或执行段无效");
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  if (!lookupCurrentPose(pose) || !fixedVelocityStopped() || frozen_outstanding_follows_ != 0)
  {
    return;
  }
  const auto & segment = frozen_segments_[frozen_segment_];
  const auto cursor = detail::advanceFixedCursor(frozen_path_, segment, frozen_cursor_, pose);
  if (detail::fixedDistance(pose, frozen_path_.poses[cursor]) > get_parameter("autonomous_fixed_resume_distance").as_double())
  {
    failTask("车辆偏离原固定路径，禁止生成恢复连接线");
    return;
  }
  frozen_cursor_ = cursor;
  // 保留游标前一个原始采样点，使极短剩余段仍有真实切线，不添加新坐标。
  frozen_dispatch_first_ = resume && cursor > segment.first ? cursor - 1 : segment.first;
  if (!segment.rotation && frozen_dispatch_first_ >= segment.last)
  {
    frozen_dispatch_first_ = segment.last - 1;
  }
  nav_msgs::msg::Path subpath;
  subpath.header = frozen_path_.header;
  frozen_execution_id_ = std::max(now().nanoseconds(), frozen_last_dispatch_id_ + 1);
  frozen_last_dispatch_id_ = frozen_execution_id_;
  subpath.header.stamp = rclcpp::Time(frozen_execution_id_, get_clock()->get_clock_type());
  subpath.poses.assign(frozen_path_.poses.begin() + frozen_dispatch_first_, frozen_path_.poses.begin() + segment.last + 1);
  frozen_phase_ = "DISPATCHING";
  frozen_soft_pause_since_.reset();
  frozen_localization_reason_.clear();
  frozen_follow_inflight_ = true;
  frozen_status_since_ = now().seconds();
  task_.last_progress_time = now();
  task_.last_progress_pose = pose;
  sendFollowPath(subpath);
}

void RegulatedNavigator::finishFrozenSegment()
{
  geometry_msgs::msg::PoseStamped pose;
  const auto & segment = frozen_segments_.at(frozen_segment_);
  const bool final = frozen_segment_ + 1 == frozen_segments_.size();
  const auto & target = final ? task_.goal : frozen_path_.poses[segment.last];
  const double xy = final ? get_parameter("autonomous_fixed_xy_tolerance").as_double() : 0.01;
  const double yaw = final ? get_parameter("autonomous_fixed_yaw_tolerance").as_double() : 0.08726646259971647;
  if (!lookupCurrentPose(pose) || !fixedVelocityStopped() || detail::fixedDistance(pose, target) > xy || std::abs(detail::fixedAngle(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(target.pose.orientation))) > yaw)
  {
    failTask("固定子路径返回成功，但目标位姿或新鲜停稳复核未通过");
    return;
  }
  frozen_follow_inflight_ = false;
  stopRobot();
  if (final)
  {
    LOG_INFO("自主冻结路径严格验收通过，generation={}，规划请求=1，fingerprint={}", task_.generation, frozen_fingerprint_);
    succeedTask();
    return;
  }
  ++frozen_segment_;
  frozen_cursor_ = frozen_segments_[frozen_segment_].first;
  frozen_phase_ = "WAIT_RESUME";
  frozen_wait_since_ = now().seconds();
  frozen_stable_since_.reset();
}

void RegulatedNavigator::onFixedExecutionStatus(const std_msgs::msg::String::SharedPtr message)
{
  if (task_.type == TaskType::NONE || !frozen_follow_inflight_)
  {
    return;
  }
  // 状态协议仅解析固定的整数／枚举字段；不将字符串内容执行为配置或命令。
  static const std::regex format(R"status(\{"execution_id":([0-9]+),"point_index":([0-9]+),"phase":"([A-Z_]+)"(?:,"reason":"([A-Z_]+)","tf_stamp_ns":[0-9]+,"tf_age":-?[0-9]+\.[0-9]+,"odom_age":-?[0-9]+\.[0-9]+,"receive_age":-?[0-9]+\.[0-9]+,"pause_duration":[0-9]+\.[0-9]+)?\})status");
  std::smatch match;
  if (!std::regex_match(message->data, match, format))
  {
    return;
  }
  try
  {
    if (std::stoll(match[1].str()) != frozen_execution_id_)
    {
      return;
    }
    const std::size_t index = frozen_dispatch_first_ + static_cast<std::size_t>(std::stoull(match[2].str()));
    if (index > frozen_segments_.at(frozen_segment_).last)
    {
      return;
    }
    frozen_cursor_ = std::max(frozen_cursor_, index);
    frozen_phase_ = match[3].str();
    frozen_status_since_ = now().seconds();
    const std::string reason = match[4].matched ? match[4].str() : "UNSPECIFIED";
    if ((frozen_phase_ == "PAUSED_LOCALIZATION" || frozen_phase_ == "WAIT_LOCALIZATION") && reason != frozen_localization_reason_)
    {
      LOG_INFO("自主冻结路径定位门控：execution={}，phase={}，reason={}", frozen_execution_id_, frozen_phase_, reason);
    }
    frozen_localization_reason_ = reason;
    if (frozen_phase_ == "FAILED")
    {
      failTask("固定路径控制包装报告不可恢复故障");
    }
  }
  catch (const std::exception &)
  {
    // 非法或迟到状态不改变任务。
  }
}

void RegulatedNavigator::monitorFixedAutonomy()
{
  const double time = now().seconds();
  if (planning_active_ && (now() - task_.last_replan_time).seconds() > server_timeout_)
  {
    failTask("唯一一次全局规划请求超时");
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  const bool localized = lookupCurrentPose(pose);
  const bool controller_lost = frozen_phase_ == "WAIT_LOCALIZATION";
  const bool controller_paused = frozen_phase_ == "PAUSED_LOCALIZATION";
  if (frozen_follow_inflight_ && (time < frozen_status_since_ || time - frozen_status_since_ > 2.0))
  {
    failTask("固定路径执行状态超时");
    return;
  }
  // 行走中的暂停只认当前执行状态，导航器的另一份 TF 缓存不再重复门控速度。
  if (controller_paused && frozen_follow_inflight_ && !frozen_localization_wait_)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!frozen_soft_pause_since_)
    {
      frozen_soft_pause_since_ = steady_now;
      LOG_INFO("自主冻结路径短时暂停，保留 FollowPath，execution={}，navigator_tf_valid={}", frozen_execution_id_, localized);
    }
    const double paused = std::chrono::duration<double>(steady_now - *frozen_soft_pause_since_).count();
    if (paused > localization_recovery_timeout_)
    {
      failTask("原 Action 定位或速度反馈暂停超过恢复上限");
    }
    frozen_inhibited_since_.reset();
    task_.last_progress_time = now();
    return;
  }
  if (controller_lost)
  {
    frozen_soft_pause_since_.reset();
    if (!frozen_localization_wait_)
    {
      frozen_wait_since_ = time;
      frozen_localization_wait_ = true;
      if (frozen_follow_inflight_)
      {
        ++follow_sequence_;
        if (active_follow_goal_)
        {
          follow_client_->async_cancel_goal(active_follow_goal_);
          active_follow_goal_.reset();
        }
        frozen_follow_inflight_ = false;
      }
      frozen_phase_ = "WAIT_RESUME";
    }
    frozen_stable_since_.reset();
    stopRobot();
    if (time - frozen_wait_since_ > localization_recovery_timeout_)
    {
      failTask("定位或速度反馈恢复超时；保留原路径且不重规划");
    }
    return;
  }
  if (frozen_soft_pause_since_)
  {
    LOG_INFO("自主冻结路径短时暂停恢复，沿原 Action 继续，execution={}", frozen_execution_id_);
    frozen_soft_pause_since_.reset();
    task_.last_progress_time = now();
  }
  if (frozen_phase_ == "WAIT_START" || frozen_phase_ == "WAIT_RESUME")
  {
    stopRobot();
    if (time - frozen_wait_since_ > localization_recovery_timeout_)
    {
      failTask("等待新鲜定位、停稳或旧执行结束超时");
      return;
    }
    if (!detail::autonomousSettled(localized && fixedVelocityStopped() && frozen_outstanding_follows_ == 0, time, localization_stable_duration_, frozen_stable_since_))
    {
      return;
    }
    frozen_localization_wait_ = false;
    if (!frozen_plan_requested_)
    {
      try
      {
        if (task_.goal.header.frame_id != global_frame_)
        {
          task_.goal = tf_buffer_->transform(task_.goal, global_frame_, tf2::durationFromSec(0.0));
        }
      }
      catch (const tf2::TransformException & error)
      {
        failTask(error.what());
        return;
      }
      frozen_phase_ = "PLANNING";
      startPlanning(false);
    }
    else if (!planning_active_ && !frozen_path_.poses.empty())
    {
      followFrozenSegment(true);
    }
    return;
  }
  if (planning_active_ || !frozen_follow_inflight_)
  {
    return;
  }
  bool output_inhibited = false;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    // 当前 Humble Collision Monitor 不发布状态消息；观察最终零输出与仍有跟踪需求的差异。
    output_inhibited = has_controller_velocity_ && has_smoothed_velocity_ && (std::abs(latest_controller_velocity_.linear.x) > 0.02 || std::abs(latest_controller_velocity_.angular.z) > 0.05) && std::abs(latest_smoothed_velocity_.linear.x) < 1e-6 && std::abs(latest_smoothed_velocity_.angular.z) < 1e-6;
  }
  output_inhibited = output_inhibited && fixedVelocityStopped();
  if (!output_inhibited)
  {
    frozen_inhibited_since_.reset();
  }
  else if (!frozen_inhibited_since_)
  {
    frozen_inhibited_since_ = time;
  }
  const bool output_wait = frozen_inhibited_since_ && time - *frozen_inhibited_since_ >= 0.5;
  const bool obstacle = (collision_state_enabled_ && collision_state_.snapshot().stopped()) || frozen_phase_ == "WAIT_OBSTACLE" || output_wait;
  if (obstacle)
  {
    if (frozen_obstacle_since_ < 0.0)
    {
      frozen_obstacle_since_ = time;
      LOG_INFO("自主冻结路径停车等待：phase={}，下游输出抑制={}，路径标识={}", frozen_phase_, output_wait, frozen_fingerprint_);
    }
    task_.last_progress_time = now();
    if (time - frozen_obstacle_since_ > get_parameter("autonomous_fixed_obstacle_timeout").as_double())
    {
      failTask("原路径障碍持续阻挡超过等待上限");
      return;
    }
  }
  else
  {
    frozen_obstacle_since_ = -1.0;
    if (!localized)
    {
      // 沿途宽限期间只暂缓导航器几何进度；包装仍检查反馈、TF 上限和碰撞。
      task_.last_progress_time = now();
      return;
    }
    const double moved = detail::fixedDistance(pose, task_.last_progress_pose);
    const double rotated = std::abs(detail::fixedAngle(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(task_.last_progress_pose.pose.orientation)));
    if (moved >= 0.01 || rotated >= 0.05)
    {
      task_.last_progress_pose = pose;
      task_.last_progress_time = now();
    }
    else if ((now() - task_.last_progress_time).seconds() > control_module_.progressTimeout())
    {
      failTask("原路径跟踪无进展；禁止清图重规划");
      return;
    }
  }
  task_.distance_remaining = 0.0;
  for (std::size_t i = frozen_cursor_ + 1; i < frozen_path_.poses.size(); ++i)
  {
    task_.distance_remaining += detail::fixedDistance(frozen_path_.poses[i - 1], frozen_path_.poses[i]);
  }
  if (localized)
  {
    task_.distance_remaining = std::max(task_.distance_remaining, detail::fixedDistance(pose, task_.goal));
  }
}
}
