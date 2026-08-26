#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <optional>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_regulated_modules
{

std::optional<nav_msgs::msg::Path> RegulatedNavigator::prepareFixedPath(const std::vector<byd_custom_msgs::msg::NaviSegment> & segments) {
  if (segments.empty()) {
    LOG_ERROR("导航服务至少需要一个有效分段");
    return std::nullopt;
  }

  nav_msgs::msg::Path output;
  output.header.frame_id = global_frame_;
  output.header.stamp = now();
  constexpr double distance_tolerance = 1e-3;
  const auto valid_point = [](const geometry_msgs::msg::Point & point) {return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);};
  const auto make_pose = [&output](const geometry_msgs::msg::Point & point) {geometry_msgs::msg::PoseStamped pose; pose.header = output.header; pose.pose.position.x = point.x; pose.pose.position.y = point.y; pose.pose.position.z = 0.0; pose.pose.orientation.w = 1.0; return pose;};
  for (std::size_t index = 0; index < segments.size(); ++index) {
    const auto & input = segments[index];
    if (!valid_point(input.node1) || !valid_point(input.node2) || !valid_point(input.control_pos1) || !valid_point(input.control_pos2)) {
      LOG_ERROR("导航服务第 {} 段包含非有限坐标", index);
      return std::nullopt;
    }
    const double endpoint_delta_x = input.node2.x - input.node1.x;
    const double endpoint_delta_y = input.node2.y - input.node1.y;
    if (endpoint_delta_x * endpoint_delta_x + endpoint_delta_y * endpoint_delta_y <= distance_tolerance * distance_tolerance) {
      LOG_ERROR("导航服务第 {} 段起终点重合", index);
      return std::nullopt;
    }
    if (index > 0) {
      const double join_delta_x = input.node1.x - segments[index - 1].node2.x;
      const double join_delta_y = input.node1.y - segments[index - 1].node2.y;
      if (join_delta_x * join_delta_x + join_delta_y * join_delta_y > distance_tolerance * distance_tolerance) {
        LOG_ERROR("导航服务第 {} 段与前一段不连续", index);
        return std::nullopt;
      }
    }
    nav_msgs::msg::Path four_points;
    four_points.header = output.header;
    if (input.segment_type == byd_custom_msgs::msg::NaviSegment::SEGMENT_TYPR_LINE) {
      nav_msgs::msg::Path line;
      line.header = output.header;
      line.poses.push_back(make_pose(input.node1));
      line.poses.push_back(make_pose(input.node2));
      liner2to4point(line, four_points);
    } else if (input.segment_type == byd_custom_msgs::msg::NaviSegment::SEGMENT_TYPR_BEZIER) {
      four_points.poses.push_back(make_pose(input.node1));
      four_points.poses.push_back(make_pose(input.control_pos1));
      four_points.poses.push_back(make_pose(input.control_pos2));
      four_points.poses.push_back(make_pose(input.node2));
    } else {
      LOG_ERROR("导航服务第 {} 段类型 {} 不受支持", index, input.segment_type);
      return std::nullopt;
    }
    nav_msgs::msg::Path bezier_points;
    generateBezierUniformPoints(four_points, fixed_path_step_, bezier_points);
    if (bezier_points.poses.empty()) {
      LOG_ERROR("导航服务第 {} 段插值失败", index);
      return std::nullopt;
    }
    if (!output.poses.empty()) {output.poses.pop_back();}
    output.poses.insert(output.poses.end(), std::make_move_iterator(bezier_points.poses.begin()), std::make_move_iterator(bezier_points.poses.end()));
  }
  if (output.poses.size() < 2) {
    LOG_ERROR("导航服务生成的路径点不足两个");
    return std::nullopt;
  }
  for (const auto & pose : output.poses) {LOG_INFO("Path Point -> x: {}, y: {}", pose.pose.position.x, pose.pose.position.y);}
  return output;
}

rclcpp_action::GoalResponse RegulatedNavigator::handleNavigationServiceGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const NavigationService::Goal> goal) {
  if (operation_mode_ != NavigationMode::FIXED_PATH) {
    LOG_WARN("当前模式不接受 NavigationService Action Goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!active_ || goal->navi_segment.empty()) {
    LOG_WARN("拒绝 NavigationService Goal：节点未激活或分段数组为空");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RegulatedNavigator::handleNavigationServiceCancel(const std::shared_ptr<NavigationServiceHandle> goal) {
  if (goal == active_navigation_service_goal_) {
    cancel_requested_ = true;
    LOG_DEBUG("收到 NavigationService 取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RegulatedNavigator::handleNavigationServiceAccepted(const std::shared_ptr<NavigationServiceHandle> goal) {
  const auto prepared_path = prepareFixedPath(goal->get_goal()->navi_segment);
  if (!prepared_path) {
    auto result = std::make_shared<NavigationService::Result>();
    result->finish = false;
    goal->abort(result);
    LOG_ERROR("NavigationService Goal 已接受但分段路径生成失败");
    return;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_))) {
    auto result = std::make_shared<NavigationService::Result>();
    result->finish = false;
    goal->abort(result);
    LOG_ERROR("NavigationService Goal 已接受但 FollowPath Action Server 不可用");
    return;
  }

  preemptCurrentTask();
  active_navigation_service_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::NAVIGATION_SERVICE;
  task_.task_id = goal->get_goal()->task_id;
  task_.goal = prepared_path->poses.back();
  task_.active_path = *prepared_path;
  for (std::size_t index = 1; index < prepared_path->poses.size(); ++index) {const auto & previous = prepared_path->poses[index - 1].pose.position; const auto & current = prepared_path->poses[index].pose.position; task_.total_path_length += std::hypot(current.x - previous.x, current.y - previous.y);}
  task_.distance_remaining = task_.total_path_length;
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  task_.last_progress_pose = geometry_msgs::msg::PoseStamped();
  LOG_INFO("接受 NavigationService Action，generation={}，task_id={}，frame={}，路径点数={}，总长度={:.3f}m", task_.generation, task_.task_id, prepared_path->header.frame_id, prepared_path->poses.size(), task_.total_path_length);
  publishFixedPath(task_.active_path);
  publishSpeedLimit(goal);
  sendFollowPath(*prepared_path);
}

void RegulatedNavigator::publishFixedPath(const nav_msgs::msg::Path & path) {
  if (!fixed_path_pub_ || !fixed_path_pub_->is_activated() || !fixed_path_boundaries_pub_ || !fixed_path_boundaries_pub_->is_activated()) {
    LOG_WARN("固定路径可视化发布器未激活，跳过发布");
    return;
  }
  if (path.poses.empty()) {
    LOG_WARN("固定路径为空，跳过 RViz2 可视化发布");
    return;
  }
  auto visualization_path = path;
  visualization_path.header.stamp = now();
  for (auto & pose : visualization_path.poses) {pose.header = visualization_path.header;}
  fixed_path_pub_->publish(visualization_path);
  visualization_msgs::msg::MarkerArray boundaries;
  const auto make_line = [&visualization_path](const int id, const double lateral_offset, const double height, const double width, const float red, const float green, const float blue, const float alpha) {visualization_msgs::msg::Marker marker; marker.header = visualization_path.header; marker.ns = "fixed_path_tech_boundaries"; marker.id = id; marker.type = visualization_msgs::msg::Marker::LINE_STRIP; marker.action = visualization_msgs::msg::Marker::ADD; marker.pose.orientation.w = 1.0; marker.scale.x = width; marker.color.r = red; marker.color.g = green; marker.color.b = blue; marker.color.a = alpha; marker.frame_locked = true; marker.points.reserve(visualization_path.poses.size()); for (std::size_t index = 0; index < visualization_path.poses.size(); ++index) {const std::size_t previous_index = index == 0 ? 0 : index - 1; const std::size_t next_index = index + 1 < visualization_path.poses.size() ? index + 1 : index; const auto & previous = visualization_path.poses[previous_index].pose.position; const auto & next = visualization_path.poses[next_index].pose.position; const auto & current = visualization_path.poses[index].pose.position; const double tangent_x = next.x - previous.x; const double tangent_y = next.y - previous.y; const double tangent_length = std::hypot(tangent_x, tangent_y); geometry_msgs::msg::Point point; point.x = current.x; point.y = current.y; point.z = height; if (tangent_length > 1e-9) {point.x -= lateral_offset * tangent_y / tangent_length; point.y += lateral_offset * tangent_x / tangent_length;} marker.points.push_back(point);} return marker;};
  boundaries.markers.push_back(make_line(0, fixed_path_boundary_half_width_, 0.035, 0.14, 0.0F, 0.55F, 1.0F, 0.20F));
  boundaries.markers.push_back(make_line(1, -fixed_path_boundary_half_width_, 0.035, 0.14, 0.45F, 0.0F, 1.0F, 0.20F));
  boundaries.markers.push_back(make_line(2, fixed_path_boundary_half_width_, 0.055, 0.035, 0.0F, 0.95F, 1.0F, 1.0F));
  boundaries.markers.push_back(make_line(3, -fixed_path_boundary_half_width_, 0.055, 0.035, 0.75F, 0.20F, 1.0F, 1.0F));
  visualization_msgs::msg::Marker nodes;
  nodes.header = visualization_path.header;
  nodes.ns = "fixed_path_tech_nodes";
  nodes.id = 4;
  nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.pose.orientation.w = 1.0;
  nodes.scale.x = 0.07;
  nodes.scale.y = 0.07;
  nodes.scale.z = 0.07;
  nodes.color.r = 0.35F;
  nodes.color.g = 0.95F;
  nodes.color.b = 1.0F;
  nodes.color.a = 0.9F;
  nodes.frame_locked = true;
  for (std::size_t index = 0; index < visualization_path.poses.size(); index += 5) {const auto left_point = boundaries.markers[2].points[index]; const auto right_point = boundaries.markers[3].points[index]; nodes.points.push_back(left_point); nodes.points.push_back(right_point);}
  boundaries.markers.push_back(std::move(nodes));
  fixed_path_boundaries_pub_->publish(boundaries);
  LOG_INFO("固定路径与科技风边界已发布到 RViz2，path_topic={}，boundaries_topic={}，frame={}，路径点数={}，左右半宽={:.2f}m", fixed_path_visualization_topic_, fixed_path_boundaries_topic_, visualization_path.header.frame_id, visualization_path.poses.size(), fixed_path_boundary_half_width_);
}

void RegulatedNavigator::resumeCurrentTask() {
  if (task_.type == TaskType::NAVIGATION_SERVICE) {
    if (task_.active_path.poses.empty()) {
      failTask("没有可恢复的固定路径");
      return;
    }
    LOG_INFO("恢复固定路径任务，generation={}，路径点数={}", task_.generation, task_.active_path.poses.size());
    publishFixedPath(task_.active_path);
    sendFollowPath(task_.active_path);
    return;
  }
  LOG_INFO("恢复自主导航任务，generation={}，重新进入规划链", task_.generation);
  startPlanning(false);
}

void RegulatedNavigator::liner2to4point(const nav_msgs::msg::Path & input_path, nav_msgs::msg::Path & output_path) {
  const auto & start = input_path.poses[0].pose.position;
  const auto & end = input_path.poses[1].pose.position;
  output_path.header = input_path.header;
  output_path.poses.clear();
  output_path.poses.reserve(4);
  output_path.poses.push_back(input_path.poses[0]);
  geometry_msgs::msg::PoseStamped first_control;
  first_control.header = input_path.poses[0].header;
  first_control.pose.position.x = start.x + (end.x - start.x) / 3.0;
  first_control.pose.position.y = start.y + (end.y - start.y) / 3.0;
  first_control.pose.position.z = 0.0;
  first_control.pose.orientation.w = 1.0;
  output_path.poses.push_back(first_control);
  geometry_msgs::msg::PoseStamped second_control;
  second_control.header = input_path.poses[0].header;
  second_control.pose.position.x = start.x + 2.0 * (end.x - start.x) / 3.0;
  second_control.pose.position.y = start.y + 2.0 * (end.y - start.y) / 3.0;
  second_control.pose.position.z = 0.0;
  second_control.pose.orientation.w = 1.0;
  output_path.poses.push_back(second_control);
  output_path.poses.push_back(input_path.poses[1]);
}

geometry_msgs::msg::PoseStamped RegulatedNavigator::bezier3(const nav_msgs::msg::Path & input_path, const double t, const bool use_result) {
  const double inverse_t = 1.0 - t;
  const double basis0 = inverse_t * inverse_t * inverse_t;
  const double basis1 = 3.0 * inverse_t * inverse_t * t;
  const double basis2 = 3.0 * inverse_t * t * t;
  const double basis3 = t * t * t;
  const auto & point0 = input_path.poses[0].pose.position;
  const auto & point1 = input_path.poses[1].pose.position;
  const auto & point2 = input_path.poses[2].pose.position;
  const auto & point3 = input_path.poses[3].pose.position;
  geometry_msgs::msg::PoseStamped result;
  result.header = input_path.header;
  result.pose.position.x = basis0 * point0.x + basis1 * point1.x + basis2 * point2.x + basis3 * point3.x;
  result.pose.position.y = basis0 * point0.y + basis1 * point1.y + basis2 * point2.y + basis3 * point3.y;
  result.pose.position.z = 0.0;
  if (!use_result) {
    result.pose.orientation.w = 1.0;
    return result;
  }
  const double derivative_x = 3.0 * inverse_t * inverse_t * (point1.x - point0.x) + 6.0 * inverse_t * t * (point2.x - point1.x) + 3.0 * t * t * (point3.x - point2.x);
  const double derivative_y = 3.0 * inverse_t * inverse_t * (point1.y - point0.y) + 6.0 * inverse_t * t * (point2.y - point1.y) + 3.0 * t * t * (point3.y - point2.y);
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, std::atan2(derivative_y, derivative_x));
  orientation.normalize();
  result.pose.orientation = tf2::toMsg(orientation);
  return result;
}

void RegulatedNavigator::computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> & poses, std::vector<double> & arc_lengths) {
  arc_lengths.assign(poses.size(), 0.0);
  for (std::size_t index = 1; index < poses.size(); ++index) {
    const double delta_x = poses[index].pose.position.x - poses[index - 1].pose.position.x;
    const double delta_y = poses[index].pose.position.y - poses[index - 1].pose.position.y;
    arc_lengths[index] = arc_lengths[index - 1] + std::hypot(delta_x, delta_y);
  }
}

double RegulatedNavigator::findTfromArcLength(const std::vector<double> & arc_lengths, const std::vector<double> & ts, const double target_length) {
  if (target_length <= arc_lengths.front()) {return ts.front();}
  if (target_length >= arc_lengths.back()) {return ts.back();}
  const auto upper = std::upper_bound(arc_lengths.begin(), arc_lengths.end(), target_length);
  const std::size_t upper_index = static_cast<std::size_t>(std::distance(arc_lengths.begin(), upper));
  const std::size_t lower_index = upper_index - 1;
  const double lower_length = arc_lengths[lower_index];
  const double upper_length = arc_lengths[upper_index];
  if (upper_length <= lower_length) {return ts[upper_index];}
  return ts[lower_index] + (target_length - lower_length) / (upper_length - lower_length) * (ts[upper_index] - ts[lower_index]);
}

void RegulatedNavigator::generateBezierUniformPoints(const nav_msgs::msg::Path & input_path, const double interval, nav_msgs::msg::Path & output_path) {
  output_path = nav_msgs::msg::Path();
  if (input_path.poses.size() != 4 || !std::isfinite(interval) || interval <= 0.0) {
    LOG_ERROR("贝塞尔插值要求四个控制点和有限正采样间距，控制点数={}，间距={}", input_path.poses.size(), interval);
    return;
  }
  double control_polygon_length = 0.0;
  for (std::size_t index = 1; index < input_path.poses.size(); ++index) {
    const double delta_x = input_path.poses[index].pose.position.x - input_path.poses[index - 1].pose.position.x;
    const double delta_y = input_path.poses[index].pose.position.y - input_path.poses[index - 1].pose.position.y;
    control_polygon_length += std::hypot(delta_x, delta_y);
  }
  const int dense_count = std::max(10, static_cast<int>(std::ceil(control_polygon_length / (interval * 3.0))));
  std::vector<geometry_msgs::msg::PoseStamped> dense_poses;
  std::vector<double> parameters;
  dense_poses.reserve(static_cast<std::size_t>(dense_count));
  parameters.reserve(static_cast<std::size_t>(dense_count));
  for (int index = 0; index < dense_count; ++index) {
    const double parameter = static_cast<double>(index) / static_cast<double>(dense_count - 1);
    parameters.push_back(parameter);
    dense_poses.push_back(bezier3(input_path, parameter, false));
  }
  std::vector<double> arc_lengths;
  computeArcLengths(dense_poses, arc_lengths);
  const double total_length = arc_lengths.back();
  const int sample_count = static_cast<int>(std::floor(total_length / interval)) + 1;
  output_path.header = input_path.header;
  output_path.poses.reserve(static_cast<std::size_t>(sample_count + 1));
  for (int index = 0; index < sample_count; ++index) {
    const double target_length = std::min(static_cast<double>(index) * interval, total_length);
    output_path.poses.push_back(bezier3(input_path, findTfromArcLength(arc_lengths, parameters, target_length), true));
  }
  const auto & last_position = output_path.poses.back().pose.position;
  const auto & end_position = input_path.poses.back().pose.position;
  if (last_position.x != end_position.x || last_position.y != end_position.y) {output_path.poses.push_back(bezier3(input_path, 1.0, true));}
}

void RegulatedNavigator::publishSpeedLimit(const std::shared_ptr<NavigationServiceHandle> goal)
{
  auto msg = nav2_msgs::msg::SpeedLimit();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  // 不使用百分比模式（与订阅端逻辑对应：percentage 为 true 会报错）
  msg.percentage = false;
  auto line = goal->get_goal()->navi_segment[0];
  msg.speed_limit = -2*(line.motion_direction - 1.5)*line.max_speed;
  speed_limit_pub_->publish(msg);
}

}  // namespace nav2_regulated_modules
