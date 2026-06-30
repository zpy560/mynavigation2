// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <vector>
#include <memory>
#include "nav2_smoother/savitzky_golay_smoother.hpp"

namespace nav2_smoother
{

using namespace smoother_utils;  // NOLINT
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;

void SavitzkyGolaySmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name + ".refinement_num", rclcpp::ParameterValue(2));
  node->get_parameter(name + ".do_refinement", do_refinement_);
  node->get_parameter(name + ".refinement_num", refinement_num_);
}

bool SavitzkyGolaySmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration & max_time)
{
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time.seconds();

  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;

  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 9) {
      // Populate path segment
      // 中文：填充路径片段。
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      // 中文：确保剩余时间仍允许继续平滑。
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time.seconds() - duration_cast<duration<double>>(now - start).count();

      if (time_remaining <= 0.0) {
        RCLCPP_WARN(
          logger_,
          "Smoothing time exceeded allowed duration of %0.2f.", max_time.seconds());
        return false;
      }

      // Smooth path segment
      // 中文：平滑路径片段。
      success = success && smoothImpl(curr_path_segment, reversing_segment);

      // Assemble the path changes to the main path
      // 中文：将路径片段改动合并回主路径。
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool SavitzkyGolaySmoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  // Must be at least 10 in length to enter function
  // 中文：长度至少为 10 才能进入该函数。
  const unsigned int & path_size = path.poses.size();

  // 7-point SG filter
  const std::array<double, 7> filter = {
    -2.0 / 21.0,
    3.0 / 21.0,
    6.0 / 21.0,
    7.0 / 21.0,
    6.0 / 21.0,
    3.0 / 21.0,
    -2.0 / 21.0};

  auto applyFilter = [&](const std::vector<geometry_msgs::msg::Point> & data)
    -> geometry_msgs::msg::Point
    {
      geometry_msgs::msg::Point val;
      for (unsigned int i = 0; i != filter.size(); i++) {
        val.x += filter[i] * data[i].x;
        val.y += filter[i] * data[i].y;
      }
      return val;
    };

  auto applyFilterOverAxes =
    [&](std::vector<geometry_msgs::msg::PoseStamped> & plan_pts) -> void
    {
      // Handle initial boundary conditions, first point is fixed
      // 中文：处理初始边界条件，第一个点固定。
      unsigned int idx = 1;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 3].pose.position});

      // Apply nominal filter
      // 中文：应用标准滤波器。
      for (idx = 3; idx < path_size - 4; ++idx) {
        plan_pts[idx].pose.position = applyFilter(
        {
          plan_pts[idx - 3].pose.position,
          plan_pts[idx - 2].pose.position,
          plan_pts[idx - 1].pose.position,
          plan_pts[idx].pose.position,
          plan_pts[idx + 1].pose.position,
          plan_pts[idx + 2].pose.position,
          plan_pts[idx + 3].pose.position});
      }

      // Handle terminal boundary conditions, last point is fixed
      // 中文：处理末端边界条件，最后一个点固定。
      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 2].pose.position,
        plan_pts[idx + 2].pose.position});

      idx++;
      plan_pts[idx].pose.position = applyFilter(
      {
        plan_pts[idx - 3].pose.position,
        plan_pts[idx - 2].pose.position,
        plan_pts[idx - 1].pose.position,
        plan_pts[idx].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position,
        plan_pts[idx + 1].pose.position});
    };

  applyFilterOverAxes(path.poses);

  // Lets do additional refinement, it shouldn't take more than a couple milliseconds
  // 中文：执行额外细化，预计只需几毫秒。
  if (do_refinement_) {
    for (int i = 0; i < refinement_num_; i++) {
      applyFilterOverAxes(path.poses);
    }
  }

  updateApproximatePathOrientations(path, reversing_segment);
  return true;
}

}  // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smoother::SavitzkyGolaySmoother, nav2_core::Smoother)
