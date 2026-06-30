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

#ifndef NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace nav2_smoother
{

/**
 * @class nav2_smoother::SimpleSmoother
 * @brief A path smoother implementation
  * 中文：路径平滑器实现。
 */
class SimpleSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief A constructor for nav2_smoother::SimpleSmoother
   * 中文：nav2_smoother::SimpleSmoother 的构造函数。
   */
  SimpleSmoother() = default;

  /**
   * @brief A destructor for nav2_smoother::SimpleSmoother
   * 中文：nav2_smoother::SimpleSmoother 的析构函数。
   */
  ~SimpleSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  /**
   * @brief Method to cleanup resources.
   * 中文：清理资源的方法。
   */
  void cleanup() override {costmap_sub_.reset();}

  /**
   * @brief Method to activate smoother and any threads involved in execution.
   * 中文：激活 smoother 以及执行相关线程的方法。
   */
  void activate() override {}

  /**
   * @brief Method to deactivate smoother and any threads involved in execution.
   * 中文：停用 smoother 以及执行相关线程的方法。
   */
  void deactivate() override {}

  /**
   * @brief Method to smooth given path
   * 中文：平滑给定路径的方法。
   *
   * @param path In-out path to be smoothed
   * 中文：输入输出参数：需要平滑的路径。
   * @param max_time Maximum duration smoothing should take
   * 中文：路径平滑允许使用的最长时间。
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   * 中文：平滑完成返回 true，因时间限制中断返回 false。
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  /**
   * @brief Smoother method - does the smoothing on a segment
   * 中文：平滑器方法：对路径片段执行平滑。
   * @param path Reference to path
   * 中文：路径引用。
   * @param reversing_segment Return if this is a reversing segment
   * 中文：返回该片段是否为倒车片段。
   * @param costmap Pointer to minimal costmap
   * 中文：最小 costmap 指针。
   * @param max_time Maximum time to compute, stop early if over limit
   * 中文：最大计算时间，超限则提前停止。
   * @return If smoothing was successful
   * 中文：平滑是否成功。
   */
  bool smoothImpl(
    nav_msgs::msg::Path & path,
    bool & reversing_segment,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time);

  /**
   * @brief Get the field value for a given dimension
   * 中文：获取指定维度的字段值。
   * @param msg Current pose to sample
   * 中文：要采样的当前位姿。
   * @param dim Dimension ID of interest
   * 中文：关注的维度 ID。
   * @return dim value
   * 中文：维度值。
   */
  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * 中文：设置指定维度的字段值。
   * @param msg Current pose to sample
   * 中文：要采样的当前位姿。
   * @param dim Dimension ID of interest
   * 中文：关注的维度 ID。
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  double tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_;
  bool do_refinement_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleSmoother")};
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
