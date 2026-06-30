// Copyright (c) 2020 Fetullah Atas
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
// limitations under the License.

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_

/**
 * While C++17 isn't the project standard. We have to force LLVM/CLang
  * 中文：虽然 C++17 不是本项目标准，但这里必须强制 LLVM/Clang
 * to ignore deprecated declarations
  * 中文：忽略 deprecated 声明。
 */
#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM


#include <filesystem>
#include <mutex>
#include <string>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"


namespace nav2_waypoint_follower
{

class PhotoAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  /**
  * @brief Construct a new Photo At Waypoint object
   * 中文：构造新的 PhotoAtWaypoint 对象。
  *
  */
  PhotoAtWaypoint();

  /**
   * @brief Destroy the Photo At Waypoint object
   * 中文：销毁 PhotoAtWaypoint 对象。
   *
   */
  ~PhotoAtWaypoint();

  /**
   * @brief declares and loads parameters used
   * 中文：声明并加载使用到的参数。
   *
   * @param parent parent node that plugin will be created within
   * 中文：插件将在其中创建的父节点。
   * @param plugin_name should be provided in nav2_params.yaml==> waypoint_follower
   * 中文：应在 nav2_params.yaml 的 waypoint_follower 中提供。
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   * 中文：重写该函数以定义机器人到达路点后要执行的任务主体。
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * 中文：任务执行成功时返回 true。
   * @return if task execution failed
   * 中文：任务执行失败时返回 false。
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

  /**
   * @brief
   *
   * @param msg
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief given a shared pointer to sensor::msg::Image type, make a deep copy to inputted cv Mat
   * 中文：给定 sensor::msg::Image 类型共享指针，将图像深拷贝到输入的 cv::Mat。
   *
   * @param msg
   * @param mat
   */
  static void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & mat);

protected:
  // to ensure safety when accessing global var curr_frame_
  // 中文：确保访问全局变量 curr_frame_ 时的安全性。
  std::mutex global_mutex_;
  // the taken photos will be saved under this directory
  // 中文：拍摄的照片会保存在该目录下。
  std::filesystem::path save_dir_;
  // .png ? .jpg ? or some other well known format
  std::string image_format_;
  // the topic to subscribe in order capture a frame
  // 中文：用于捕获一帧图像而订阅的 topic。
  std::string image_topic_;
  // whether plugin is enabled
  // 中文：插件是否启用。
  bool is_enabled_;
  // current frame;
  // 中文：当前图像帧。
  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_;
  // global logger
  // 中文：全局 logger。
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  // ros subscriber to get camera image
  // 中文：用于获取相机图像的 ROS 订阅器。
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscriber_;
};
}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_
