/* Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_map_server/map_server.hpp"

#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"
#include "spdlog_wrapper.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("map_server", "", options), map_available_(false)
{
  RCLCPP_INFO(get_logger(), "Creating");
  LOG_INFO("Creating MapServer lifecycle node");

  // Declare the node parameters
  // 中文：声明地图文件、地图话题和地图坐标系参数，后续 configure 阶段读取。
  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "map");
  declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  LOG_INFO("Configuring MapServer lifecycle node");

  // MapServer bridges three map layers:
  // 1. YAML metadata layer: file path, resolution, origin, thresholds, mode.
  // 2. Image pixel layer: grayscale/alpha pixels from pgm/png/bmp files.
  // 3. OccupancyGrid layer: ROS message cached in msg_, served and published to Nav2.
  // 中文：MapServer 串联地图三层结构：YAML 元数据层 -> image 像素层 -> OccupancyGrid 消息层。
  // Get the name of the YAML file to use (can be empty if no initial map should be used)
  // 中文：读取 YAML 地图文件参数；为空时不预加载地图，只等待 load_map 服务动态加载。
  std::string yaml_filename = get_parameter("yaml_filename").as_string();
  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  LOG_INFO(
    "MapServer parameters yaml_filename='{}', topic_name='{}', frame_id='{}'",
    yaml_filename.c_str(), topic_name.c_str(), frame_id_.c_str());

  // only try to load map if parameter was set
  // 中文：配置阶段如果指定 yaml_filename，则立即走 YAML -> image -> OccupancyGrid 数据流。
  if (!yaml_filename.empty()) {
    // Shared pointer to LoadMap::Response is also should be initialized
    // in order to avoid null-pointer dereference
    // 中文：复用 LoadMap 响应结构承载加载结果，避免空指针并统一状态码。
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    LOG_INFO("MapServer loading initial map from '{}'", yaml_filename.c_str());
    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      LOG_INFO("MapServer failed to load initial map from '{}'", yaml_filename.c_str());
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }
    LOG_INFO(
      "MapServer initial map ready width={}, height={}, resolution={}",
      msg_.info.width, msg_.info.height, msg_.info.resolution);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "yaml-filename parameter is empty, set map through '%s'-service",
      load_map_service_name_.c_str());
  }

  // Make name prefix for services
  // 中文：服务名带节点名前缀，例如 map_server/map 和 map_server/load_map。
  const std::string service_prefix = get_name() + std::string("/");

  // Create a service that provides the occupancy grid
  // 中文：GetMap 服务直接返回当前缓存的 OccupancyGrid。
  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  // 中文：transient_local + reliable 让新订阅者也能收到最近一次发布的地图。
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  // 中文：LoadMap 服务运行期重新读取 YAML/图像，更新缓存并发布新地图。
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  LOG_INFO("Activating MapServer lifecycle node");

  // Publish the map using the latched topic
  // 中文：激活发布器；如果配置阶段已加载地图，则立即发布一次缓存地图。
  occ_pub_->on_activate();
  if (map_available_) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
    LOG_INFO(
      "MapServer published latched map width={}, height={}, resolution={}",
      msg_.info.width, msg_.info.height, msg_.info.resolution);
  } else {
    LOG_INFO("MapServer activated without cached map");
  }

  // create bond connection
  // 中文：创建 lifecycle bond，供 lifecycle_manager 监控节点存活。
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  LOG_INFO("Deactivating MapServer lifecycle node");

  occ_pub_->on_deactivate();

  // destroy bond connection
  // 中文：退出 active 状态时断开 lifecycle bond。
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  LOG_INFO("Cleaning up MapServer resources");

  occ_pub_.reset();
  occ_service_.reset();
  load_map_service_.reset();
  map_available_ = false;
  msg_ = nav_msgs::msg::OccupancyGrid();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  LOG_INFO("Shutting down MapServer lifecycle node");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  // 中文：生命周期未激活时不返回地图，避免外部拿到未准备好的缓存数据。
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetMap request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling GetMap request");
  LOG_INFO(
    "Handling GetMap request from OccupancyGrid cache map_available={}, frame_id='{}', width={}, height={}, resolution={}",
    map_available_, msg_.header.frame_id.c_str(), msg_.info.width, msg_.info.height,
    msg_.info.resolution);
  response->map = msg_;
}

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  // if not in ACTIVE state, ignore request
  // 中文：LoadMap 只允许 active 状态处理，保证发布器和服务都已完成生命周期激活。
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received LoadMap request but not in ACTIVE state, ignoring!");
    response->result = response->RESULT_UNDEFINED_FAILURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling LoadMap request");
  // Load from file
  // 中文：服务请求携带 map_url；读取成功后更新 msg_ 缓存并发布到地图话题。
  LOG_INFO("Handling LoadMap request map_url='{}'", request->map_url.c_str());
  if (loadMapResponseFromYaml(request->map_url, response)) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));  // publish new map
    LOG_INFO(
      "MapServer loaded and published new map width={}, height={}, resolution={}",
      msg_.info.width, msg_.info.height, msg_.info.resolution);
  } else {
    LOG_INFO(
      "MapServer failed to load requested map '{}', result={}",
      request->map_url.c_str(), response->result);
  }
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  LOG_INFO("MapServer loading map response from YAML '{}'", yaml_file.c_str());
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      LOG_INFO("MapServer load failed: map does not exist '{}'", yaml_file.c_str());
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      LOG_INFO("MapServer load failed: invalid metadata '{}'", yaml_file.c_str());
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      LOG_INFO("MapServer load failed: invalid map data '{}'", yaml_file.c_str());
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to specific node
      // 中文：底层 map_io 默认 frame_id 为 map，这里按节点参数修正 header 和加载时间。
      updateMsgHeader();

      map_available_ = true;
      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
      LOG_INFO(
        "MapServer cached OccupancyGrid layer from '{}', frame_id='{}', width={}, height={}, resolution={}, origin=({}, {}, {})",
        yaml_file.c_str(), frame_id_.c_str(), msg_.info.width, msg_.info.height,
        msg_.info.resolution, msg_.info.origin.position.x, msg_.info.origin.position.y,
        msg_.info.origin.position.z);
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  // 中文：将地图加载时间和消息时间戳刷新为当前节点时间。
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();
  LOG_INFO(
    "MapServer updated OccupancyGrid header frame_id='{}', stamp_sec={}, stamp_nanosec={}",
    msg_.header.frame_id.c_str(), msg_.header.stamp.sec, msg_.header.stamp.nanosec);
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapServer)
