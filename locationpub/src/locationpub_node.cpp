#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double half_yaw = yaw * 0.5;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(half_yaw);
  quaternion.w = std::cos(half_yaw);
  return quaternion;
}

}  // namespace

class LocationPub : public rclcpp::Node
{
public:
  LocationPub()
  : Node("locationpub")
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "map");
    child_frame_ = declare_parameter<std::string>("child_frame", "turtlebot3_waffle");
    x_ = declare_parameter<double>("x", 0.569);
    y_ = declare_parameter<double>("y", 0.541);
    z_ = declare_parameter<double>("z", 0.0);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    tf_time_offset_ = declare_parameter<double>("tf_time_offset", 0.2);
    const double publish_rate = declare_parameter<double>("publish_rate", 30.0);

    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than 0.0");
    }
    if (tf_time_offset_ < 0.0) {
      throw std::invalid_argument("tf_time_offset must be greater than or equal to 0.0");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocationPub::publishTf, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing localization TF %s -> %s at %.3f, %.3f, %.3f yaw %.3f with %.3fs stamp offset",
      parent_frame_.c_str(), child_frame_.c_str(), x_, y_, z_, yaw_, tf_time_offset_);
  }

private:
  void publishTf()
  {
    geometry_msgs::msg::TransformStamped transform;
    const auto stamp_offset = std::chrono::duration<double>(tf_time_offset_);
    const auto stamp = now() - rclcpp::Duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(stamp_offset));
    const auto stamp_ns = stamp.nanoseconds();
    transform.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
    transform.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = z_;
    transform.transform.rotation = quaternionFromYaw(yaw_);

    tf_broadcaster_->sendTransform(transform);
  }

  std::string parent_frame_;
  std::string child_frame_;
  double x_;
  double y_;
  double z_;
  double yaw_;
  double tf_time_offset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocationPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
