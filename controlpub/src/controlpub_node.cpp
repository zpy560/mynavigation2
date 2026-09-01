#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "byd_custom_msgs/msg/control_res.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spdlog_wrapper.hpp"

class ControlPub : public rclcpp::Node
{
public:
  ControlPub()
  : Node("controlpub")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/control_to_uart");
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(input_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(), std::bind(&ControlPub::onCmdVel, this, std::placeholders::_1));
    LOG_INFO("controlpub 已启动，仅在收到有效 {} 后转发到 {}", input_topic, output_topic_);
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (!msg) {LOG_WARN("收到空的 /cmd_vel 消息，忽略转发"); return;}
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {LOG_WARN("收到非有限 /cmd_vel，linear.x={}，angular.z={}，忽略转发", msg->linear.x, msg->angular.z); return;}
    std::lock_guard<std::mutex> lock(publisher_mutex_);
    if (!publisher_) {publisher_ = create_publisher<byd_custom_msgs::msg::ControlRes>(output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile()); LOG_INFO("首次收到有效 /cmd_vel，开始注册并转发到 {}", output_topic_);}
    byd_custom_msgs::msg::ControlRes control_msg;
    control_msg.v = msg->linear.x;
    control_msg.w = msg->angular.z;
    control_msg.v_lift = 0.0;
    control_msg.w_rotation = 0.0;
    publisher_->publish(control_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
  std::mutex publisher_mutex_;
  std::string output_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  SpdlogWrapper::init("controlpub", "controlpub");
  rclcpp::spin(std::make_shared<ControlPub>());
  rclcpp::shutdown();
  SpdlogWrapper::shutdown();
  return 0;
}
