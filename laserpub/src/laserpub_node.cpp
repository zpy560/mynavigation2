#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserPub : public rclcpp::Node
{
public:
  LaserPub()
  : Node("laserpub")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    topic_ = declare_parameter<std::string>("topic", "scan");
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    range_min_ = declare_parameter<double>("range_min", 0.12);
    range_max_ = declare_parameter<double>("range_max", 3.5);
    default_range_ = declare_parameter<double>("default_range", range_max_);
    sample_count_ = declare_parameter<int>("sample_count", 360);
    obstacle_range_ = declare_parameter<double>("obstacle_range", 1.0);
    obstacle_width_ = declare_parameter<int>("obstacle_width", 6);
    publish_test_obstacles_ = declare_parameter<bool>("publish_test_obstacles", true);
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);

    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than 0.0");
    }
    if (sample_count_ < 2) {
      throw std::invalid_argument("sample_count must be at least 2");
    }
    if (angle_max_ <= angle_min_) {
      throw std::invalid_argument("angle_max must be greater than angle_min");
    }
    if (range_max_ <= range_min_) {
      throw std::invalid_argument("range_max must be greater than range_min");
    }

    default_range_ = std::clamp(default_range_, range_min_, range_max_);
    obstacle_range_ = std::clamp(obstacle_range_, range_min_, range_max_);
    obstacle_width_ = std::max(0, obstacle_width_);
    scan_period_ = 1.0 / publish_rate;
    angle_increment_ = (angle_max_ - angle_min_) / static_cast<double>(sample_count_ - 1);
    time_increment_ = scan_period_ / static_cast<double>(sample_count_);
    ranges_.assign(static_cast<size_t>(sample_count_), static_cast<float>(default_range_));
    if (publish_test_obstacles_) {
      addObstacleBeam(0.0);
      addObstacleBeam(M_PI * 0.5);
      addObstacleBeam(-M_PI * 0.5);
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(scan_period_)),
      std::bind(&LaserPub::publishScan, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing LaserScan topic='/%s' frame_id='%s' samples=%d range=[%.2f, %.2f] rate=%.2fHz test_obstacles=%s",
      topic_.c_str(), frame_id_.c_str(), sample_count_, range_min_, range_max_, publish_rate,
      publish_test_obstacles_ ? "true" : "false");
  }

private:
  void addObstacleBeam(const double angle)
  {
    const int center = static_cast<int>(std::lround((angle - angle_min_) / angle_increment_));
    for (int offset = -obstacle_width_; offset <= obstacle_width_; ++offset) {
      const int index = center + offset;
      if (index >= 0 && index < sample_count_) {
        ranges_[static_cast<size_t>(index)] = static_cast<float>(obstacle_range_);
      }
    }
  }

  void publishScan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = frame_id_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = static_cast<float>(time_increment_);
    scan.scan_time = static_cast<float>(scan_period_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = ranges_;
    scan.intensities.assign(ranges_.size(), 0.0F);

    publisher_->publish(scan);
  }

  std::string frame_id_;
  std::string topic_;
  double angle_min_;
  double angle_max_;
  double range_min_;
  double range_max_;
  double default_range_;
  double obstacle_range_;
  int obstacle_width_;
  bool publish_test_obstacles_;
  int sample_count_;
  double scan_period_;
  double angle_increment_;
  double time_increment_;
  std::vector<float> ranges_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
