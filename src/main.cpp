// #include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <limits>

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode(const std::string &node_name = "obstacle_detector_node")
      : Node(node_name), node_name_(node_name) {
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&ObstacleDetectorNode::laserscan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "%s Ready for TurtleBot3...", node_name_.c_str());
  }

private:
  std::string node_name_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<float> valid_ranges;
    valid_ranges.reserve(msg->ranges.size());
    for (const auto &r : msg->ranges) {
      if (std::isfinite(r)) {
        valid_ranges.push_back(r);
      }
    }

    if (valid_ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid laser readings received!");
      return;
    }

    float min_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());

    // Log minimum distance
    RCLCPP_INFO(this->get_logger(), "Minimum distance to obstacle: %.2f meters", min_distance);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
