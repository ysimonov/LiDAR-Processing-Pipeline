#ifndef PROCESSING_DOWNSAMPLING
#define PROCESSING_DOWNSAMPLING

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 Packages
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

namespace downsampling
{
// Node that consumes messages.
class Subscriber : public rclcpp::Node
{
  public:
    Subscriber(const std::string &name, const std::string &input);

  private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

// Node that produces messages.
class Publisher : public rclcpp::Node
{
  public:
    Publisher(const std::string &name, const std::string &output);

  private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace downsampling

#endif // PROCESSING_DOWNSAMPLING