#include <chrono>
#include <cstdint>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class LioToFcAdapter final : public rclcpp::Node
{
public:
  LioToFcAdapter()
  : Node("lio_to_fc_adapter")
  {
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odometry_001", 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        receive_odometry(std::move(message));
      });
    timer_ = create_wall_timer(50ms, [this]() {publish_latest();});
    RCLCPP_INFO(
      get_logger(),
      "Direct pose relay /odometry -> /odometry_001 at 20 Hz; twist is zeroed");
  }

private:
  void receive_odometry(nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (have_previous_stamp_ && stamp_ns < previous_stamp_ns_) {
      ++rollback_count_;
      if (rollback_count_ == 1 || rollback_count_ % 100 == 0) {
        const double rollback_ms =
          static_cast<double>(previous_stamp_ns_ - stamp_ns) / 1.0e6;
        RCLCPP_WARN(
          get_logger(),
          "FAST-LIO header rollback observed (count=%zu, latest=%.3f ms); "
          "sample retained",
          rollback_count_, rollback_ms);
      }
    }
    previous_stamp_ns_ = stamp_ns;
    have_previous_stamp_ = true;
    latest_ = std::move(message);
    ++source_generation_;
  }

  void publish_latest()
  {
    if (!latest_ || source_generation_ == published_generation_) {
      return;
    }
    nav_msgs::msg::Odometry output = *latest_;
    output.twist.twist.linear.x = 0.0;
    output.twist.twist.linear.y = 0.0;
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.x = 0.0;
    output.twist.twist.angular.y = 0.0;
    output.twist.twist.angular.z = 0.0;
    publisher_->publish(output);
    published_generation_ = source_generation_;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_;
  std::uint64_t source_generation_{0};
  std::uint64_t published_generation_{0};
  std::int64_t previous_stamp_ns_{0};
  std::size_t rollback_count_{0};
  bool have_previous_stamp_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioToFcAdapter>());
  rclcpp::shutdown();
  return 0;
}
