// Copyright 2026 UAV Workspace Contributors

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace uav_planning
{

class ThinOdomAdapter : public rclcpp::Node
{
public:
  ThinOdomAdapter()
  : Node("thin_odom_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odometry");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/uav/planning/odometry");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/uav/planning/odometry_adapter/diagnostics");
    window_size_ = declare_parameter<int>("window_size", 20);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.35);
    min_dt_sec_ = declare_parameter<double>("min_dt_sec", 0.04);
    max_dt_sec_ = declare_parameter<double>("max_dt_sec", 0.50);
    max_reasonable_velocity_mps_ =
      declare_parameter<double>("max_reasonable_velocity_mps", 8.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);

    validateParameters();

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, output_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, input_qos,
      std::bind(&ThinOdomAdapter::odomCallback, this, std::placeholders::_1));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / diagnostics_rate_hz_),
      std::bind(&ThinOdomAdapter::publishDiagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Planning odometry adapter: %s -> %s; steady-clock regression window=%d, "
      "EMA alpha=%.3f, dt=[%.3f, %.3f] s, max speed=%.3f m/s",
      input_topic_.c_str(), output_topic_.c_str(), window_size_, ema_alpha_, min_dt_sec_,
      max_dt_sec_, max_reasonable_velocity_mps_);
  }

  ~ThinOdomAdapter() override
  {
    RCLCPP_INFO(
      get_logger(),
      "Final adapter counters: messages=%lu valid_updates=%lu source_rollbacks=%lu "
      "invalid_dt=%lu rejected_velocity=%lu invalid_pose=%lu speed_min=%.6f "
      "speed_max=%.6f speed_mean=%.6f",
      message_count_, valid_update_count_, source_stamp_rollback_count_, invalid_dt_count_,
      rejected_velocity_count_, invalid_pose_count_, finiteMinSpeed(), max_speed_, meanSpeed());
  }

private:
  struct PositionSample
  {
    std::chrono::steady_clock::time_point time;
    std::array<double, 3> position;
  };

  void validateParameters() const
  {
    if (window_size_ < 2) {
      throw std::invalid_argument("window_size must be >= 2");
    }
    if (!(ema_alpha_ > 0.0 && ema_alpha_ <= 1.0)) {
      throw std::invalid_argument("ema_alpha must be in (0, 1]");
    }
    if (!(min_dt_sec_ > 0.0 && max_dt_sec_ > min_dt_sec_)) {
      throw std::invalid_argument("require 0 < min_dt_sec < max_dt_sec");
    }
    if (max_reasonable_velocity_mps_ <= 0.0 || diagnostics_rate_hz_ <= 0.0) {
      throw std::invalid_argument("max velocity and diagnostics rate must be positive");
    }
  }

  static bool finitePosition(const nav_msgs::msg::Odometry & msg)
  {
    return std::isfinite(msg.pose.pose.position.x) &&
           std::isfinite(msg.pose.pose.position.y) &&
           std::isfinite(msg.pose.pose.position.z);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ++message_count_;
    trackSourceStamp(*msg);

    nav_msgs::msg::Odometry output = *msg;
    if (finitePosition(*msg)) {
      updateVelocity(*msg);
    } else {
      ++invalid_pose_count_;
    }

    output.twist.twist.linear.x = filtered_velocity_[0];
    output.twist.twist.linear.y = filtered_velocity_[1];
    output.twist.twist.linear.z = filtered_velocity_[2];
    odom_pub_->publish(output);
  }

  void trackSourceStamp(const nav_msgs::msg::Odometry & msg)
  {
    const int64_t stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(msg.header.stamp.nanosec);
    if (have_source_stamp_ && stamp_ns < previous_source_stamp_ns_) {
      ++source_stamp_rollback_count_;
    }
    previous_source_stamp_ns_ = stamp_ns;
    have_source_stamp_ = true;
  }

  void updateVelocity(const nav_msgs::msg::Odometry & msg)
  {
    const auto now = std::chrono::steady_clock::now();
    PositionSample sample{
      now,
      {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z}};

    if (!samples_.empty()) {
      const double newest_gap = std::chrono::duration<double>(now - samples_.back().time).count();
      if (!(newest_gap > 0.0) || newest_gap > max_dt_sec_) {
        ++invalid_dt_count_;
        samples_.clear();
      }
    }

    samples_.push_back(sample);
    while (samples_.size() > static_cast<size_t>(window_size_)) {
      samples_.pop_front();
    }

    if (samples_.size() < 2) {
      return;
    }

    const double span =
      std::chrono::duration<double>(samples_.back().time - samples_.front().time).count();
    if (!(span > 0.0)) {
      ++invalid_dt_count_;
      samples_.clear();
      return;
    }
    if (span < min_dt_sec_) {
      return;
    }

    const auto origin = samples_.front().time;
    double mean_t = 0.0;
    std::array<double, 3> mean_position{0.0, 0.0, 0.0};
    for (const auto & item : samples_) {
      mean_t += std::chrono::duration<double>(item.time - origin).count();
      for (size_t axis = 0; axis < 3; ++axis) {
        mean_position[axis] += item.position[axis];
      }
    }
    const double count = static_cast<double>(samples_.size());
    mean_t /= count;
    for (double & value : mean_position) {
      value /= count;
    }

    double denominator = 0.0;
    std::array<double, 3> numerator{0.0, 0.0, 0.0};
    for (const auto & item : samples_) {
      const double centered_t =
        std::chrono::duration<double>(item.time - origin).count() - mean_t;
      denominator += centered_t * centered_t;
      for (size_t axis = 0; axis < 3; ++axis) {
        numerator[axis] += centered_t * (item.position[axis] - mean_position[axis]);
      }
    }
    if (!(denominator > std::numeric_limits<double>::epsilon())) {
      ++invalid_dt_count_;
      return;
    }

    std::array<double, 3> candidate{};
    double squared_speed = 0.0;
    for (size_t axis = 0; axis < 3; ++axis) {
      candidate[axis] = numerator[axis] / denominator;
      squared_speed += candidate[axis] * candidate[axis];
    }
    const double speed = std::sqrt(squared_speed);
    if (!std::isfinite(speed) || speed > max_reasonable_velocity_mps_) {
      ++rejected_velocity_count_;
      return;
    }

    if (!have_filtered_velocity_) {
      filtered_velocity_ = candidate;
      have_filtered_velocity_ = true;
    } else {
      for (size_t axis = 0; axis < 3; ++axis) {
        filtered_velocity_[axis] =
          ema_alpha_ * candidate[axis] + (1.0 - ema_alpha_) * filtered_velocity_[axis];
      }
    }

    const double filtered_speed = std::sqrt(
      filtered_velocity_[0] * filtered_velocity_[0] +
      filtered_velocity_[1] * filtered_velocity_[1] +
      filtered_velocity_[2] * filtered_velocity_[2]);
    min_speed_ = std::min(min_speed_, filtered_speed);
    max_speed_ = std::max(max_speed_, filtered_speed);
    speed_sum_ += filtered_speed;
    ++valid_update_count_;
    last_window_span_sec_ = span;
  }

  diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value) const
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  double finiteMinSpeed() const
  {
    return valid_update_count_ == 0 ? 0.0 : min_speed_;
  }

  double meanSpeed() const
  {
    return valid_update_count_ == 0 ? 0.0 : speed_sum_ / static_cast<double>(valid_update_count_);
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "thin_odom_adapter/velocity_estimator";
    status.hardware_id = "planning_odometry_adapter";
    status.level =
      (invalid_pose_count_ > 0 || rejected_velocity_count_ > 0) ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = valid_update_count_ > 0 ? "velocity estimate active" : "warming up";
    status.values = {
      keyValue("message_count", std::to_string(message_count_)),
      keyValue("valid_update_count", std::to_string(valid_update_count_)),
      keyValue("source_stamp_rollback_count", std::to_string(source_stamp_rollback_count_)),
      keyValue("invalid_dt_count", std::to_string(invalid_dt_count_)),
      keyValue("rejected_velocity_count", std::to_string(rejected_velocity_count_)),
      keyValue("invalid_pose_count", std::to_string(invalid_pose_count_)),
      keyValue("velocity_min_mps", std::to_string(finiteMinSpeed())),
      keyValue("velocity_max_mps", std::to_string(max_speed_)),
      keyValue("velocity_mean_mps", std::to_string(meanSpeed())),
      keyValue("last_window_span_sec", std::to_string(last_window_span_sec_)),
      keyValue("time_basis", "std::chrono::steady_clock"),
      keyValue("source_stamp_policy", "preserved; rollback counted")};
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  int window_size_;
  double ema_alpha_;
  double min_dt_sec_;
  double max_dt_sec_;
  double max_reasonable_velocity_mps_;
  double diagnostics_rate_hz_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::deque<PositionSample> samples_;
  std::array<double, 3> filtered_velocity_{0.0, 0.0, 0.0};
  bool have_filtered_velocity_{false};
  bool have_source_stamp_{false};
  int64_t previous_source_stamp_ns_{0};
  uint64_t message_count_{0};
  uint64_t valid_update_count_{0};
  uint64_t source_stamp_rollback_count_{0};
  uint64_t invalid_dt_count_{0};
  uint64_t rejected_velocity_count_{0};
  uint64_t invalid_pose_count_{0};
  double min_speed_{std::numeric_limits<double>::infinity()};
  double max_speed_{0.0};
  double speed_sum_{0.0};
  double last_window_span_sec_{0.0};
};

}  // namespace uav_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_planning::ThinOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}
