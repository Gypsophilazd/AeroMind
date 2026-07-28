#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class RegisteredCloudMapSaver : public rclcpp::Node {
 public:
  RegisteredCloudMapSaver() : Node("registered_cloud_map_saver") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered");
    output_path_ = declare_parameter<std::string>("output_path", "maps/session_map.pcd");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    input_frame_ = declare_parameter<std::string>("input_frame", "odom");
    voxel_size_ = declare_parameter<double>("voxel_size_m", 0.15);
    compact_every_ = declare_parameter<int>("compact_every_clouds", 20);
    max_points_ = declare_parameter<int>("max_points", 3000000);
    if (!(voxel_size_ > 0.0) || compact_every_ < 1 || max_points_ < 1000) {
      throw std::invalid_argument("invalid map saver limits");
    }

    rclcpp::QoS input_qos(1);
    input_qos.reliable().durability_volatile();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, input_qos,
        std::bind(&RegisteredCloudMapSaver::cloudCallback, this, std::placeholders::_1));

    rclcpp::QoS map_qos(1);
    map_qos.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapping/accumulated_cloud", map_qos);
    service_ = create_service<std_srvs::srv::Trigger>(
        "/save_map", std::bind(&RegisteredCloudMapSaver::saveCallback, this,
                                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Accumulating %s (%s) for %s; voxel=%.3f m",
                input_topic_.c_str(), input_frame_.c_str(), output_path_.c_str(), voxel_size_);
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  static void removeInvalid(Cloud &cloud) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    cloud.points.erase(
        std::remove_if(cloud.points.begin(), cloud.points.end(), [](const Point &point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
        }),
        cloud.points.end());
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;
  }

  Cloud compact(const Cloud &input) const {
    Cloud result;
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    filter.setInputCloud(input.makeShared());
    filter.filter(result);
    removeInvalid(result);
    return result;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->header.frame_id != input_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring cloud frame '%s'; expected '%s' (no TF is applied)",
                            msg->header.frame_id.c_str(), input_frame_.c_str());
      return;
    }
    Cloud incoming;
    pcl::fromROSMsg(*msg, incoming);
    removeInvalid(incoming);
    if (incoming.empty()) return;

    Cloud snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      accumulated_ += incoming;
      ++cloud_count_;
      if (cloud_count_ % static_cast<std::size_t>(compact_every_) == 0 ||
          accumulated_.size() > static_cast<std::size_t>(max_points_)) {
        accumulated_ = compact(accumulated_);
      }
      snapshot = accumulated_;
    }
    if (cloud_count_ % static_cast<std::size_t>(compact_every_) == 0) {
      publish(snapshot, msg->header.stamp);
      RCLCPP_INFO(get_logger(), "Map buffer: %zu clouds, %zu voxel points",
                  cloud_count_, snapshot.size());
    }
  }

  void publish(const Cloud &cloud, const builtin_interfaces::msg::Time &stamp) {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = stamp;
    map_pub_->publish(message);
  }

  void saveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    Cloud result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulated_.empty()) {
        response->success = false;
        response->message = "no registered cloud has been received";
        return;
      }
      accumulated_ = compact(accumulated_);
      result = accumulated_;
    }

    try {
      const std::filesystem::path path(output_path_);
      if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
      if (pcl::io::savePCDFileBinaryCompressed(output_path_, result) != 0) {
        response->success = false;
        response->message = "PCL failed to write " + output_path_;
        return;
      }
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
      return;
    }
    publish(result, now());
    response->success = true;
    response->message = output_path_ + " points=" + std::to_string(result.size()) +
                        " frame=" + map_frame_ + " source_frame=" + input_frame_;
    RCLCPP_INFO(get_logger(), "Saved real registered-cloud map: %s", response->message.c_str());
  }

  std::string input_topic_;
  std::string output_path_;
  std::string map_frame_;
  std::string input_frame_;
  float voxel_size_;
  int compact_every_;
  int max_points_;
  std::size_t cloud_count_{0};
  Cloud accumulated_;
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredCloudMapSaver>());
  rclcpp::shutdown();
  return 0;
}
