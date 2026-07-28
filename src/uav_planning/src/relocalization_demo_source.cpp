#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class RelocalizationDemoSource : public rclcpp::Node {
 public:
  RelocalizationDemoSource() : Node("relocalization_demo_source") {
    source_path_ = declare_parameter<std::string>("source_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_pose_ = declare_parameter<bool>("publish_pose", true);
    publish_marker_ = declare_parameter<bool>("publish_marker", true);
    if (source_path_.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(source_path_, source_) < 0 ||
        source_.empty()) {
      throw std::runtime_error("cannot load restart-session source PCD: " + source_path_);
    }
    rclcpp::QoS cloud_qos(1);
    cloud_qos.reliable().transient_local();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, cloud_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/relocalization/state_marker", 1);
    status_sub_ = create_subscription<std_msgs::msg::String>(
        "/relocalization/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::ConstSharedPtr message) {
          status_ = message->data.rfind("LOCALIZED", 0) == 0 ? "AFTER: global pose recovered"
                                                               : "BEFORE: restarted odom origin";
        });
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&RelocalizationDemoSource::publish, this));
    RCLCPP_INFO(get_logger(), "Publishing recorded restart-session scan: %s (%zu points)",
                source_path_.c_str(), source_.size());
  }

 private:
  void publish() {
    const auto stamp = now();
    if (++counter_ % 20 == 1) {
      sensor_msgs::msg::PointCloud2 cloud;
      pcl::toROSMsg(source_, cloud);
      cloud.header.stamp = stamp;
      cloud.header.frame_id = odom_frame_;
      cloud_pub_->publish(cloud);
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.orientation.w = 1.0;
    if (publish_pose_) odom_pub_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.rotation.w = 1.0;
    if (publish_pose_) tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Path path;
    path.header = odometry.header;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    path.poses.push_back(pose);
    if (publish_pose_) path_pub_->publish(path);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "relocalization_state";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 1.4;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.45;
    marker.color.r = status_.rfind("AFTER", 0) == 0 ? 0.1f : 1.0f;
    marker.color.g = status_.rfind("AFTER", 0) == 0 ? 1.0f : 0.25f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;
    marker.text = status_;
    if (publish_marker_) marker_pub_->publish(marker);
  }

  std::string source_path_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_pose_;
  bool publish_marker_;
  std::string status_{"BEFORE: restarted odom origin"};
  pcl::PointCloud<pcl::PointXYZ> source_;
  std::size_t counter_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelocalizationDemoSource>());
  rclcpp::shutdown();
  return 0;
}
