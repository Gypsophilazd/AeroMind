#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KissRelocalizer : public rclcpp::Node {
 public:
  KissRelocalizer() : Node("kiss_relocalizer") {
    map_path_ = declare_parameter<std::string>("map_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    map_topic_ = declare_parameter<std::string>("map_topic", "/prior_map");
    aligned_topic_ =
        declare_parameter<std::string>("aligned_cloud_topic", "/relocalization/aligned_cloud");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    voxel_size_ = declare_parameter<double>("voxel_size_m", 0.30);
    min_inliers_ = declare_parameter<int>("min_inliers", 5);
    tf_rate_ = declare_parameter<double>("tf_rate_hz", 20.0);
    publish_identity_ = declare_parameter<bool>("publish_identity_until_localized", true);
    declare_parameter<double>("initial_x", 0.0);
    declare_parameter<double>("initial_y", 0.0);
    declare_parameter<double>("initial_z", 0.0);
    declare_parameter<double>("initial_yaw", 0.0);
    if (map_path_.empty() || !(voxel_size_ > 0.0) || min_inliers_ < 2 || !(tf_rate_ > 0.0)) {
      throw std::invalid_argument("map_path and positive KISS/TF parameters are required");
    }
    if (pcl::io::loadPCDFile<Point>(map_path_, prior_map_) < 0 || prior_map_.empty()) {
      throw std::runtime_error("cannot load nonempty PCD prior map: " + map_path_);
    }
    removeInvalid(prior_map_);
    if (prior_map_.size() < 50) throw std::runtime_error("prior map has too few finite points");

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, latched);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_topic_, latched);
    status_pub_ = create_publisher<std_msgs::msg::String>("/relocalization/status", latched);

    rclcpp::QoS cloud_qos(1);
    cloud_qos.reliable().durability_volatile();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        std::bind(&KissRelocalizer::cloudCallback, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::Trigger>(
        "/relocalize", std::bind(&KissRelocalizer::serviceCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_rate_),
                                  std::bind(&KissRelocalizer::publishTf, this));
    map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishPriorMap(); });
    publishPriorMap();
    publishStatus("WAITING_FOR_SCAN map=" + map_path_);
    RCLCPP_INFO(get_logger(), "KISS relocalizer ready: source=%s(%s), target=%s(%s)",
                cloud_topic_.c_str(), odom_frame_.c_str(), map_path_.c_str(), map_frame_.c_str());
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  static void removeInvalid(Cloud &cloud) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, indices);
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
  }

  static std::vector<Eigen::Vector3f> toVector(const Cloud &cloud) {
    std::vector<Eigen::Vector3f> result;
    result.reserve(cloud.size());
    for (const auto &point : cloud) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        result.emplace_back(point.x, point.y, point.z);
      }
    }
    return result;
  }

  static bool finiteTransform(const Eigen::Matrix4d &transform) {
    return transform.array().isFinite().all() &&
           std::abs(transform.block<3, 3>(0, 0).determinant() - 1.0) < 1e-2;
  }

  Eigen::Matrix4d coarseGuess() const {
    const double x = get_parameter("initial_x").as_double();
    const double y = get_parameter("initial_y").as_double();
    const double z = get_parameter("initial_z").as_double();
    const double yaw = get_parameter("initial_yaw").as_double();
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    result.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return result;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    if (message->header.frame_id != odom_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring cloud frame '%s'; expected '%s'",
                            message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    Cloud cloud;
    pcl::fromROSMsg(*message, cloud);
    removeInvalid(cloud);
    if (cloud.size() < 50) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = std::move(cloud);
    latest_cloud_stamp_ = message->header.stamp;
    have_cloud_ = true;
  }

  void serviceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    Cloud source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_cloud_) {
        response->success = false;
        response->message = "no finite odom-frame cloud is available";
        return;
      }
      source = latest_cloud_;
    }

    const Eigen::Matrix4d initial = coarseGuess();
    if (!finiteTransform(initial)) {
      response->success = false;
      response->message = "non-finite coarse initial guess";
      return;
    }
    Cloud coarse_source;
    pcl::transformPointCloud(source, coarse_source, initial.cast<float>());

    publishStatus("MATCHING");
    kiss_matcher::KISSMatcherConfig config(static_cast<float>(voxel_size_));
    config.use_quatro_ = true;
    config.use_ratio_test_ = false;
    kiss_matcher::KISSMatcher matcher(config);
    const auto solution = matcher.estimate(toVector(coarse_source), toVector(prior_map_));
    Eigen::Matrix4d residual = Eigen::Matrix4d::Identity();
    residual.block<3, 3>(0, 0) = solution.rotation;
    residual.block<3, 1>(0, 3) = solution.translation;
    const Eigen::Matrix4d map_T_odom = residual * initial;
    const std::size_t inliers = matcher.getNumFinalInliers();
    if (!solution.valid || inliers < static_cast<std::size_t>(min_inliers_) ||
        !finiteTransform(map_T_odom)) {
      std::ostringstream text;
      text << "KISS rejected: valid=" << solution.valid << " inliers=" << inliers;
      response->success = false;
      response->message = text.str();
      publishStatus("FAILED " + response->message);
      return;
    }

    Cloud aligned;
    pcl::transformPointCloud(source, aligned, map_T_odom.cast<float>());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_T_odom_ = map_T_odom;
      localized_ = true;
    }
    sensor_msgs::msg::PointCloud2 aligned_message;
    pcl::toROSMsg(aligned, aligned_message);
    aligned_message.header.frame_id = map_frame_;
    aligned_message.header.stamp = now();
    aligned_pub_->publish(aligned_message);
    publishTf();

    const Eigen::Vector3d translation = map_T_odom.block<3, 1>(0, 3);
    const double yaw = std::atan2(map_T_odom(1, 0), map_T_odom(0, 0));
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(6);
    text << "T_map_odom x=" << translation.x() << " y=" << translation.y()
         << " z=" << translation.z() << " yaw=" << yaw << " inliers=" << inliers;
    response->success = true;
    response->message = text.str();
    publishStatus("LOCALIZED " + response->message);
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void publishPriorMap() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(prior_map_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    map_pub_->publish(message);
  }

  void publishStatus(const std::string &value) {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  void publishTf() {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    bool should_publish = publish_identity_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (localized_) {
        transform = map_T_odom_;
        should_publish = true;
      }
    }
    if (!should_publish) return;
    const Eigen::Quaterniond quaternion(transform.block<3, 3>(0, 0));
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform.translation.x = transform(0, 3);
    message.transform.translation.y = transform(1, 3);
    message.transform.translation.z = transform(2, 3);
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    tf_broadcaster_->sendTransform(message);
  }

  std::string map_path_;
  std::string cloud_topic_;
  std::string map_topic_;
  std::string aligned_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  double voxel_size_;
  int min_inliers_;
  double tf_rate_;
  bool publish_identity_;
  Cloud prior_map_;
  Cloud latest_cloud_;
  builtin_interfaces::msg::Time latest_cloud_stamp_;
  bool have_cloud_{false};
  bool localized_{false};
  Eigen::Matrix4d map_T_odom_{Eigen::Matrix4d::Identity()};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KissRelocalizer>());
  rclcpp::shutdown();
  return 0;
}
