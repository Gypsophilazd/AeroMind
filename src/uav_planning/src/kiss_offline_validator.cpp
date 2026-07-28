#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

std::vector<Eigen::Vector3f> toVector(const Cloud &cloud) {
  std::vector<Eigen::Vector3f> result;
  result.reserve(cloud.size());
  for (const auto &point : cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      result.emplace_back(point.x, point.y, point.z);
    }
  }
  return result;
}

double rotationError(const Eigen::Matrix3d &estimate, const Eigen::Matrix3d &truth) {
  const Eigen::Matrix3d delta = estimate * truth.transpose();
  return Eigen::AngleAxisd(delta).angle();
}

bool finiteTransform(const Eigen::Matrix4d &transform) {
  return transform.array().isFinite().all() &&
         std::abs(transform.block<3, 3>(0, 0).determinant() - 1.0) < 1e-2;
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 7 || argc > 9) {
    std::cerr << "Usage: kiss_offline_validator MAP.pcd X Y Z YAW_DEG EVIDENCE.json "
                 "[SOURCE.pcd] [REPEATS]\n";
    return 2;
  }
  const std::string map_path = argv[1];
  const double true_x = std::stod(argv[2]);
  const double true_y = std::stod(argv[3]);
  const double true_z = std::stod(argv[4]);
  const double true_yaw = std::stod(argv[5]) * M_PI / 180.0;
  const std::string evidence_path = argv[6];
  const std::string source_path = argc >= 8 ? argv[7] : "/tmp/relocalization_source.pcd";
  const int repeats = argc >= 9 ? std::stoi(argv[8]) : 3;
  if (repeats < 2) {
    std::cerr << "RELOCALIZATION_REPEATABILITY: FAIL (repeats must be >=2)\n";
    return 2;
  }

  Cloud map_raw;
  if (pcl::io::loadPCDFile<Point>(map_path, map_raw) < 0 || map_raw.empty()) {
    std::cerr << "KISS_OFFLINE_MATCH: FAIL (cannot load map)\n";
    return 1;
  }
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(map_raw, map_raw, finite_indices);
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(0.20f, 0.20f, 0.20f);
  filter.setInputCloud(map_raw.makeShared());
  Cloud map;
  filter.filter(map);
  if (map.size() < 100) {
    std::cerr << "KISS_OFFLINE_MATCH: FAIL (map has too few finite voxel points)\n";
    return 1;
  }

  Eigen::Matrix4d truth = Eigen::Matrix4d::Identity();
  truth.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(true_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  truth.block<3, 1>(0, 3) = Eigen::Vector3d(true_x, true_y, true_z);
  Cloud source;
  pcl::transformPointCloud(map, source, truth.inverse().cast<float>());
  if (pcl::io::savePCDFileBinaryCompressed(source_path, source) != 0) {
    std::cerr << "KISS_OFFLINE_MATCH: FAIL (cannot save restart-session source PCD)\n";
    return 1;
  }

  double worst_translation = 0.0;
  double worst_rotation = 0.0;
  double max_repeat_translation = 0.0;
  double max_repeat_rotation = 0.0;
  std::size_t minimum_inliers = std::numeric_limits<std::size_t>::max();
  bool all_valid = true;
  Eigen::Matrix4d first = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d last = Eigen::Matrix4d::Identity();
  for (int run = 0; run < repeats; ++run) {
    kiss_matcher::KISSMatcherConfig config(0.30f);
    config.use_quatro_ = true;
    config.use_ratio_test_ = false;
    kiss_matcher::KISSMatcher matcher(config);
    const auto solution = matcher.estimate(toVector(source), toVector(map));
    Eigen::Matrix4d estimate = Eigen::Matrix4d::Identity();
    estimate.block<3, 3>(0, 0) = solution.rotation;
    estimate.block<3, 1>(0, 3) = solution.translation;
    const auto inliers = matcher.getNumFinalInliers();
    const double translation_error =
        (estimate.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm();
    const double rotation_error = rotationError(
        estimate.block<3, 3>(0, 0), truth.block<3, 3>(0, 0));
    if (run == 0) first = estimate;
    if (run > 0) {
      max_repeat_translation = std::max(
          max_repeat_translation,
          (estimate.block<3, 1>(0, 3) - last.block<3, 1>(0, 3)).norm());
      max_repeat_rotation = std::max(
          max_repeat_rotation,
          rotationError(estimate.block<3, 3>(0, 0), last.block<3, 3>(0, 0)));
    }
    last = estimate;
    worst_translation = std::max(worst_translation, translation_error);
    worst_rotation = std::max(worst_rotation, rotation_error);
    minimum_inliers = std::min(minimum_inliers, inliers);
    all_valid = all_valid && solution.valid && finiteTransform(estimate) && inliers >= 5;
    std::cout << "run=" << run + 1 << " valid=" << solution.valid
              << " inliers=" << inliers << " translation_error_m=" << translation_error
              << " rotation_error_deg=" << rotation_error * 180.0 / M_PI << '\n';
  }

  // The source and target are voxelized at 0.20 m before matching.  A 0.05 m
  // repeatability bound remains well below one voxel while allowing KISS's
  // randomized correspondence sampling to vary by a few centimetres.
  constexpr double kMaxRepeatTranslationM = 0.05;
  constexpr double kMaxRepeatRotationRad = 0.5 * M_PI / 180.0;
  const bool direction_pass = all_valid && worst_translation < 0.15 &&
                              worst_rotation < 3.0 * M_PI / 180.0;
  const bool repeat_pass = max_repeat_translation < kMaxRepeatTranslationM &&
                           max_repeat_rotation < kMaxRepeatRotationRad;
  const bool pass = direction_pass && repeat_pass;
  const double recovered_x = first(0, 3);
  const double recovered_y = first(1, 3);
  const double recovered_z = first(2, 3);
  const double recovered_yaw = std::atan2(first(1, 0), first(0, 0));

  if (std::filesystem::path(evidence_path).has_parent_path()) {
    std::filesystem::create_directories(std::filesystem::path(evidence_path).parent_path());
  }
  std::ofstream json(evidence_path);
  json << std::fixed << std::setprecision(9)
       << "{\n"
       << "  \"test_kind\": \"recorded_registered_cloud_lio_restart_simulation\",\n"
       << "  \"map_path\": \"" << map_path << "\",\n"
       << "  \"source_path\": \"" << source_path << "\",\n"
       << "  \"map_points\": " << map.size() << ",\n"
       << "  \"source_points\": " << source.size() << ",\n"
       << "  \"repeats\": " << repeats << ",\n"
       << "  \"true_map_T_odom\": {\"x\": " << true_x << ", \"y\": " << true_y
       << ", \"z\": " << true_z << ", \"yaw_rad\": " << true_yaw << "},\n"
       << "  \"estimated_map_T_odom\": {\"x\": " << recovered_x
       << ", \"y\": " << recovered_y << ", \"z\": " << recovered_z
       << ", \"yaw_rad\": " << recovered_yaw << "},\n"
       << "  \"worst_translation_error_m\": " << worst_translation << ",\n"
       << "  \"worst_rotation_error_deg\": " << worst_rotation * 180.0 / M_PI << ",\n"
       << "  \"max_repeat_translation_delta_m\": " << max_repeat_translation << ",\n"
       << "  \"max_repeat_rotation_delta_deg\": " << max_repeat_rotation * 180.0 / M_PI
       << ",\n"
       << "  \"repeat_translation_limit_m\": " << kMaxRepeatTranslationM << ",\n"
       << "  \"repeat_rotation_limit_deg\": "
       << kMaxRepeatRotationRad * 180.0 / M_PI << ",\n"
       << "  \"minimum_final_inliers\": " << minimum_inliers << ",\n"
       << "  \"kiss_transform_direction_known\": " << (direction_pass ? "true" : "false")
       << ",\n"
       << "  \"map_odom_semantics_pass\": " << (direction_pass ? "true" : "false") << ",\n"
       << "  \"repeatability_pass\": " << (repeat_pass ? "true" : "false") << ",\n"
       << "  \"global_pose_recovery_pass\": " << (pass ? "true" : "false") << "\n"
       << "}\n";
  json.close();

  std::cout << "KISS_TRANSFORM_DIRECTION: " << (direction_pass ? "KNOWN" : "UNKNOWN") << '\n'
            << "MAP_ODOM_SEMANTICS: " << (direction_pass ? "PASS" : "FAIL") << '\n'
            << "KISS_OFFLINE_MATCH: " << (direction_pass ? "PASS" : "FAIL") << '\n'
            << "RELOCALIZATION_REPEATABILITY: " << (repeat_pass ? "PASS" : "FAIL") << '\n'
            << "LIO_RESTART_SIMULATION: " << (pass ? "PASS" : "FAIL") << '\n'
            << "GLOBAL_POSE_RECOVERY: " << (pass ? "PASS" : "FAIL") << '\n';
  return pass ? 0 : 1;
}
