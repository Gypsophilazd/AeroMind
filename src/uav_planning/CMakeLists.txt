cmake_minimum_required(VERSION 3.8)
project(uav_planning)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robin CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(kiss_matcher CONFIG REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(thin_odom_adapter src/thin_odom_adapter.cpp)
target_compile_features(thin_odom_adapter PUBLIC cxx_std_17)
ament_target_dependencies(thin_odom_adapter diagnostic_msgs nav_msgs rclcpp)

add_executable(registered_cloud_map_saver src/registered_cloud_map_saver.cpp)
target_compile_features(registered_cloud_map_saver PUBLIC cxx_std_17)
target_include_directories(registered_cloud_map_saver PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(registered_cloud_map_saver
  pcl_conversions rclcpp sensor_msgs std_srvs)
target_link_libraries(registered_cloud_map_saver ${PCL_LIBRARIES})

add_executable(kiss_relocalizer src/kiss_relocalizer.cpp)
target_compile_features(kiss_relocalizer PUBLIC cxx_std_17)
target_include_directories(kiss_relocalizer PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(kiss_relocalizer
  geometry_msgs pcl_conversions rclcpp sensor_msgs std_msgs std_srvs tf2_ros)
target_link_libraries(kiss_relocalizer kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES})

add_executable(kiss_offline_validator src/kiss_offline_validator.cpp)
target_compile_features(kiss_offline_validator PUBLIC cxx_std_17)
target_include_directories(kiss_offline_validator PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(kiss_offline_validator
  kiss_matcher::kiss_matcher_core ${PCL_LIBRARIES})

add_executable(relocalization_demo_source src/relocalization_demo_source.cpp)
target_compile_features(relocalization_demo_source PUBLIC cxx_std_17)
target_include_directories(relocalization_demo_source PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(relocalization_demo_source
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs tf2_ros
  visualization_msgs)
target_link_libraries(relocalization_demo_source ${PCL_LIBRARIES})

install(TARGETS
  thin_odom_adapter
  registered_cloud_map_saver
  kiss_relocalizer
  kiss_offline_validator
  relocalization_demo_source
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/final_navigation_simulation.py
  scripts/ego_goal_adapter.py
  scripts/ego_trajectory_executor.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
