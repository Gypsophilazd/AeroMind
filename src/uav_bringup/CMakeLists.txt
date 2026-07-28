cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(lio_to_fc_adapter src/lio_to_fc_adapter.cpp)
ament_target_dependencies(lio_to_fc_adapter nav_msgs rclcpp)

install(TARGETS lio_to_fc_adapter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
