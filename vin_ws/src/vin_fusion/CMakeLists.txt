cmake_minimum_required(VERSION 3.8)
project(vin_fusion)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Targets get added here as the files appear:
#   1. library  vin_fusion_core  (src/eskf.cpp, src/measurements.cpp) - no ROS, just Eigen
#   2. node     fusion_node      (src/fusion_node.cpp) - links the library + ROS
#   3. test     test_eskf        (test/test_eskf.cpp)  - gtest on the library
add_executable(fusion_node src/fusion_node.cpp)
target_include_directories(fusion_node PRIVATE include)
target_link_libraries(fusion_node Eigen3::Eigen)
ament_target_dependencies(fusion_node rclcpp sensor_msgs geometry_msgs)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})

ament_package()
