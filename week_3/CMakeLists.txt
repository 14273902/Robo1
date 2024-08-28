cmake_minimum_required(VERSION 3.5)
project(sprint1_assessed)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Declare the executable
add_executable(laser_scan_filter_node src/laser_scan_filter_node.cpp)

# Link dependencies
ament_target_dependencies(laser_scan_filter_node rclcpp sensor_msgs)

# Install the executable
install(TARGETS
  laser_scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament package
ament_package()
