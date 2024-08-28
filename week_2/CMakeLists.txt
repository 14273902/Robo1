# cmake_minimum_required(VERSION 3.5)
# project(laser_scan_processor_node)

# # Default to C++14
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# add_executable(laser_scan_processor_node src/solution.cpp)

# ament_target_dependencies(laser_scan_processor_node
#   rclcpp
#   sensor_msgs
#   std_msgs
# )

# install(TARGETS
#   laser_scan_processor_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()










cmake_minimum_required(VERSION 3.8)
project(week_2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(solution src/solution.cpp)

# Link dependencies
ament_target_dependencies(solution rclcpp sensor_msgs)

# Install the executable
install(TARGETS
  solution
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if needed (optional)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
