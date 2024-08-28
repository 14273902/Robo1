cmake_minimum_required(VERSION 3.5)
project(cpp_circle)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_processor src/image_processor.cpp)

ament_target_dependencies(image_processor
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  image_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
