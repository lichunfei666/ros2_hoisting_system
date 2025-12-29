cmake_minimum_required(VERSION 3.8)
project(ground_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(communication REQUIRED)

# Add executable
add_executable(ground_station_node src/ground_station_node.cpp)

# Link dependencies
ament_target_dependencies(ground_station_node
  rclcpp
  std_msgs
  std_srvs
  communication
)

# Install executable
install(TARGETS
  ground_station_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install web resources
install(DIRECTORY
  resources/
  DESTINATION share/${PROJECT_NAME}/resources/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()