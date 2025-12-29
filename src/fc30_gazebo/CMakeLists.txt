cmake_minimum_required(VERSION 3.8)
project(fc30_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(communication REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS})

# Try to find ros_gz_sim, but make it optional
find_package(ros_gz_sim QUIET)

# Check if ros_gz_sim was found
if(ros_gz_sim_FOUND)
  message(STATUS "ROS Gazebo packages found, building with full Gazebo support")
else()
  message(WARNING "ROS Gazebo packages not found, building without Gazebo support")
endif()

# Add executables
add_executable(fc30_controller src/fc30_controller.cpp)
ament_target_dependencies(fc30_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  communication
)

add_executable(psdk_bridge src/psdk_bridge.cpp)
ament_target_dependencies(psdk_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  communication
)

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Add trajectory_planner executable
add_executable(trajectory_planner src/trajectory_planner.cpp)
ament_target_dependencies(trajectory_planner
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
)

# Find required packages for state estimator
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add Gazebo plugin for rotor dynamics
# Commented out due to gz/sim header issues
# if(ros_gz_sim_FOUND)
#   find_package(geometry_msgs REQUIRED)
#   find_package(rosidl_default_generators REQUIRED)
#   
#   add_library(rotor_dynamics_plugin SHARED src/rotor_dynamics_plugin.cpp)
#   target_include_directories(rotor_dynamics_plugin PRIVATE ${ros_gz_sim_INCLUDE_DIRS})
#   target_link_libraries(rotor_dynamics_plugin ${ros_gz_sim_LIBRARIES} ${geometry_msgs_LIBRARIES})
#   
#   install(TARGETS rotor_dynamics_plugin
#     DESTINATION lib/${PROJECT_NAME}
#   )
# endif()

# Install executables
install(TARGETS
  fc30_controller
  psdk_bridge
  state_estimator
  trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for Gazebo models, worlds, launch and config
install(DIRECTORY models worlds launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Set Gazebo model path
if(NOT DEFINED ENV{GAZEBO_MODEL_PATH})
  set(ENV{GAZEBO_MODEL_PATH} ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models)
else()
  set(ENV{GAZEBO_MODEL_PATH} $ENV{GAZEBO_MODEL_PATH}:${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()