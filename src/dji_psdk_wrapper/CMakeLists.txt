cmake_minimum_required(VERSION 3.8)
project(dji_psdk_wrapper)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error -Wno-missing-field-initializers)
endif()

# 设置PSDK版本和路径
set(PSDK_VERSION "master" CACHE STRING "DJI PSDK version")
set(PSDK_GIT_TAG "master" CACHE STRING "DJI PSDK git tag")
# 使用相对路径，相对于当前CMakeLists.txt文件的位置
set(PSDK_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk" CACHE PATH "Path to DJI PSDK installation")

# 检查PSDK是否安装
set(PSDK_VERSION_OK FALSE)
if(EXISTS ${PSDK_INSTALL_PATH}/psdk_lib/include/dji_version.h)
  set(PSDK_VERSION_OK TRUE)
  message(STATUS "PSDK already installed at ${PSDK_INSTALL_PATH}")
  
  # 读取并显示当前PSDK版本信息
  file(READ ${PSDK_INSTALL_PATH}/psdk_lib/include/dji_version.h PSDK_VERSION_CONTENT)
  string(REGEX MATCH "#define DJI_VERSION_MAJOR   ([0-9]+)" MAJOR_MATCH "${PSDK_VERSION_CONTENT}")
  string(REGEX MATCH "#define DJI_VERSION_MINOR   ([0-9]+)" MINOR_MATCH "${PSDK_VERSION_CONTENT}")
  string(REGEX MATCH "#define DJI_VERSION_MODIFY  ([0-9]+)" MODIFY_MATCH "${PSDK_VERSION_CONTENT}")
  
  if(MAJOR_MATCH AND MINOR_MATCH AND MODIFY_MATCH)
    set(DETECTED_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")
    message(STATUS "Detected PSDK version: ${DETECTED_VERSION}")
  endif()
else()
  message(STATUS "PSDK not found at ${PSDK_INSTALL_PATH}")
endif()

# 如果PSDK不存在，自动下载master分支
if(NOT EXISTS ${PSDK_INSTALL_PATH} OR NOT PSDK_VERSION_OK)
  message(STATUS "PSDK not found or version mismatch. Will download version ${PSDK_VERSION} from GitHub.")
  
  # 清理旧的PSDK安装（如果存在）
  if(EXISTS ${PSDK_INSTALL_PATH})
    message(STATUS "Removing old PSDK installation at ${PSDK_INSTALL_PATH}")
    file(REMOVE_RECURSE ${PSDK_INSTALL_PATH})
  endif()
  
  include(FetchContent)
  
  # 强制重新下载
  FetchContent_Declare(
    dji_psdk
    GIT_REPOSITORY https://github.com/dji-sdk/Payload-SDK.git
    GIT_TAG ${PSDK_GIT_TAG}
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED FALSE
  )
  
  # 清理缓存
  unset(dji_psdk_POPULATED CACHE)
  unset(dji_psdk_SOURCE_DIR CACHE)
  unset(dji_psdk_BINARY_DIR CACHE)
  
  FetchContent_MakeAvailable(dji_psdk)
  
  # 创建安装目录
  file(MAKE_DIRECTORY ${PSDK_INSTALL_PATH})
  
  # 复制PSDK文件到正确位置
  file(COPY ${dji_psdk_SOURCE_DIR}/psdk_lib/ DESTINATION ${PSDK_INSTALL_PATH}/psdk_lib/)
  file(COPY ${dji_psdk_SOURCE_DIR}/samples/ DESTINATION ${PSDK_INSTALL_PATH}/samples/)
  file(COPY ${dji_psdk_SOURCE_DIR}/tools/ DESTINATION ${PSDK_INSTALL_PATH}/tools/)
  file(COPY ${dji_psdk_SOURCE_DIR}/README.md DESTINATION ${PSDK_INSTALL_PATH}/)
  file(COPY ${dji_psdk_SOURCE_DIR}/LICENSE.txt DESTINATION ${PSDK_INSTALL_PATH}/)
  
  message(STATUS "PSDK ${PSDK_VERSION} downloaded and installed to ${PSDK_INSTALL_PATH}")
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
find_package(libstatistics_collector REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(mavros_msgs REQUIRED)
find_package(communication REQUIRED)

# 使用PSDK样例中的cJSON库

# ========================================
# 日志系统配置
# ========================================
# 使用ROS2自带的日志系统，不需要额外的spdlog依赖
message(STATUS "Using ROS2 built-in logging system")

# ========================================
# yaml-cpp 依赖配置
# ========================================

find_package(yaml-cpp QUIET)

if(NOT yaml-cpp_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.8.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(yaml-cpp)
endif()

# 检查线程支持
find_package(Threads REQUIRED)

# 检查 libusb 支持
find_package(PkgConfig REQUIRED)
pkg_check_modules(USB1 REQUIRED libusb-1.0)

# 设置PSDK包含目录（使用本地头文件）
set(PSDK_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk/psdk_lib/include
  ${USB1_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk/samples/sample_c/module_sample/utils
)

# 设置PSDK库目录（使用本地库文件）
set(PSDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk/psdk_lib/lib)

# 根据架构选择正确的库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(PSDK_LIB_ARCH "aarch64-linux-gnu-gcc")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set(PSDK_LIB_ARCH "arm-himix100-linux-gcc")
else()
  set(PSDK_LIB_ARCH "x86_64-linux-gnu-gcc")
endif()

message(STATUS "Using PSDK library architecture: ${PSDK_LIB_ARCH}")
message(STATUS "PSDK library directory: ${PSDK_LIB_DIR}/${PSDK_LIB_ARCH}")

# 检查PSDK库文件是否存在
set(PSDK_LIBRARY_FILE "${PSDK_LIB_DIR}/${PSDK_LIB_ARCH}/libpayloadsdk.a")
if(EXISTS ${PSDK_LIBRARY_FILE})
  message(STATUS "Found PSDK library: ${PSDK_LIBRARY_FILE}")
else()
  message(WARNING "PSDK library not found at ${PSDK_LIBRARY_FILE}")
  message(WARNING "Please copy the static library files to ${PSDK_LIB_DIR}/${PSDK_LIB_ARCH}/")
  message(WARNING "You can download PSDK from: https://developer.dji.com/payload-sdk/")
  message(WARNING "Alternatively, you can set PSDK_LIB_DIR to the correct path using -DPSDK_LIB_DIR=...")
endif()

# 创建third_party目录结构（如果不存在）
set(PSDK_MAIN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk/psdk_lib/include")
if(NOT EXISTS ${PSDK_MAIN_INCLUDE_DIR})
  message(STATUS "Creating third_party directory structure...")
  file(MAKE_DIRECTORY ${PSDK_MAIN_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${PSDK_LIB_DIR}/${PSDK_LIB_ARCH})
endif()

# 添加PSDK包装器库
add_library(psdk_wrapper SHARED
  src/dji_psdk_wrapper.cpp
  hal/hal_i2c.c
  hal/hal_network.c
  hal/hal_uart.c
  hal/hal_usb_bulk.c
  osal/osal.c
  osal/osal_fs.c
  osal/osal_socket.c
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dji_psdk/samples/sample_c/module_sample/utils/cJSON.c
)

target_include_directories(psdk_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PSDK_INCLUDE_DIRS}
)

# HAL和OSAL目录仅作为私有包含目录
target_include_directories(psdk_wrapper PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/hal
  ${CMAKE_CURRENT_SOURCE_DIR}/osal
)

# 添加LIBUSB_INSTALLED宏
target_compile_definitions(psdk_wrapper PRIVATE LIBUSB_INSTALLED)

# 链接PSDK库和系统库
target_link_libraries(psdk_wrapper
  ${PSDK_LIBRARY_FILE}
  ${USB1_LIBRARIES}
  pthread
  aio
  dl
  yaml-cpp
  Threads::Threads
  udev
)

ament_target_dependencies(psdk_wrapper
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  libstatistics_collector
  ament_index_cpp
  communication
)

# 添加PSDK节点
add_executable(dji_psdk_node src/dji_psdk_node.cpp)
target_include_directories(dji_psdk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dji_psdk_node psdk_wrapper)
ament_target_dependencies(dji_psdk_node rclcpp ament_index_cpp communication std_srvs)

# 显式添加std_srvs的include目录以解决IDE问题
target_include_directories(dji_psdk_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${std_srvs_INCLUDE_DIRS}
)

# 注释掉不存在的日志示例程序
# add_executable(logger_spdlog_example examples/logger_spdlog_example.cpp)
# target_include_directories(logger_spdlog_example PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# 
# target_link_libraries(logger_spdlog_example psdk_wrapper)
# ament_target_dependencies(logger_spdlog_example rclcpp)

# 安装库和可执行文件
install(TARGETS
  psdk_wrapper
  dji_psdk_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装包含目录
install(DIRECTORY
  include/dji_psdk_wrapper/
  DESTINATION include/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# 注释掉不存在的启动文件安装
# install(DIRECTORY
#   launch/
#   DESTINATION share/${PROJECT_NAME}/launch/
# )

# 安装PSDK示例配置（如果PSDK已安装）
if(EXISTS ${PSDK_INSTALL_PATH})
  install(DIRECTORY
    ${PSDK_INSTALL_PATH}/sample/platform/linux/common/config/
    DESTINATION share/${PROJECT_NAME}/config/psdk/
    OPTIONAL
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 创建环境钩子以自动设置LD_LIBRARY_PATH
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dji_psdk_wrapper_library_path.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/env-hooks/${PROJECT_NAME}_library_path.dsv"
  @ONLY
)

# 创建环境钩子以自动设置PSDK相关环境变量
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/dji_psdk_wrapper_env_vars.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/env-hooks/${PROJECT_NAME}_env_vars.dsv"
  @ONLY
)

ament_environment_hooks(
  "${CMAKE_CURRENT_BINARY_DIR}/env-hooks/${PROJECT_NAME}_library_path.dsv"
  "${CMAKE_CURRENT_BINARY_DIR}/env-hooks/${PROJECT_NAME}_env_vars.dsv"
)

ament_package()