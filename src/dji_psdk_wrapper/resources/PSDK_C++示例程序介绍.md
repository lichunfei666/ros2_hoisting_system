# PSDK C++示例程序介绍

## 1. 示例程序概述

DJI PSDK（Payload Software Development Kit）提供了丰富的C++示例程序，帮助开发者快速了解和使用PSDK的各种功能。这些示例程序涵盖了无人机的核心功能，包括飞行控制、相机管理、云台控制、实时视频、感知系统等。

## 2. 示例程序目录结构

PSDK C++示例程序位于以下目录：
```
/home/lcf/ros2_hoisting_system/src/dji_psdk_wrapper/third_party/dji_psdk/samples/sample_c++/module_sample/
```

主要包含以下模块：

| 模块名称 | 功能描述 |
|---------|---------|
| camera_manager | 相机管理功能示例 |
| flight_controller | 飞行控制器功能示例 |
| gimbal | 云台控制功能示例 |
| hms_manager | HMS（健康管理系统）功能示例 |
| liveview | 实时视频流处理示例 |
| perception | 感知系统（LiDAR、雷达等）示例 |
| positioning | 定位系统示例 |
| widget_manager | 组件管理器示例 |

## 3. 各模块示例程序详解

### 3.1 相机管理 (camera_manager)

**功能**：演示如何使用PSDK管理无人机相机，包括相机参数设置、拍照、录像等功能。

**主要文件**：
- `test_camera_manager_entry.cpp`：相机管理示例程序入口
- `test_camera_manager_entry.h`：相机管理示例程序头文件

### 3.2 飞行控制器 (flight_controller)

**功能**：演示无人机飞行控制功能，包括起飞、降落、位置控制、速度控制、返航等。

**主要文件**：
- `test_flight_controller_entry.cpp`：飞行控制器示例程序入口
- `test_flight_controller_entry.h`：飞行控制器示例程序头文件
- `test_flight_controller_command_flying.cpp`：飞行控制命令示例
- `test_flight_controller_command_flying.h`：飞行控制命令头文件
- `config/flying_config.json`：飞行配置文件

**支持的功能示例**：
- [0] 键盘控制飞行
- [1] 起飞降落
- [2] 位置控制起飞降落
- [3] 返航并强制降落
- [4] 速度控制起飞降落
- [5] 紧急停止飞行
- [6] 设置和获取参数
- [7] Waypoint 2.0任务（仅支持M300 RTK）
- [8] Waypoint 3.0任务（不支持M300 RTK）
- [9] 兴趣点任务（仅支持M3E/M3T）
- [a] EU-C6 FTS触发示例（仅支持M3D/M3DT）
- [b] 慢速旋转螺旋桨示例（仅支持M400）
- [c] 选择FTS PWM触发位置（支持M4/M4T/M4D/M4TD）
- [d] 选择FTS PWM触发位置（支持M400）
- [e] 设置获取感知参数（支持M400）
- [f] 设置命令开始任务（支持M400）

### 3.3 云台控制 (gimbal)

**功能**：演示如何控制无人机云台，包括姿态控制、模式设置等。

**主要文件**：
- `test_gimbal_entry.cpp`：云台控制示例程序入口
- `test_gimbal_entry.hpp`：云台控制示例程序头文件

### 3.4 HMS管理器 (hms_manager)

**功能**：演示健康管理系统（HMS）的使用，包括HMS事件处理、状态查询等。

**主要文件**：
- `hms_manager_entry.cpp`：HMS管理器示例程序入口
- `hms_manager_entry.h`：HMS管理器示例程序头文件

### 3.5 实时视频 (liveview)

**功能**：演示如何获取和处理无人机相机的实时视频流，包括视频解码、图像处理等。

**主要文件**：
- `test_liveview_entry.cpp`：实时视频示例程序入口
- `test_liveview_entry.hpp`：实时视频示例程序头文件
- `test_liveview.cpp`：实时视频处理实现
- `test_liveview.hpp`：实时视频处理头文件
- `dji_camera_stream_decoder.cpp`：相机视频流解码器
- `dji_camera_stream_decoder.hpp`：相机视频流解码器头文件
- `dji_camera_image_handler.cpp`：相机图像处理
- `dji_camera_image_handler.hpp`：相机图像处理头文件
- `dji_liveview_object_detection.cpp`：实时视频目标检测
- `dji_liveview_object_detection.hpp`：实时视频目标检测头文件
- `image_processor_yolovfastest.cpp`：YOLO目标检测实现
- `image_processor_yolovfastest.hpp`：YOLO目标检测头文件

**支持的功能**：
- 视频流获取和解码
- 图像显示
- 目标检测（基于YOLO和Haar级联）

### 3.6 感知系统 (perception)

**功能**：演示无人机感知系统的使用，包括LiDAR数据获取、雷达数据处理等。

**主要文件**：
- `test_lidar_entry.cpp`：LiDAR示例程序入口
- `test_lidar_entry.hpp`：LiDAR示例程序头文件
- `test_perception_entry.cpp`：感知系统示例程序入口
- `test_perception_entry.hpp`：感知系统示例程序头文件
- `test_perception.cpp`：感知系统实现
- `test_perception.hpp`：感知系统头文件
- `test_radar_entry.cpp`：雷达示例程序入口
- `test_radar_entry.hpp`：雷达示例程序头文件

### 3.7 定位系统 (positioning)

**功能**：演示无人机定位系统的使用，包括GPS数据获取、RTK定位等。

**主要文件**：
- `test_positioning_entry.cpp`：定位系统示例程序入口
- `test_positioning_entry.h`：定位系统示例程序头文件

### 3.8 组件管理器 (widget_manager)

**功能**：演示组件管理器的使用，包括组件的添加、删除、配置等。

**主要文件**：
- `test_widget_manager.cpp`：组件管理器示例程序实现
- `test_widget_manager.hpp`：组件管理器示例程序头文件

## 4. 示例程序使用方法

### 4.1 运行示例程序

大部分示例程序都有一个入口函数，可以通过调用该函数来运行示例。例如，飞行控制器示例的入口函数是：

```cpp
void DjiUser_RunFlightControllerSample(void);
```

### 4.2 飞行控制器示例使用说明

飞行控制器示例提供了一个交互式菜单，用户可以通过键盘输入选择不同的功能：

```
| Available commands:                                                                                            |
| [0] Flight controller sample - control flying with keyboard                                                    |
| [1] Flight controller sample - take off landing                                                                |
| [2] Flight controller sample - take off position ctrl landing                                                  |
| [3] Flight controller sample - take off go home force landing                                                  |
| [4] Flight controller sample - take off velocity ctrl landing                                                  |
| [5] Flight controller sample - arrest flying                                                                   |
| [6] Flight controller sample - set get parameters                                                              |
| [7] Waypoint 2.0 sample - run airline mission by settings (only support on M300 RTK)                           |
| [8] Waypoint 3.0 sample - run airline mission by kmz file (not support on M300 RTK)                            |
| [9] Interest point sample - run interest point mission by settings (only support on M3E/M3T)                   |
| [a] EU-C6 FTS trigger sample - receive fts callback to trigger parachute function (only support on M3D/M3DT)   |
| [b] Slow rotate blade sample, only support on M400                                                             |
| [c] Select FTS pwm trigger position, support on M4/M4T/M4D/M4TD                                                |
| [d] Select FTS pwm trigger position, support on M400                                                           |
| [e] Flight controller sample - set get perception parameters, support on M400                                  |
| [f] Flight controller sample - set cmd start mission, support on M400                                          |
```

用户可以输入对应的数字或字母来选择要运行的功能。

## 5. 示例程序依赖和编译

### 5.1 依赖

PSDK C++示例程序依赖于以下组件：
- DJI PSDK库
- C++标准库
- 操作系统抽象层（OSAL）
- 其他第三方库（如OpenCV用于图像处理）

### 5.2 编译

示例程序的编译通常通过CMake进行，可以在PSDK安装目录下找到相应的CMakeLists.txt文件。编译步骤如下：

1. 创建编译目录：
```bash
mkdir -p build && cd build
```

2. 配置CMake：
```bash
cmake ..
```

3. 编译：
```bash
make
```

## 6. 示例程序与ROS2集成

在本项目中，PSDK的功能已经通过`dji_psdk_wrapper`包集成到ROS2系统中。该包封装了PSDK的核心功能，提供了ROS2接口，使开发者可以通过ROS2的话题、服务和动作来控制无人机。

主要集成模块包括：
- 飞行控制接口
- 相机控制接口
- 云台控制接口
- 实时视频流接口
- 感知系统接口

## 7. 总结

PSDK C++示例程序提供了全面的无人机功能演示，涵盖了飞行控制、相机管理、云台控制、实时视频、感知系统等核心功能。这些示例程序不仅帮助开发者快速了解PSDK的使用方法，还可以作为实际项目开发的参考模板。

通过这些示例程序，开发者可以：
1. 快速上手PSDK的各种功能
2. 了解PSDK的编程模型和API使用方法
3. 学习无人机应用开发的最佳实践
4. 基于示例程序进行二次开发，构建自己的无人机应用

对于想要深入了解PSDK开发的开发者来说，这些示例程序是非常宝贵的学习资源。