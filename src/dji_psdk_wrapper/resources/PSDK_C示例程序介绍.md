# PSDK C示例程序介绍

## 1. 示例程序概述

DJI PSDK（Payload Software Development Kit）提供了丰富的C语言示例程序，帮助开发者快速了解和使用PSDK的各种功能。这些示例程序涵盖了无人机的核心功能，包括飞行控制、相机管理、云台控制、实时视频、数据传输等。

## 2. 示例程序目录结构

PSDK C示例程序位于以下目录：
```
/home/lcf/ros2_hoisting_system/src/dji_psdk_wrapper/third_party/dji_psdk/samples/sample_c/
```

主要包含以下模块：

| 模块名称 | 功能描述 |
|---------|---------|
| camera_emu | 相机模拟器示例 |
| camera_manager | 相机管理功能示例 |
| cloud_api | 云API示例 |
| data_transmission | 数据传输示例 |
| fc_subscription | 飞控数据订阅示例 |
| flight_control | 飞行控制示例 |
| gimbal_emu | 云台模拟器示例 |
| gimbal_manager | 云台管理示例 |
| hms | 健康管理系统示例 |
| interest_point | 兴趣点任务示例 |
| liveview | 实时视频示例 |
| mop_channel | MOP通道示例 |
| payload_collaboration | 负载协同示例 |
| perception | 感知系统示例 |
| positioning | 定位系统示例 |
| power_management | 电源管理示例 |
| tethered_battery | 系留电池示例 |
| time_sync | 时间同步示例 |
| upgrade | 升级示例 |
| utils | 工具函数示例 |
| waypoint_v2 | 航点V2任务示例 |
| waypoint_v3 | 航点V3任务示例 |
| widget | 组件示例 |
| widget_interaction_test | 组件交互测试示例 |
| xport | 负载扩展接口示例 |

## 3. 各模块示例程序详解

### 3.1 相机模拟器 (camera_emu)

**功能**：演示如何模拟相机功能，包括媒体文件管理、媒体文件操作等。

**主要文件**：
- `test_payload_cam_emu_base.c`：相机模拟器基础功能实现
- `test_payload_cam_emu_base.h`：相机模拟器基础功能头文件
- `test_payload_cam_emu_media.c`：相机模拟器媒体功能实现
- `test_payload_cam_emu_media.h`：相机模拟器媒体功能头文件
- `dji_media_file_manage/dji_media_file_core.c`：媒体文件核心功能
- `dji_media_file_manage/dji_media_file_jpg.c`：JPG媒体文件处理
- `dji_media_file_manage/dji_media_file_mp4.c`：MP4媒体文件处理

**媒体文件示例**：
- `media_file/PSDK_0001_ORG.jpg`：JPG图片示例
- `media_file/PSDK_0004_ORG.mp4`：MP4视频示例
- `media_file/PSDK_0005.h264`：H264视频示例

### 3.2 相机管理 (camera_manager)

**功能**：演示如何管理无人机相机，包括相机参数设置、拍照、录像等功能。

**主要文件**：
- `test_camera_manager.c`：相机管理功能实现
- `test_camera_manager.h`：相机管理功能头文件

### 3.3 云API (cloud_api)

**功能**：演示如何使用云API进行数据交互。

**主要文件**：
- `test_cloud_api_by_web_socket.c`：基于WebSocket的云API示例
- `test_cloud_api_by_web_socket.h`：基于WebSocket的云API头文件

### 3.4 数据传输 (data_transmission)

**功能**：演示如何进行数据传输。

**主要文件**：
- `test_data_transmission.c`：数据传输功能实现
- `test_data_transmission.h`：数据传输功能头文件

### 3.5 飞控数据订阅 (fc_subscription)

**功能**：演示如何订阅飞控数据。

**主要文件**：
- `test_fc_subscription.c`：飞控数据订阅功能实现
- `test_fc_subscription.h`：飞控数据订阅功能头文件

### 3.6 飞行控制 (flight_control)

**功能**：演示无人机飞行控制功能，包括起飞、降落、位置控制、速度控制、返航等。

**主要文件**：
- `test_flight_control.c`：飞行控制功能实现
- `test_flight_control.h`：飞行控制功能头文件

### 3.7 云台模拟器 (gimbal_emu)

**功能**：演示如何模拟云台功能。

**主要文件**：
- `test_payload_gimbal_emu.c`：云台模拟器功能实现
- `test_payload_gimbal_emu.h`：云台模拟器功能头文件

### 3.8 云台管理 (gimbal_manager)

**功能**：演示如何管理无人机云台，包括姿态控制、模式设置等。

**主要文件**：
- `test_gimbal_manager.c`：云台管理功能实现
- `test_gimbal_manager.h`：云台管理功能头文件

### 3.9 健康管理系统 (hms)

**功能**：演示健康管理系统（HMS）的使用，包括HMS事件处理、状态查询等。

**主要文件**：
- `test_hms.c`：HMS功能实现
- `test_hms.h`：HMS功能头文件

**配置文件**：
- `data/hms.json`：HMS配置文件

**文本资源**：
- `hms_text/cn/`：中文HMS文本
- `hms_text/en/`：英文HMS文本
- `hms_text_c/en/`：英文HMS文本（C格式）

### 3.10 兴趣点任务 (interest_point)

**功能**：演示兴趣点任务的使用。

**主要文件**：
- `test_interest_point.c`：兴趣点任务功能实现
- `test_interest_point.h`：兴趣点任务功能头文件

### 3.11 实时视频 (liveview)

**功能**：演示如何获取和处理无人机相机的实时视频流。

**主要文件**：
- `test_liveview.c`：实时视频功能实现
- `test_liveview.h`：实时视频功能头文件

### 3.12 MOP通道 (mop_channel)

**功能**：演示MOP（Mobile SDK Over Protocol）通道的使用。

**主要文件**：
- `test_mop_channel.c`：MOP通道功能实现
- `test_mop_channel.h`：MOP通道功能头文件

**测试文件**：
- `mop_channel_test_file/mop_send_test_file.mp4`：MOP通道测试视频文件

### 3.13 负载协同 (payload_collaboration)

**功能**：演示负载协同功能。

**主要文件**：
- `test_payload_collaboration.c`：负载协同功能实现
- `test_payload_collaboration.h`：负载协同功能头文件

### 3.14 感知系统 (perception)

**功能**：演示无人机感知系统的使用，包括LiDAR数据获取、雷达数据处理等。

**主要文件**：
- `test_perception.c`：感知系统功能实现
- `test_perception.h`：感知系统功能头文件

### 3.15 定位系统 (positioning)

**功能**：演示无人机定位系统的使用，包括GPS数据获取、RTK定位等。

**主要文件**：
- `test_positioning.c`：定位系统功能实现
- `test_positioning.h`：定位系统功能头文件

### 3.16 电源管理 (power_management)

**功能**：演示电源管理功能。

**主要文件**：
- `test_power_management.c`：电源管理功能实现
- `test_power_management.h`：电源管理功能头文件

### 3.17 系留电池 (tethered_battery)

**功能**：演示系留电池功能。

**主要文件**：
- `test_tethered_battery.c`：系留电池功能实现
- `test_tethered_battery.h`：系留电池功能头文件

### 3.18 时间同步 (time_sync)

**功能**：演示时间同步功能。

**主要文件**：
- `test_time_sync.c`：时间同步功能实现
- `test_time_sync.h`：时间同步功能头文件

### 3.19 升级 (upgrade)

**功能**：演示升级功能。

**主要文件**：
- `test_upgrade.c`：升级功能实现
- `test_upgrade.h`：升级功能头文件
- `test_upgrade_common_file_transfer.c`：升级文件传输功能实现
- `test_upgrade_common_file_transfer.h`：升级文件传输功能头文件
- `test_upgrade_platform_opt.c`：升级平台选项实现
- `test_upgrade_platform_opt.h`：升级平台选项头文件

### 3.20 工具函数 (utils)

**功能**：提供各种工具函数，包括JSON解析、文件操作、链表、MD5计算等。

**主要文件**：
- `cJSON.c`：JSON解析功能
- `cJSON.h`：JSON解析功能头文件
- `dji_config_manager.c`：配置管理功能
- `dji_config_manager.h`：配置管理功能头文件
- `util_buffer.c`：缓冲区管理功能
- `util_buffer.h`：缓冲区管理功能头文件
- `util_file.c`：文件操作功能
- `util_file.h`：文件操作功能头文件
- `util_link_list.c`：链表功能
- `util_link_list.h`：链表功能头文件
- `util_md5.c`：MD5计算功能
- `util_md5.h`：MD5计算功能头文件
- `util_misc.c`：杂项功能
- `util_misc.h`：杂项功能头文件
- `util_time.c`：时间管理功能
- `util_time.h`：时间管理功能头文件

### 3.21 航点V2任务 (waypoint_v2)

**功能**：演示Waypoint V2任务的使用（仅支持M300 RTK）。

**主要文件**：
- `test_waypoint_v2.c`：Waypoint V2任务功能实现
- `test_waypoint_v2.h`：Waypoint V2任务功能头文件

### 3.22 航点V3任务 (waypoint_v3)

**功能**：演示Waypoint V3任务的使用（不支持M300 RTK）。

**主要文件**：
- `test_waypoint_v3.c`：Waypoint V3任务功能实现
- `test_waypoint_v3.h`：Waypoint V3任务功能头文件

**测试文件**：
- `waypoint_file/waypoint_v3_test_file.kmz`：Waypoint V3任务KMZ文件示例
- `waypoint_file_c/waypoint_v3_test_file_kmz.h`：Waypoint V3任务KMZ文件头文件（C格式）

### 3.23 组件 (widget)

**功能**：演示组件功能的使用。

**主要文件**：
- `test_widget.c`：组件功能实现
- `test_widget.h`：组件功能头文件
- `test_widget_speaker.c`：组件扬声器功能实现
- `test_widget_speaker.h`：组件扬声器功能头文件

**资源文件**：
- `widget_file/cn_big_screen/`：中文大屏幕组件资源
- `widget_file/en_big_screen/`：英文大屏幕组件资源
- `widget_file_c/en_big_screen/`：英文大屏幕组件资源（C格式）

**资源头文件**：
- `file_binary_array_list_en.c`：英文文件二进制数组列表
- `file_binary_array_list_en.h`：英文文件二进制数组列表头文件

### 3.24 组件交互测试 (widget_interaction_test)

**功能**：演示组件交互功能的测试。

**主要文件**：
- `test_widget_interaction.c`：组件交互功能实现
- `test_widget_interaction.h`：组件交互功能头文件

**资源文件**：
- `widget_file/cn_big_screen/`：中文大屏幕组件资源
- `widget_file/en_big_screen/`：英文大屏幕组件资源
- `widget_file_c/en_big_screen/`：英文大屏幕组件资源（C格式）

**资源头文件**：
- `file_binary_array_list_en.c`：英文文件二进制数组列表
- `file_binary_array_list_en.h`：英文文件二进制数组列表头文件

### 3.25 负载扩展接口 (xport)

**功能**：演示负载扩展接口的使用。

**主要文件**：
- `test_payload_xport.c`：负载扩展接口功能实现
- `test_payload_xport.h`：负载扩展接口功能头文件

## 4. 平台示例

PSDK C示例程序还提供了平台特定的示例，位于`platform/`目录下，目前主要支持Linux平台：

### 4.1 Linux平台示例

**目录**：
```
/home/lcf/ros2_hoisting_system/src/dji_psdk_wrapper/third_party/dji_psdk/samples/sample_c/platform/linux/
```

**主要组件**：
- `common/`：通用组件，包括第三方库、监控、OSAL（操作系统抽象层）、升级平台选项等
- `raspberry_pi/`：树莓派平台示例，包括应用程序和硬件抽象层

## 5. 示例程序使用方法

大部分示例程序都提供了头文件和源文件，可以通过包含相应的头文件并调用相关函数来使用这些示例功能。示例程序通常遵循以下模式：

1. 初始化PSDK
2. 调用示例功能函数
3. 处理返回结果
4. 清理资源

例如，使用相机管理示例的基本流程：

```c
#include "camera_manager/test_camera_manager.h"

// 初始化PSDK
psdk_init();

// 调用相机管理示例功能
test_camera_manager_function();

// 清理资源
psdk_deinit();
```

## 6. 总结

PSDK C示例程序提供了全面的无人机功能演示，涵盖了飞行控制、相机管理、云台控制、实时视频、数据传输等核心功能。这些示例程序不仅帮助开发者快速了解PSDK的使用方法，还可以作为实际项目开发的参考模板。

通过这些示例程序，开发者可以：
1. 快速上手PSDK的各种功能
2. 了解PSDK的编程模型和API使用方法
3. 学习无人机应用开发的最佳实践
4. 基于示例程序进行二次开发，构建自己的无人机应用

对于想要深入了解PSDK开发的开发者来说，这些示例程序是非常宝贵的学习资源。