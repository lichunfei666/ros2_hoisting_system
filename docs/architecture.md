# 系统架构

## 系统概述

ROS2 Hoisting System是一个基于ROS 2的无人机吊装系统，用于控制无人机进行物品的吊运作业。系统采用模块化设计，各功能模块之间通过ROS 2的话题和服务进行通信，实现了无人机控制、机器人控制、地面站监控等功能。

系统还提供了一键启动和停止脚本，方便用户快速部署和管理所有服务。

## 整体架构

系统采用分层架构设计，主要分为以下几层：

1. **硬件抽象层**：负责与硬件设备的通信，包括无人机、机器人、传感器等
2. **核心控制层**：负责系统的核心控制逻辑，包括无人机控制、机器人控制等
3. **通信层**：负责系统各模块之间的通信，定义了统一的消息和服务接口
4. **应用层**：提供用户界面和高级应用功能，如地面站监控系统
5. **安全监控层**：负责系统的安全监控和故障处理

## 功能模块

### 0. 系统管理脚本

**职责**：提供一键启动和停止所有服务的功能，简化系统部署和管理。

**文件结构**：
```
ros2_hoisting_system/
├── start_all.sh    # 一键启动所有服务脚本
└── stop_all.sh     # 一键停止所有服务脚本
```

### 1. 通信模块 (communication)

**职责**：定义系统各模块之间通信的消息和服务接口，为其他模块提供统一的通信标准。

**文件结构**：
```
communication/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── FlightControlCommand.msg
    ├── RobotControlCommand.msg
    ├── GPSInfo.msg
    ├── DroneState.msg
    └── ...
    └── srv/
        ├── HookControl.srv
        ├── GetDroneInfo.srv
        └── ...
```

### 2. DJI PSDK封装模块 (dji_psdk_wrapper)

**职责**：封装DJI Payload SDK，提供与DJI无人机的通信接口，将无人机的数据和控制命令转换为ROS 2的话题和服务。

**文件结构**：
```
dji_psdk_wrapper/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── psdk_config.yaml
├── hal/
│   └── ...
├── include/
│   └── dji_psdk_wrapper/
│       ├── psdk_node.hpp
│       └── ...
├── osal/
│   └── ...
└── src/
    ├── psdk_node.cpp
    └── ...
```

### 3. 无人机控制模块 (drone_control)

**职责**：实现无人机的飞行控制逻辑，接收控制命令并通过PSDK封装模块发送给无人机。

**文件结构**：
```
drone_control/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── drone_control/
│       ├── drone_controller.hpp
│       └── ...
└── src/
    ├── drone_controller.cpp
    └── ...
```

### 4. 机器人控制模块 (robot_control)

**职责**：实现地面机器人的控制逻辑，包括机器人的移动、抓取等功能。

**文件结构**：
```
robot_control/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── robot_control/
│       ├── robot_controller.hpp
│       └── ...
└── src/
    ├── robot_controller.cpp
    └── ...
```

### 5. 地面站模块 (ground_station)

**职责**：提供用户界面，显示系统状态信息，接收用户输入并发送控制命令。

**文件结构**：
```
ground_station/
├── CMakeLists.txt
├── package.xml
├── resources/
│   └── ...
└── src/
    ├── ground_station_node.cpp
    └── ...
```

### 6. 安全监控模块 (safety_monitor)

**职责**：监控系统的运行状态，检测安全隐患并采取相应的安全措施。

**文件结构**：
```
safety_monitor/
├── CMakeLists.txt
├── package.xml
└── src/
    └── safety_monitor_node.cpp
```

### 7. 视觉处理模块 (vision_processing)

**职责**：处理摄像头采集的图像数据，实现目标检测、跟踪等功能。

**文件结构**：
```
vision_processing/
├── CMakeLists.txt
├── package.xml
└── src/
    └── vision_processing_node.cpp
```

## 通信机制

系统各模块之间通过ROS 2的话题和服务进行通信：

### 话题通信

| 话题名称 | 消息类型 | 发布者 | 订阅者 | 功能描述 |
|---------|---------|-------|-------|---------|
| /flight_control_command | communication/msg/FlightControlCommand | ground_station | drone_control | 飞行控制命令 |
| /robot_control_command | communication/msg/RobotControlCommand | ground_station | robot_control | 机器人控制命令 |
| /gps_info | communication/msg/GPSInfo | dji_psdk_wrapper | drone_control, ground_station | GPS信息 |
| /drone_state | communication/msg/DroneState | dji_psdk_wrapper | ground_station, safety_monitor | 无人机状态 |
| /robot_state | communication/msg/RobotState | robot_control | ground_station | 机器人状态 |

### 服务通信

| 服务名称 | 服务类型 | 提供者 | 调用者 | 功能描述 |
|---------|---------|-------|-------|---------|
| /hook_control_service | communication/srv/HookControl | drone_control | ground_station | 挂钩控制服务 |
| /drone_info_service | communication/srv/GetDroneInfo | dji_psdk_wrapper | ground_station | 无人机信息服务 |
| /robot_info_service | communication/srv/GetRobotInfo | robot_control | ground_station | 机器人信息服务 |

## 数据流程

### 1. 无人机控制流程

```
ground_station → /flight_control_command → drone_control → dji_psdk_wrapper → DJI 无人机
```

### 2. 机器人控制流程

```
ground_station → /robot_control_command → robot_control → 地面机器人
```

### 3. 数据采集流程

```
DJI 无人机 → dji_psdk_wrapper → /gps_info, /drone_state → ground_station, safety_monitor
传感器 → 相应模块 → 状态话题 → ground_station, safety_monitor
```

## 技术栈

| 技术/框架 | 用途 | 版本 |
|----------|------|------|
| ROS 2 | 机器人操作系统 | Jazzy |
| C++ | 主要开发语言 | C++17 |
| DJI PSDK | 无人机通信接口 | 3.14.0 |
| CMake | 构建系统 | 3.16+ |
| Colcon | ROS 2构建工具 | 最新版 |
| YAML | 配置文件格式 | 最新版 |

## 扩展接口

系统设计了良好的扩展接口，支持添加新的功能模块和硬件设备：

1. **硬件扩展**：通过扩展PSDK封装模块，可以支持新的无人机型号
2. **功能扩展**：通过添加新的功能模块，可以扩展系统的功能
3. **接口扩展**：通过在通信模块中定义新的消息和服务，可以扩展系统的通信接口

## 安全设计

系统采用多层次的安全设计，确保系统的安全运行：

1. **硬件安全**：采用DJI PSDK提供的硬件安全机制，确保与无人机的安全通信
2. **软件安全**：实现了命令验证、故障检测、自动应急处理等安全机制
3. **通信安全**：采用ROS 2的通信安全机制，确保模块之间通信的安全性
4. **监控安全**：安全监控模块实时监控系统状态，及时发现和处理安全隐患

## 部署架构

系统可以部署在多种硬件平台上，包括：

1. **地面站电脑**：运行地面站模块和安全监控模块
2. **无人机载计算机**：运行PSDK封装模块和无人机控制模块
3. **机器人控制箱**：运行机器人控制模块

各平台之间通过网络进行通信，实现了分布式部署和集中控制。