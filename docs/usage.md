# 使用指南

## 环境激活

在使用系统前，需要先激活ROS 2环境：

```bash
# 激活ROS 2系统环境
source /opt/ros/jazzy/setup.bash

# 激活项目环境（相对路径）
source install/setup.bash
```

## 一键启动所有服务

系统提供了一键启动脚本，可以方便地启动所有必要服务：

```bash
# 运行一键启动脚本
./start_all.sh
```

该脚本将启动以下服务：
- HTTP服务（用于地面站Web界面）
- rosbridge服务（WebSocket通信）
- 无人机节点（dji_psdk_node）
- 地面站节点（ground_station_node）

## 一键停止所有服务

系统提供了一键停止脚本，可以方便地停止所有服务：

```bash
# 运行一键停止脚本
./stop_all.sh
```

## 单独运行节点

如果需要单独运行某个节点，可以使用以下命令：

### 1. 运行DJI PSDK节点

DJI PSDK节点是系统的核心节点，负责与DJI无人机通信：

```bash
# 运行PSDK节点
ros2 run dji_psdk_wrapper dji_psdk_node
```

### 2. 运行无人机控制节点

无人机控制节点负责发送飞行命令和控制无人机：

```bash
# 运行无人机控制节点
ros2 run drone_control drone_controller
```

### 3. 运行机器人控制节点

机器人控制节点负责控制地面机器人设备：

```bash
# 运行机器人控制节点
ros2 run robot_control robot_controller
```

### 4. 运行地面站节点

地面站节点提供用户界面和监控功能：

```bash
# 运行地面站节点
ros2 run ground_station ground_station_node
```

## 话题通信

### 发布话题

#### 1. 发布飞行控制命令

```bash
# 发布起飞命令
ros2 topic pub /flight_control_command communication/msg/FlightControlCommand "{command_type: 'takeoff'}"

# 发布着陆命令
ros2 topic pub /flight_control_command communication/msg/FlightControlCommand "{command_type: 'land'}"
```

#### 2. 发布机器人控制命令

```bash
# 发布机器人移动命令
ros2 topic pub /robot_control_command communication/msg/RobotControlCommand "{command_type: 'move', x: 1.0, y: 0.0, z: 0.0}"
```

### 订阅话题

#### 1. 订阅GPS信息

```bash
# 订阅GPS话题
ros2 topic echo /gps_info
```

#### 2. 订阅无人机状态

```bash
# 订阅无人机状态话题
ros2 topic echo /drone_state
```

## 服务调用

### 1. 调用挂钩控制服务

```bash
# 调用挂钩控制服务（打开挂钩）
ros2 service call /hook_control_service communication/srv/HookControl "{command: 'open'}"

# 调用挂钩控制服务（关闭挂钩）
ros2 service call /hook_control_service communication/srv/HookControl "{command: 'close'}"
```

### 2. 调用无人机信息服务

```bash
# 调用无人机信息服务
ros2 service call /drone_info_service communication/srv/GetDroneInfo "{}"
```

## 参数配置

### 1. 查看节点参数

```bash
# 查看PSDK节点参数
ros2 param list /psdk_node
```

### 2. 设置节点参数

```bash
# 设置PSDK节点的日志级别
ros2 param set /psdk_node log_level 2
```

## 常见问题

### Q: 节点无法启动

**A:** 请检查以下几点：
1. 是否正确激活了ROS 2环境
2. 是否正确编译了项目
3. 是否安装了所有依赖
4. 是否有足够的权限

### Q: 无法与无人机通信

**A:** 请检查以下几点：
1. 无人机是否已经连接到Payload设备
2. PSDK配置文件是否正确
3. 是否有足够的权限访问USB设备

### Q: 话题没有数据

**A:** 请检查以下几点：
1. 节点是否已经正确启动
2. 是否有其他节点在发布该话题
3. 话题名称是否正确

## 日志查看

可以通过以下命令查看节点的日志：

```bash
# 查看PSDK节点的日志
ros2 node info /psdk_node
```

或者使用rqt工具查看更详细的日志：

```bash
# 启动rqt日志查看工具
rqt_console
```