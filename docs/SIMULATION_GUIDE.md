# ROS 2 吊升系统仿真飞行与调试指南

## 1. 系统概述

本工程实现了一个基于ROS 2的无人机吊升系统，包含以下核心组件：

- **Gazebo仿真环境**：提供物理引擎和3D可视化
- **状态估计节点**：融合GPS和IMU数据，提供精确的位姿估计
- **控制器节点**：实现PID控制算法，控制无人机轨迹跟踪
- **轨迹规划节点**：生成平滑的飞行轨迹
- **PSDK桥接节点**：连接仿真系统与DJI PSDK API

## 2. 开始仿真飞行

### 2.1 启动完整系统

首先，确保所有包已成功构建：

```bash
colcon build --packages-select fc30_gazebo dji_psdk_wrapper communication
```

然后，启动整个系统：

```bash
source install/setup.bash
ros2 launch fc30_gazebo fc30_gazebo.launch.py
```

这将启动：
- Gazebo仿真环境（包含增强的测试场景）
- 状态估计节点
- 控制器节点
- 轨迹规划节点
- PSDK桥接节点

### 2.2 发送航点指令

系统启动后，可以通过以下命令发送航点指令，让无人机执行轨迹：

```bash
# 发送第一个航点（起飞）
ros2 topic pub -1 /fc30/waypoint geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 2.0}"

# 发送第二个航点（移动到指定位置）
ros2 topic pub -1 /fc30/waypoint geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 2.0}"

# 发送第三个航点（移动到着陆平台）
ros2 topic pub -1 /fc30/waypoint geometry_msgs/msg/Point "{x: 0.0, y: 3.0, z: 1.0}"
```

### 2.3 手动控制（可选）

如果需要手动控制无人机，可以使用ROS 2的命令行工具发布目标姿态：

```bash
ros2 topic pub -r 10 /fc30/target_pose geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
  pose: {
    position: {x: 0.0, y: 0.0, z: 2.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## 3. ROS 2 运行调试

### 3.1 查看节点和话题

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看特定话题的消息类型
ros2 topic info /fc30/pose_estimate
```

### 3.2 监听话题数据

```bash
# 监听状态估计结果
ros2 topic echo /fc30/pose_estimate

# 监听无人机速度指令
ros2 topic echo /fc30/rotor_velocities

# 监听控制器输出
ros2 topic echo /fc30/target_pose
```

### 3.3 可视化轨迹

可以使用RViz2可视化轨迹和无人机状态：

```bash
ros2 run rviz2 rviz2
```

在RViz2中：
1. 设置Fixed Frame为"world"
2. 添加Marker显示，选择话题"/fc30/trajectory_marker"
3. 添加Pose显示，选择话题"/fc30/pose_estimate"

### 3.4 查看节点日志

```bash
# 查看所有节点的日志
ros2 logs

# 查看特定节点的日志
ros2 logs --node state_estimator
ros2 logs --node fc30_controller
ros2 logs --node trajectory_planner
```

### 3.5 调试参数

可以使用ROS 2的参数系统调整控制参数：

```bash
# 列出节点的参数
ros2 param list /fc30_controller

# 获取参数值
ros2 param get /fc30_controller kp_pos_x

# 设置参数值
ros2 param set /fc30_controller kp_pos_x 0.8
```

## 4. 测试场景说明

增强的仿真环境包含以下测试场景：

### 4.1 障碍物赛道
- 红色立柱：位于 (2.0, 0.0, 1.0)
- 绿色立柱：位于 (4.0, 1.0, 1.0)
- 蓝色立柱：位于 (6.0, -1.0, 1.0)

### 4.2 着陆平台
- 黄色平台：位于 (0.0, 3.0, 0.3)，尺寸 1.0x1.0x0.2
- 紫色平台：位于 (5.0, 3.0, 1.0)，尺寸 0.8x0.8x0.2

### 4.3 目标物体
- 青色盒子：位于 (3.0, 5.0, 0.5)，尺寸 0.5x0.5x0.5，可用于抓取任务测试

## 5. 常见问题排查

### 5.1 构建失败

如果构建失败，检查以下几点：

```bash
# 确保所有依赖已安装
sudo apt install ros-humble-ros-gz-sim ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-tf2 ros-humble-tf2-ros libeigen3-dev

# 清理构建目录并重新构建
rm -rf build/ install/ log/
colcon build --packages-select fc30_gazebo dji_psdk_wrapper communication
```

### 5.2 节点启动失败

检查节点日志，查看具体错误信息：

```bash
ros2 logs --node <节点名称>
```

### 5.3 无人机无响应

1. 检查状态估计节点是否正常工作：
   ```bash
   ros2 topic echo /fc30/pose_estimate
   ```

2. 检查控制器是否接收到目标指令：
   ```bash
   ros2 topic echo /fc30/target_pose
   ```

3. 检查旋翼速度指令是否发布：
   ```bash
   ros2 topic echo /fc30/rotor_velocities
   ```

### 5.4 轨迹跟踪不稳定

调整PID参数以提高稳定性：

```bash
# 增加比例增益提高响应速度
ros2 param set /fc30_controller kp_pos_x 1.0
ros2 param set /fc30_controller kp_pos_y 1.0
ros2 param set /fc30_controller kp_pos_z 1.5

# 增加阻尼增益减少震荡
ros2 param set /fc30_controller kd_pos_x 0.5
ros2 param set /fc30_controller kd_pos_y 0.5
ros2 param set /fc30_controller kd_pos_z 0.7
```

## 6. 扩展功能

### 6.1 添加更多航点

可以编写一个简单的脚本发送多个航点：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.publisher_ = self.create_publisher(Point, '/fc30/waypoint', 10)
        self.send_waypoints()

    def send_waypoints(self):
        waypoints = [
            (0.0, 0.0, 2.0),   # 起飞
            (2.0, 0.0, 2.0),   # 第一个点
            (2.0, 2.0, 2.0),   # 第二个点
            (0.0, 2.0, 2.0),   # 第三个点
            (0.0, 0.0, 2.0),   # 返回起点
            (0.0, 0.0, 0.5)    # 降落
        ]

        for wp in waypoints:
            point = Point()
            point.x, point.y, point.z = wp
            self.publisher_.publish(point)
            print(f"Sent waypoint: {wp}")
            rclpy.sleep(1.0)  # 等待1秒后发送下一个航点

def main(args=None):
    rclpy.init(args=args)
    waypoint_sender = WaypointSender()
    rclpy.spin_once(waypoint_sender)
    waypoint_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.2 使用增强的仿真环境

默认使用的是增强的测试环境。如果需要切换回简单环境，可以修改启动文件：

```bash
# 编辑启动文件
nano src/fc30_gazebo/launch/fc30_gazebo.launch.py
```

将`world`参数从`fc30_enhanced_world.world`改为`fc30_world.world`。

或者直接使用相对路径编辑：
```bash
# 编辑启动文件
nano src/fc30_gazebo/launch/fc30_gazebo.launch.py
```

## 7. 性能优化

### 7.1 提高仿真速度

如果仿真运行缓慢，可以尝试以下优化：

1. 降低Gazebo的图形质量
2. 减少物理更新频率：
   ```bash
   ros2 param set /gazebo physics_engine/default_physics/real_time_update_rate 500.0
   ```
3. 关闭不必要的日志输出：
   ```bash
   ros2 run rclpy_guardian rclpy_guardian --ros-args --log-level ERROR
   ```

### 7.2 提高估计精度

可以调整状态估计节点的滤波参数：

```bash
# 调整GPS低通滤波系数
ros2 param set /state_estimator gps_lowpass_alpha 0.1

# 调整IMU低通滤波系数
ros2 param set /state_estimator imu_lowpass_alpha 0.2
```

## 8. 总结

本工程提供了一个功能完整的ROS 2无人机吊升系统仿真平台，包含：

- ✅ 精确的状态估计
- ✅ 稳定的PID控制
- ✅ 平滑的轨迹规划
- ✅ 丰富的测试场景
- ✅ 完善的调试工具

通过本指南，您可以快速上手仿真飞行，并进行系统调试和性能优化。如果遇到问题，请参考常见问题排查部分，或查看节点日志获取详细信息。