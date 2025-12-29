# FC30 Drone Gazebo Simulation

This package provides a Gazebo simulation environment for the FC30 drone, integrated with the ROS 2 PSDK wrapper.

## Features

- **FC30 Drone Model**: Detailed SDF model with realistic physical properties
- **Sensors**: Camera, IMU, and GPS with ROS 2 interfaces
- **Control System**: Basic PID controller for flight control
- **PSDK Integration**: Bridge node to connect with the existing PSDK system
- **Custom World**: Simple simulation environment with ground plane and lighting

## Installation

1. **Build the package**:
   ```bash
   cd /home/lcf/ros2_hoisting_system
   colcon build --packages-select fc30_gazebo
   ```

2. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launch Gazebo Simulation

```bash
ros2 launch fc30_gazebo fc30_gazebo.launch.py
```

This will:
- Start Gazebo with the FC30 world
- Spawn the FC30 drone model
- Start the FC30 controller node
- Start the PSDK bridge node

### Send Commands to the Drone

The drone accepts commands through the `/psdk_cmd` topic using the `communication/PSDKCmd` message type:

1. **Takeoff**:
   ```bash
   ros2 topic pub -1 /psdk_cmd communication/msg/PSDKCmd "{command_type: 1}"
   ```

2. **Land**:
   ```bash
   ros2 topic pub -1 /psdk_cmd communication/msg/PSDKCmd "{command_type: 2}"
   ```

3. **Move to Position**:
   ```bash
   ros2 topic pub -1 /psdk_cmd communication/msg/PSDKCmd "{command_type: 5, x: 2.0, y: 1.0, z: 2.0, yaw: 0.0}"
   ```

4. **Hover**:
   ```bash
   ros2 topic pub -1 /psdk_cmd communication/msg/PSDKCmd "{command_type: 3}"
   ```

### Monitor Drone Status

Drone state information is published to the `/psdk_data` topic:

```bash
ros2 topic echo /psdk_data
```

## Package Structure

```
fc30_gazebo/
├── models/
│   └── fc30/
│       ├── model.config      # Model configuration
│       └── model.sdf         # SDF model description
├── worlds/
│   └── fc30_world.world     # Gazebo world file
├── launch/
│   └── fc30_gazebo.launch.py # Launch file
├── src/
│   ├── fc30_controller.cpp   # Flight controller node
│   └── psdk_bridge.cpp       # PSDK bridge node
├── CMakeLists.txt            # Build configuration
└── package.xml               # Package metadata and dependencies
```

## Nodes

### fc30_controller

Controls the drone's flight in Gazebo based on PSDK commands.

- **Subscribes**:
  - `/psdk_cmd` (communication/PSDKCmd): PSDK commands
  - `/fc30/pose` (geometry_msgs/PoseStamped): Current drone pose
  - `/fc30/twist` (geometry_msgs/TwistStamped): Current drone twist
  - `/fc30/imu` (sensor_msgs/Imu): IMU data

- **Publishes**:
  - `/fc30/rotor_*/cmd_vel` (std_msgs/Float64): Rotor velocities

### psdk_bridge

Bridges communication between Gazebo and the PSDK system.

- **Subscribes**:
  - `/psdk_cmd` (communication/PSDKCmd): PSDK commands
  - `/fc30/imu` (sensor_msgs/Imu): IMU data
  - `/fc30/gps/fix` (sensor_msgs/NavSatFix): GPS data

- **Publishes**:
  - `/psdk_data` (communication/PSDKData): Drone state data for PSDK
  - `/fc30/pose` (geometry_msgs/PoseStamped): Target pose commands
  - `/fc30/twist` (geometry_msgs/TwistStamped): Target twist commands

## Customization

### Modify Drone Parameters

Edit the SDF model file at `models/fc30/model.sdf` to change:
- Physical properties (mass, inertia)
- Sensor parameters (camera resolution, IMU noise)
- Visual appearance

### Tune PID Controllers

Modify the PID gains in `src/fc30_controller.cpp` to improve flight performance:

```cpp
// PID gains (example values, need tuning)
kp_pos_ = {0.5, 0.5, 1.0};
ki_pos_ = {0.0, 0.0, 0.1};
kd_pos_ = {0.2, 0.2, 0.5};

kp_att_ = {2.0, 2.0, 1.0};
ki_att_ = {0.0, 0.0, 0.0};
kd_att_ = {0.5, 0.5, 0.2};
```

### Create New Worlds

Add new world files in the `worlds/` directory and update the launch file to use them.

## Integration with PSDK

The `psdk_bridge` node provides seamless integration with the existing PSDK system by:

1. Converting PSDK commands to Gazebo control commands
2. Converting Gazebo sensor data to PSDK data format
3. Maintaining the same topic names as the physical drone

This allows existing PSDK applications to work with the simulation without modification.

## Troubleshooting

### Gazebo Can't Find the Model

Make sure the Gazebo model path is set correctly:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/lcf/ros2_hoisting_system/install/fc30_gazebo/share/fc30_gazebo/models
```

### Nodes Fail to Start

Check that all dependencies are installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Poor Flight Performance

Tune the PID gains in `src/fc30_controller.cpp` for better flight characteristics.

## License

Apache License 2.0