# 安装指南

## 系统要求

- Ubuntu 22.04 LTS 或更高版本
- ROS 2 Jazzy 或更高版本
- DJI PSDK 3.14.0 或更高版本
- C++17 兼容编译器
- 至少 4GB RAM
- 至少 10GB 可用磁盘空间

## 依赖安装

### 1. 安装 ROS 2

请参考 [ROS 2 官方安装指南](https://docs.ros.org/en/jazzy/Installation.html) 安装 ROS 2 Jazzy。

### 2. 安装必要的依赖包

```bash
# 安装基础依赖
sudo apt-get update
sudo apt-get install -y git cmake build-essential python3-colcon-common-extensions

# 安装 ROS 2 依赖
sudo apt-get install -y ros-jazzy-rclcpp ros-jazzy-std-msgs ros-jazzy-sensor-msgs 
ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-tf2 ros-jazzy-tf2-ros

# 安装 DJI PSDK 依赖
sudo apt-get install -y libusb-1.0-0-dev libyaml-cpp-dev
```

### 3. 安装 DJI PSDK

1. 从 [DJI 开发者网站](https://developer.dji.com/payload-sdk/) 下载 PSDK 3.14.0
2. 解压到项目的 third_party 目录：

```bash
mkdir -p third_party
tar -xzvf DJI_PSDK_3.14.0.tar.gz -C third_party/
mv third_party/DJI_PSDK_3.14.0 third_party/dji_psdk
```

## 项目编译

1. 克隆项目到本地：

```bash
git clone <项目仓库地址>
cd ros2_hoisting_system
```

2. 编译项目：

```bash
# 编译整个项目
colcon build --symlink-install

# 或者编译特定包
colcon build --symlink-install --packages-select <包名>
```

3. 激活环境：

```bash
source install/setup.bash
```

## 验证安装

编译完成后，可以通过运行以下命令验证安装：

```bash
# 检查是否可以找到包
ros2 pkg list | grep dji_psdk_wrapper
```

如果输出中包含 `dji_psdk_wrapper`，则表示安装成功。