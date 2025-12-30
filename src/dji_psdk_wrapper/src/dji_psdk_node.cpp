
#include "dji_psdk_wrapper/psdk_wrapper.hpp"
#include "dji_psdk_wrapper/logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <communication/msg/system_state.hpp>
#include <communication/msg/battery_info.hpp>
#include <communication/msg/psdk_cmd.hpp>
#include <communication/srv/update_flight_control_config.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <memory>
#include <string>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace dji_psdk_wrapper
{

/**
 * @brief PSDK节点类，用于封装DJI PSDK接口并与ROS 2集成
 * 
 * 该类继承自rclcpp::Node，实现了PSDK与ROS 2的交互，包括：
 * - 参数声明与获取
 * - 发布者和订阅者的初始化
 * - PSDK回调函数的注册
 * - 无人机状态发布
 * - 飞行数据处理
 */
class PSDKNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化PSDK节点
   * 
   * 初始化节点参数、发布者、订阅者、定时器，并注册PSDK回调函数。
   * 如果auto_start参数为true，则自动初始化并启动PSDK。
   */
  PSDKNode() : Node("dji_psdk_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("app_id", "YOUR_APP_ID");
    this->declare_parameter<std::string>("app_key", "YOUR_APP_KEY");
    this->declare_parameter<std::string>("app_license", "YOUR_APP_LICENSE");
    this->declare_parameter<std::string>("developer_account", "YOUR_DEVELOPER_ACCOUNT");
    this->declare_parameter<int>("baud_rate", 460800); // 与SDK层面的默认波特率保持一致
    this->declare_parameter<bool>("auto_start", true);

    // 获取参数
    app_id_ = this->get_parameter("app_id").as_string();
    app_key_ = this->get_parameter("app_key").as_string();
    app_license_ = this->get_parameter("app_license").as_string();
    developer_account_ = this->get_parameter("developer_account").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    auto_start_ = this->get_parameter("auto_start").as_bool();

    // 初始化状态变量
    memset(&current_vehicle_state_, 0, sizeof(T_DjiVehicleState));
    memset(&current_gps_data_, 0, sizeof(T_DjiGpsData));
    memset(&current_imu_data_, 0, sizeof(T_DjiImuData));
    memset(&current_battery_info_, 0, sizeof(T_DjiBatteryInfo));
    memset(&current_multi_battery_info_, 0, sizeof(T_DjiMultiBatteryInfo));
    
    // 初始化IMU四元数为单位四元数
    current_imu_data_.quaternion[0] = 1.0;
    current_imu_data_.quaternion[1] = 0.0;
    current_imu_data_.quaternion[2] = 0.0;
    current_imu_data_.quaternion[3] = 0.0;

    // 初始化PSDK包装器
    psdk_wrapper_ = std::make_shared<PSDKWrapper>();
    
    // 设置波特率
    psdk_wrapper_->setBaudRate(baud_rate_);

    // 初始化发布者
    system_state_pub_ = this->create_publisher<communication::msg::SystemState>("drone/system_state", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("drone/imu", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("drone/gps", 10);
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odometry", 10);


    // 初始化订阅者
    psdk_cmd_sub_ = this->create_subscription<communication::msg::PSDKCmd>(
      "/drone/psdk_cmd", 10, std::bind(&PSDKNode::psdk_cmd_callback, this, std::placeholders::_1));

    // 初始化定时器
    state_publish_timer_ = this->create_wall_timer(
      100ms, std::bind(&PSDKNode::publish_system_state, this));

    // 注册PSDK回调
    psdk_wrapper_->registerStateCallback(std::bind(&PSDKNode::handle_vehicle_state, this, std::placeholders::_1));
    psdk_wrapper_->registerGpsCallback(std::bind(&PSDKNode::handle_gps_data, this, std::placeholders::_1));
    psdk_wrapper_->registerImuCallback(std::bind(&PSDKNode::handle_imu_data, this, std::placeholders::_1));
    
    // 创建开启电机服务
    turn_on_motors_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "drone/turn_on_motors", std::bind(&PSDKNode::turn_on_motors_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建关闭电机服务
    turn_off_motors_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "drone/turn_off_motors", std::bind(&PSDKNode::turn_off_motors_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建更新飞行控制配置服务
    update_flight_control_config_srv_ = this->create_service<communication::srv::UpdateFlightControlConfig>(
      "drone/update_flight_control_config", std::bind(&PSDKNode::update_flight_control_config_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 如果设置为自动启动，则初始化并启动PSDK
    if (auto_start_)
    {
      initialize_psdk();
    }
  
    RCLCPP_INFO(this->get_logger(), "PSDK Node initialized");
  }

  /**
   * @brief 析构函数，关闭PSDK并释放资源
   * 
   * 停止PSDK包装器，释放相关资源，并记录日志。
   */
  ~PSDKNode()
  {
    if (psdk_wrapper_)
    {
      psdk_wrapper_->stop();
    }
    RCLCPP_INFO(this->get_logger(), "PSDK Node shutdown");
  }

private:
  // PSDK包装器
  std::shared_ptr<PSDKWrapper> psdk_wrapper_;

  // 参数
  std::string app_id_;               // DJI开发者账号
  std::string app_key_;              // 应用密钥
  std::string app_license_;          // 应用许可证
  std::string developer_account_;    // 开发者账号
  int baud_rate_;                    // 波特率
  bool auto_start_;                  // 是否自动启动PSDK

  // 发布者
  rclcpp::Publisher<communication::msg::SystemState>::SharedPtr system_state_pub_;  // 系统状态发布者
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;                    // IMU数据发布者
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;              // GPS数据发布者
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;            // 里程计数据发布者


  // 订阅者
  rclcpp::Subscription<communication::msg::PSDKCmd>::SharedPtr psdk_cmd_sub_;  // PSDK命令订阅者
  
  // 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_on_motors_srv_;  // 启动电机服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn_off_motors_srv_;  // 关闭电机服务
  rclcpp::Service<communication::srv::UpdateFlightControlConfig>::SharedPtr update_flight_control_config_srv_;  // 更新飞行控制配置服务

  // 定时器
  rclcpp::TimerBase::SharedPtr state_publish_timer_;  // 系统状态发布定时器

  // 状态变量
  T_DjiVehicleState current_vehicle_state_;    // 当前车辆状态
  T_DjiGpsData current_gps_data_;              // 当前GPS数据
  T_DjiImuData current_imu_data_;              // 当前IMU数据
  T_DjiBatteryInfo current_battery_info_;      // 当前电池信息
  T_DjiMultiBatteryInfo current_multi_battery_info_;  // 当前多电池信息

  /**
   * @brief 初始化PSDK
   * 
   * 调用PSDK包装器的initialize和start方法，初始化并启动PSDK。
   * 记录初始化和启动过程的日志信息。
   */
  void initialize_psdk()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing PSDK...");
    
    if (psdk_wrapper_->initialize())
    {
      RCLCPP_INFO(this->get_logger(), "PSDK initialized successfully");
      
      if (psdk_wrapper_->start())
      {
        RCLCPP_INFO(this->get_logger(), "PSDK started successfully");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start PSDK");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize PSDK");
    }
  }

  /**
   * @brief 发布系统状态
   * 
   * 定期发布无人机的系统状态，包括飞行模式、状态、电池电量等信息。
   * 从PSDK获取车辆状态和电池信息，组装成SystemState消息并发布。
   */
  void publish_system_state()
  {
    auto state_msg = communication::msg::SystemState();
    state_msg.header.stamp = this->now();
    state_msg.system_name = "Drone";
    
    // 使用新的getConnectionStatus方法检测连接状态
    bool is_connected = psdk_wrapper_->getConnectionStatus();
    
    if (is_connected)
    {
      // 获取无人机状态
      T_DjiVehicleState vehicle_state;
      if (psdk_wrapper_->getDroneState(&vehicle_state) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
      {
        // 设置飞行模式
        switch (vehicle_state.flightMode)
        {
          case DJI_FLIGHT_MODE_MANUAL:
            state_msg.mode = "manual";
            break;
          case DJI_FLIGHT_MODE_MISSION:
            state_msg.mode = "mission";
            break;
          case DJI_FLIGHT_MODE_ATTITUDE:
            state_msg.mode = "attitude";
            break;
          case DJI_FLIGHT_MODE_GPS:
            state_msg.mode = "gps";
            break;
          case DJI_FLIGHT_MODE_ATTI_GPS:
            state_msg.mode = "atti_gps";
            break;
          case DJI_FLIGHT_MODE_SPORT:
            state_msg.mode = "sport";
            break;
          case DJI_FLIGHT_MODE_WAYPOINT:
            state_msg.mode = "waypoint";
            break;
          case DJI_FLIGHT_MODE_HOTPOINT:
            state_msg.mode = "hotpoint";
            break;
          case DJI_FLIGHT_MODE_FOLLOW_ME:
            state_msg.mode = "follow_me";
            break;
          case DJI_FLIGHT_MODE_CALIBRATION:
            state_msg.mode = "calibration";
            break;
          default:
            state_msg.mode = "unknown";
            break;
        }
        
        // 设置状态
        if (vehicle_state.isMotorOn)
        {
          state_msg.status = "flying";
        }
        else
        {
          state_msg.status = "landed";
        }
        
        // 添加调试信息：检查getDroneState调用后的状态
        RCLCPP_DEBUG(this->get_logger(), "publish_system_state: getDroneState succeeded");
        RCLCPP_DEBUG(this->get_logger(), "  flightMode: %d, isMotorOn: %s, isFlying: %s", 
                     vehicle_state.flightMode, 
                     vehicle_state.isMotorOn ? "true" : "false",
                     vehicle_state.isFlying ? "true" : "false");
      }
      else
      {
        // 连接状态为true但getDroneState失败，可能是临时错误
        RCLCPP_DEBUG(this->get_logger(), "publish_system_state: getDroneState failed but connection is still active");
        // 保持上次的状态或设置为unknown
        state_msg.mode = "unknown";
        state_msg.status = "unknown";
      }
    }
    else
    {
      state_msg.mode = "disconnected";
      state_msg.status = "disconnected";
      
      // 添加调试信息：串口连接已断开
      RCLCPP_DEBUG(this->get_logger(), "publish_system_state: serial port disconnected");
    }
    
    // 获取电池信息
    T_DjiBatteryInfo battery_info;
    if (psdk_wrapper_->getBatteryInfo(&battery_info) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      // 更新主电池信息到battery1
      state_msg.battery1.available = true;
      state_msg.battery1.voltage = static_cast<double>(battery_info.voltage) / 1000.0; // 转换为V
      state_msg.battery1.current = static_cast<double>(battery_info.current) / 1000.0; // 转换为A
      state_msg.battery1.percentage = battery_info.percentage;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "获取电池信息失败");
    }
    
    // 获取多电池信息（两块单体电池）
    T_DjiMultiBatteryInfo multi_battery_info;
    static std::chrono::steady_clock::time_point last_battery_log_time;
    bool need_log_battery = false;
    
    // 检查是否需要打印电池信息（每秒一次）
    if (std::chrono::steady_clock::now() - last_battery_log_time > std::chrono::seconds(1)) {
      need_log_battery = true;
      last_battery_log_time = std::chrono::steady_clock::now();
    }
    
    // 初始化电池信息，默认设为100%电量
    state_msg.battery1.header = state_msg.header;
    state_msg.battery1.battery_id = 1;
    state_msg.battery1.available = false;
    state_msg.battery1.voltage = 0.0;
    state_msg.battery1.current = 0.0;
    state_msg.battery1.percentage = 100.0; // 默认设为100%
    state_msg.battery1.temperature = 0.0;
    
    state_msg.battery2.header = state_msg.header;
    state_msg.battery2.battery_id = 2;
    state_msg.battery2.available = false;
    state_msg.battery2.voltage = 0.0;
    state_msg.battery2.current = 0.0;
    state_msg.battery2.percentage = 100.0; // 默认设为100%
    state_msg.battery2.temperature = 0.0;
    
    if (psdk_wrapper_->getMultiBatteryInfo(&multi_battery_info) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      // 设置电池1信息
      state_msg.battery1.available = multi_battery_info.battery1Available;
      if (multi_battery_info.battery1Available) {
        state_msg.battery1.voltage = multi_battery_info.battery1.voltage;
        state_msg.battery1.current = multi_battery_info.battery1.current;
        state_msg.battery1.percentage = multi_battery_info.battery1.percentage;
        state_msg.battery1.temperature = multi_battery_info.battery1.temperature;
        
        if (need_log_battery) {
          PSDK_INFO("dji_psdk_node", "电池1信息[真实]: ID=%d, 电压=%.2fV, 电流=%.2fA, 电量=%d%%, 温度=%.1f°C", 
                    multi_battery_info.battery1.batteryId, 
                    multi_battery_info.battery1.voltage, 
                    multi_battery_info.battery1.current, 
                    multi_battery_info.battery1.percentage, 
                    multi_battery_info.battery1.temperature);
        }
      } else {
        if (need_log_battery) {
          PSDK_INFO("dji_psdk_node", "电池1信息: 不可用");
        }
      }
      
      // 设置电池2信息
      state_msg.battery2.available = multi_battery_info.battery2Available;
      if (multi_battery_info.battery2Available) {
        state_msg.battery2.voltage = multi_battery_info.battery2.voltage;
        state_msg.battery2.current = multi_battery_info.battery2.current;
        state_msg.battery2.percentage = multi_battery_info.battery2.percentage;
        state_msg.battery2.temperature = multi_battery_info.battery2.temperature;
        
        if (need_log_battery) {
          PSDK_INFO("dji_psdk_node", "电池2信息[真实]: ID=%d, 电压=%.2fV, 电流=%.2fA, 电量=%d%%, 温度=%.1f°C", 
                    multi_battery_info.battery2.batteryId, 
                    multi_battery_info.battery2.voltage, 
                    multi_battery_info.battery2.current, 
                    multi_battery_info.battery2.percentage, 
                    multi_battery_info.battery2.temperature);
        }
      } else {
        if (need_log_battery) {
          PSDK_INFO("dji_psdk_node", "电池2信息: 不可用");
        }
      }
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "获取多电池信息失败");
    }
    
    state_msg.emergency_stop = false;
    state_msg.error_message = "";
    
    system_state_pub_->publish(state_msg);
  }

  /**
   * @brief 处理PSDK命令
   * 
   * 接收PSDK命令消息，处理Waypoint V3相关的命令操作。
   * 
   * @param msg PSDK命令消息，包含命令类型和参数
   */
  void psdk_cmd_callback(const communication::msg::PSDKCmd::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received PSDK command: type=%d", msg->command_type);
    
    T_DjiReturnCode result = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    
    // 处理不同类型的命令
    switch (msg->command_type)
    {
      case communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_UPLOAD:
        RCLCPP_INFO(this->get_logger(), "Uploading Waypoint V3 file: %s", msg->waypoint_file_path.c_str());
        result = psdk_wrapper_->uploadWaypointV3File(msg->waypoint_file_path);
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Waypoint V3 file uploaded successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to upload Waypoint V3 file, error code: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_START:
        RCLCPP_INFO(this->get_logger(), "Starting Waypoint V3 mission");
        result = psdk_wrapper_->executeWaypointV3Action(DJI_WAYPOINT_V3_ACTION_START);
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Waypoint V3 mission started successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to start Waypoint V3 mission, error code: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_STOP:
        RCLCPP_INFO(this->get_logger(), "Stopping Waypoint V3 mission");
        result = psdk_wrapper_->executeWaypointV3Action(DJI_WAYPOINT_V3_ACTION_STOP);
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Waypoint V3 mission stopped successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to stop Waypoint V3 mission, error code: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_PAUSE:
        RCLCPP_INFO(this->get_logger(), "Pausing Waypoint V3 mission");
        result = psdk_wrapper_->executeWaypointV3Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Waypoint V3 mission paused successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to pause Waypoint V3 mission, error code: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_RESUME:
        RCLCPP_INFO(this->get_logger(), "Resuming Waypoint V3 mission");
        result = psdk_wrapper_->executeWaypointV3Action(DJI_WAYPOINT_V3_ACTION_RESUME);
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Waypoint V3 mission resumed successfully");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to resume Waypoint V3 mission, error code: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_TAKEOFF:
        RCLCPP_INFO(this->get_logger(), "执行起飞命令，目标高度: %.2f米", msg->takeoff_height);
        
        // 调用PSDK的启动电机API（DJI PSDK中启动电机通常是起飞的第一步）
        result = psdk_wrapper_->turnOnMotors();
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "电机启动成功，无人机开始起飞");
        } else {
          RCLCPP_ERROR(this->get_logger(), "电机启动失败，错误码: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_LAND:
        RCLCPP_INFO(this->get_logger(), "执行降落命令，降落速度: %.2f米/秒", msg->landing_speed);
        
        // 调用PSDK的关闭电机API（DJI PSDK中关闭电机通常会触发降落）
        result = psdk_wrapper_->turnOffMotors();
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "降落命令执行成功");
        } else {
          RCLCPP_ERROR(this->get_logger(), "降落命令执行失败，错误码: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_EMERGENCY_STOP:
        RCLCPP_INFO(this->get_logger(), "执行紧急停止命令");
        
        // 调用PSDK的紧急停飞API
        result = psdk_wrapper_->emergencyStop();
        if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "紧急停止执行成功");
        } else {
          RCLCPP_ERROR(this->get_logger(), "紧急停止执行失败，错误码: 0x%08lX", result);
        }
        break;
        
      case communication::msg::PSDKCmd::CMD_TYPE_HOOK_CONTROL:
        RCLCPP_INFO(this->get_logger(), "Received hook command action type: %d", msg->hook_action);
        
        // 目前PSDKWrapper类中没有controlHook函数，这里只是打印日志
        RCLCPP_INFO(this->get_logger(), "Hook command received but not implemented: action_type=%d", msg->hook_action);
        break;
        
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown PSDK command type: %d", msg->command_type);
        break;
    }
  }

  /**
   * @brief 处理车辆状态
   * 
   * 接收车辆状态更新，更新当前车辆状态，并发布里程计数据。
   * 
   * @param state 车辆状态数据，包含飞行模式、电机状态等信息
   */
  void handle_vehicle_state(const T_DjiVehicleState &state)
  {
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: Entering function");
    
    current_vehicle_state_ = state;
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: current_vehicle_state_ updated");
    
    // 发布里程计数据
    auto odometry_msg = nav_msgs::msg::Odometry();
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: odometry_msg created");
    
    odometry_msg.header.stamp = this->now();
    odometry_msg.header.frame_id = "map";
    odometry_msg.child_frame_id = "base_link";
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: odometry_msg header set");
    
    // 设置位置（需要从GPS数据转换）
    odometry_msg.pose.pose.position.x = 0.0;  // 需要从GPS数据转换
    odometry_msg.pose.pose.position.y = 0.0;  // 需要从GPS数据转换
    odometry_msg.pose.pose.position.z = 0.0;  // 需要从高度数据转换
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: position set");
    
    // 设置姿态（需要从IMU数据转换）
    // 使用IMU数据，因为current_imu_data_已经被初始化为单位四元数
    odometry_msg.pose.pose.orientation.x = current_imu_data_.quaternion[0];
    odometry_msg.pose.pose.orientation.y = current_imu_data_.quaternion[1];
    odometry_msg.pose.pose.orientation.z = current_imu_data_.quaternion[2];
    odometry_msg.pose.pose.orientation.w = current_imu_data_.quaternion[3];
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: orientation set");
    
    // 设置线速度（需要从速度数据转换）
    odometry_msg.twist.twist.linear.x = 0.0;  // 需要从速度数据转换
    odometry_msg.twist.twist.linear.y = 0.0;  // 需要从速度数据转换
    odometry_msg.twist.twist.linear.z = 0.0;  // 需要从速度数据转换
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: linear velocity set");
    
    // 设置角速度（从IMU数据转换）
    // 使用IMU数据，因为current_imu_data_.gyroscope已经被初始化为零
    odometry_msg.twist.twist.angular.x = current_imu_data_.gyroscope[0];
    odometry_msg.twist.twist.angular.y = current_imu_data_.gyroscope[1];
    odometry_msg.twist.twist.angular.z = current_imu_data_.gyroscope[2];
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: angular velocity set");
    
    // 检查odometry_pub_是否为空
    if (!odometry_pub_) {
      RCLCPP_ERROR(this->get_logger(), "handle_vehicle_state: odometry_pub_ is nullptr!");
      return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: Before publishing odometry");
    
    try {
      odometry_pub_->publish(odometry_msg);
      RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: odometry published successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "handle_vehicle_state: Exception during publish: %s", e.what());
    }
    
    RCLCPP_DEBUG(this->get_logger(), "handle_vehicle_state: Function completed");
  }

  /**
   * @brief 处理GPS数据
   * 
   * 接收GPS数据更新，更新当前GPS数据，并发布GPS消息。
   * 
   * @param data GPS数据，包含经纬度、高度和精度信息
   */
  void handle_gps_data(const T_DjiGpsData &data)
  {
    current_gps_data_ = data;
    
    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = this->now();
    gps_msg.header.frame_id = "gps";
    
    gps_msg.latitude = data.latitude;
    gps_msg.longitude = data.longitude;
    gps_msg.altitude = data.altitude;
    
    // 设置精度
    gps_msg.position_covariance[0] = data.positionStandardDeviation[0] * data.positionStandardDeviation[0];
    gps_msg.position_covariance[4] = data.positionStandardDeviation[1] * data.positionStandardDeviation[1];
    gps_msg.position_covariance[8] = data.positionStandardDeviation[2] * data.positionStandardDeviation[2];
    
    // 设置状态
    if (data.positionType == DJI_GPS_POSITION_TYPE_FIX)
    {
      gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    else if (data.positionType == DJI_GPS_POSITION_TYPE_SINGLE)
    {
      gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    }
    else
    {
      gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    
    gps_pub_->publish(gps_msg);
  }

  /**
   * @brief 处理IMU数据
   * 
   * 接收IMU数据更新，更新当前IMU数据，并发布IMU消息。
   * 
   * @param data IMU数据，包含四元数、角速度和线加速度信息
   */
  void handle_imu_data(const T_DjiImuData &data)
  {
    current_imu_data_ = data;
    
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu";
    
    // 设置方向
    imu_msg.orientation.x = data.quaternion[0];
    imu_msg.orientation.y = data.quaternion[1];
    imu_msg.orientation.z = data.quaternion[2];
    imu_msg.orientation.w = data.quaternion[3];
    
    // 设置角速度
    imu_msg.angular_velocity.x = data.gyroscope[0];
    imu_msg.angular_velocity.y = data.gyroscope[1];
    imu_msg.angular_velocity.z = data.gyroscope[2];
    
    // 设置线加速度
    imu_msg.linear_acceleration.x = data.accelerometer[0];
    imu_msg.linear_acceleration.y = data.accelerometer[1];
    imu_msg.linear_acceleration.z = data.accelerometer[2];
    
    // 设置协方差
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
    
    imu_pub_->publish(imu_msg);
  }
  
  /**
   * @brief 处理启动电机服务请求
   * 
   * 接收来自其他节点的启动电机请求，调用PSDKWrapper的turnOnMotors方法，并返回响应结果。
   * 
   * @param request 服务请求（无参数）
   * @param response 服务响应，包含成功标志和消息
   */
  void turn_on_motors_callback(const std_srvs::srv::Trigger::Request::SharedPtr /* request */, 
                               std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    RCLCPP_INFO(this->get_logger(), "Received turn on motors request");
    
    if (!psdk_wrapper_) {
      RCLCPP_ERROR(this->get_logger(), "PSDKWrapper not initialized");
      response->success = false;
      response->message = "PSDKWrapper not initialized";
      return;
    }
    
    T_DjiReturnCode result = psdk_wrapper_->turnOnMotors();
    
    if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Turn on motors command sent successfully");
      response->success = true;
      response->message = "Turn on motors command sent successfully";
    } else if (result == DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND) {
      RCLCPP_ERROR(this->get_logger(), "Failed to turn on motors: Not connected to drone");
      response->success = false;
      response->message = "Not connected to drone";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to turn on motors: Error code %ld", result);
      response->success = false;
      response->message = "Failed to turn on motors: Error code " + std::to_string(result);
    }
  }
  
  /**
   * @brief 处理关闭电机服务请求
   * 
   * 接收来自其他节点的关闭电机请求，调用PSDKWrapper的turnOffMotors方法，并返回响应结果。
   * 
   * @param request 服务请求（无参数）
   * @param response 服务响应，包含成功标志和消息
   */
  void turn_off_motors_callback(const std_srvs::srv::Trigger::Request::SharedPtr /* request */, 
                               std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    RCLCPP_INFO(this->get_logger(), "Received turn off motors request");
    
    if (!psdk_wrapper_) {
      RCLCPP_ERROR(this->get_logger(), "PSDKWrapper not initialized");
      response->success = false;
      response->message = "PSDKWrapper not initialized";
      return;
    }
    
    T_DjiReturnCode result = psdk_wrapper_->turnOffMotors();
    
    if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Turn off motors command sent successfully");
      response->success = true;
      response->message = "Turn off motors command sent successfully";
    } else if (result == DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND) {
      RCLCPP_ERROR(this->get_logger(), "Failed to turn off motors: Not connected to drone");
      response->success = false;
      response->message = "Not connected to drone";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to turn off motors: Error code %ld", result);
      response->success = false;
      response->message = "Failed to turn off motors: Error code " + std::to_string(result);
    }
  }
  
  /**
   * @brief 更新飞行控制配置回调函数
   * @param request 服务请求，包含配置文件路径
   * @param response 服务响应，包含成功标志和消息
   */
  void update_flight_control_config_callback(const communication::srv::UpdateFlightControlConfig::Request::SharedPtr request, 
                                           communication::srv::UpdateFlightControlConfig::Response::SharedPtr response)
  {
    // 使用默认配置文件路径
    std::string config_path = request->config_path;
    if (config_path.empty()) {
      try {
        // 获取包的share目录
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("dji_psdk_wrapper");
        // 构建配置文件的相对路径
        config_path = package_share_directory + "/config/flying_config.json";
        RCLCPP_INFO(this->get_logger(), "Using default config path: %s", config_path.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
        response->success = false;
        response->message = "Failed to get package share directory: " + std::string(e.what());
        return;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Received update flight control config request, config_path: %s", config_path.c_str());
    }
    
    if (!psdk_wrapper_) {
      RCLCPP_ERROR(this->get_logger(), "PSDKWrapper not initialized");
      response->success = false;
      response->message = "PSDKWrapper not initialized";
      return;
    }
    
    T_DjiReturnCode result = psdk_wrapper_->updateFlightControlConfig(config_path);
    
    if (result == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Update flight control config successfully");
      response->success = true;
      response->message = "Update flight control config successfully";
    } else if (result == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to update flight control config: Not connected to aircraft");
      response->success = false;
      response->message = "Not connected to aircraft";
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to update flight control config: Error code %ld", result);
      response->success = false;
      response->message = "Failed to update flight control config: Error code " + std::to_string(result);
    }
  }
};

}  // namespace dji_psdk_wrapper

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dji_psdk_wrapper::PSDKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}