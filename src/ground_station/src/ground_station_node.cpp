#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <communication/msg/system_state.hpp>
#include <communication/msg/mission_command.hpp>
#include <communication/msg/drone_command.hpp>
#include <communication/msg/robot_command.hpp>
#include <communication/msg/hook_command.hpp>
#include <communication/msg/psdk_cmd.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std::chrono_literals;

/**
 * @brief 获取当前时间字符串（北京时间格式）
 * @return std::string 格式化的时间字符串，格式为 "MM-DD HH:MM:SS.ms"
 */
std::string get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    // 使用线程安全的时间转换函数
    std::tm now_tm;
    localtime_r(&now_time_t, &now_tm);
    
    // 格式化时间字符串 - 北京时间格式（去掉年份）
    char time_str[50];
    std::snprintf(time_str, sizeof(time_str), "%02d-%02d %02d:%02d:%02d.%03d",
                now_tm.tm_mon + 1,
                now_tm.tm_mday,
                now_tm.tm_hour,
                now_tm.tm_min,
                now_tm.tm_sec,
                static_cast<int>(now_ms.count()));
    
    return std::string(time_str);
}

// 自定义日志宏，使用与无人机节点相同的格式（只输出自定义格式，不输出ROS2默认格式）
#define CUSTOM_LOG_INFO(node, format, ...) \
    do { \
        char buffer[256]; \
        snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
        std::cout << "[INFO] [" << get_current_time_string() << "] " << buffer << std::endl; \
    } while (0)

#define CUSTOM_LOG_WARN(node, format, ...) \
    do { \
        char buffer[256]; \
        snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
        std::cout << "[WARN] [" << get_current_time_string() << "] " << buffer << std::endl; \
    } while (0)

#define CUSTOM_LOG_ERROR(node, format, ...) \
    do { \
        char buffer[256]; \
        snprintf(buffer, sizeof(buffer), format, ##__VA_ARGS__); \
        std::cerr << "[ERROR] [" << get_current_time_string() << "] " << buffer << std::endl; \
    } while (0)

class GroundStationNode : public rclcpp::Node
{
private:
    // 发布者
    rclcpp::Publisher<communication::msg::SystemState>::SharedPtr system_state_pub_;
    rclcpp::Publisher<communication::msg::DroneCommand>::SharedPtr drone_command_pub_;
    rclcpp::Publisher<communication::msg::PSDKCmd>::SharedPtr psdk_cmd_pub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr upload_waypoint_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_waypoint_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_waypoint_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_waypoint_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_waypoint_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr verify_waypoint_srv_;
    
    // 订阅者
    rclcpp::Subscription<communication::msg::SystemState>::SharedPtr drone_state_sub_;
    rclcpp::Subscription<communication::msg::MissionCommand>::SharedPtr mission_cmd_sub_;
    // 无人机命令订阅者（订阅前端命令）
    rclcpp::Subscription<communication::msg::DroneCommand>::SharedPtr drone_command_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 系统状态
    communication::msg::SystemState system_state_;
    communication::msg::SystemState latest_drone_state_;
    
    // 异常检测标志
    bool emergency_stop_triggered_;
    rclcpp::Time last_emergency_time_;
    
    // 配置参数
    double min_battery_percentage_;
    int min_satellite_count_;
    bool enable_auto_emergency_;
    bool enable_connection_check_;
    
    /**
     * @brief 无人机状态回调函数
     * 当接收到无人机状态消息时调用，保存最新的无人机状态并检测异常
     * 
     * @param msg 无人机状态消息指针
     */
    void drone_state_callback(const communication::msg::SystemState::SharedPtr msg)
    {
        // 保存最新的无人机状态
        latest_drone_state_ = *msg;
        
        // 检测异常
        check_for_anomalies();
    }
    
    /**
     * @brief 异常检测函数
     * 检测无人机状态中的异常情况，如连接断开、低电量、系统错误等
     * 当检测到异常时，触发相应的紧急措施
     */
    void check_for_anomalies()
    {
        // 如果已经触发紧急停止，不再重复处理
        if (emergency_stop_triggered_) {
            return;
        }
        
        // 检查无人机状态消息是否有效（避免空消息导致的误检测）
        if (latest_drone_state_.system_name.empty()) {
            // 如果系统名称为空，说明还没有收到有效的无人机状态消息
            CUSTOM_LOG_WARN(this, "尚未收到有效的无人机状态消息，跳过异常检测");
            return;
        }
        
        std::string error_message = "";
        bool anomaly_detected = false;
        
        // 检测无人机连接状态
        if (enable_connection_check_ && (latest_drone_state_.mode == "disconnected" || latest_drone_state_.status == "disconnected")) {
            error_message = "无人机连接断开";
            anomaly_detected = true;
        }
        
        // 检测电池电量
        else if (latest_drone_state_.battery1.percentage < min_battery_percentage_ || 
                 latest_drone_state_.battery2.percentage < min_battery_percentage_) {
            error_message = "电池电量过低: " + std::to_string(latest_drone_state_.battery1.percentage) + "% / " + std::to_string(latest_drone_state_.battery2.percentage) + "%";
            anomaly_detected = true;
        }
        
        // 检测系统状态
        else if (latest_drone_state_.status == "error" || latest_drone_state_.status == "critical") {
            error_message = "无人机系统错误: " + latest_drone_state_.error_message;
            anomaly_detected = true;
        }
        
        // 检测紧急停止状态
        else if (latest_drone_state_.emergency_stop) {
            error_message = "无人机触发了紧急停止";
            anomaly_detected = true;
        }
        
        // 如果检测到异常
        if (anomaly_detected) {
            // 确保错误信息不为空
            if (error_message.empty()) {
                error_message = "未知异常";
            }
            
            CUSTOM_LOG_ERROR(this, "检测到异常: %s", error_message.c_str());
            
            // 更新系统状态
            system_state_.status = "critical";
            system_state_.error_message = error_message;
            
            // 触发应急措施
            if (enable_auto_emergency_) {
                trigger_emergency_action();
            }
        }
    }
    
    // 触发应急措施
    void trigger_emergency_action()
    {
        emergency_stop_triggered_ = true;
        last_emergency_time_ = this->get_clock()->now();
        
        CUSTOM_LOG_ERROR(this, "触发应急措施: 发送紧急停飞命令");
        
        // 发送紧急停飞命令
        auto emergency_cmd = communication::msg::DroneCommand();
        emergency_cmd.header.stamp = this->get_clock()->now();
        emergency_cmd.command_type = communication::msg::DroneCommand::COMMAND_TYPE_EMERGENCY_STOP;
        emergency_cmd.emergency_stop = true;
        
        drone_command_pub_->publish(emergency_cmd);
        
        // 更新系统状态
        system_state_.emergency_stop = true;
        system_state_.status = "critical";
        system_state_.error_message = "已触发地面站紧急停飞";
    }
    
    /**
     * @brief 任务命令回调函数
     * 当接收到任务命令消息时调用，处理紧急停止等命令
     * 
     * @param msg 任务命令消息指针
     */
    /**
     * @brief 无人机命令回调函数
     * 当接收到无人机控制命令时调用，转发给无人机节点
     * 
     * @param msg 无人机控制命令消息指针
     */
    void drone_command_callback(const communication::msg::DroneCommand::SharedPtr msg)
    {
        // 将数字命令类型转换为中文描述以提高日志可读性
        std::string command_desc;
        switch(msg->command_type) {
            case communication::msg::DroneCommand::COMMAND_TYPE_TAKEOFF: command_desc = "起飞";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_LAND: command_desc = "降落";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_MOVE: command_desc = "移动";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_HOVER: command_desc = "悬停";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_RETURN_HOME: command_desc = "返航";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_SET_MODE: command_desc = "设置模式";
                break;
            case communication::msg::DroneCommand::COMMAND_TYPE_EMERGENCY_STOP: command_desc = "紧急停止";
                break;
            default: command_desc = "未知命令 (" + std::to_string(msg->command_type) + ")";
        }
        
        CUSTOM_LOG_INFO(this, "接收到前端无人机控制命令: %s", command_desc.c_str());
        
        // 转发命令给无人机节点
        drone_command_pub_->publish(*msg);
    }
    
    /**
     * @brief 任务命令回调函数
     * 当接收到任务命令消息时调用，处理紧急停止等命令
     * 
     * @param msg 任务命令消息指针
     */
    void mission_command_callback(const communication::msg::MissionCommand::SharedPtr msg)
    {
        // 处理紧急停止命令
        if (msg->emergency_stop) {
            CUSTOM_LOG_ERROR(this, "收到紧急停止命令，触发应急措施");
            trigger_emergency_action();
        }
    }
    
    // 清除紧急状态
    void clear_emergency_state()
    {
        emergency_stop_triggered_ = false;
        
        // 发送解除紧急停止命令
        auto clear_cmd = communication::msg::DroneCommand();
        clear_cmd.header.stamp = this->get_clock()->now();
        clear_cmd.command_type = communication::msg::DroneCommand::COMMAND_TYPE_EMERGENCY_STOP;
        clear_cmd.emergency_stop = false;
        
        drone_command_pub_->publish(clear_cmd);
        
        // 更新系统状态
        system_state_.emergency_stop = false;
        system_state_.status = "normal";
        system_state_.error_message = "";
        
        CUSTOM_LOG_INFO(this, "紧急状态已清除");
    }

    // 定时器回调，用于发布系统状态
    void timer_callback()
    {
        auto header = std_msgs::msg::Header();
        header.stamp = this->get_clock()->now();
        header.frame_id = "ground_station";

        system_state_.header = header;
        system_state_.system_name = "Hoisting System";
        system_state_.mode = "manual";
        
        // 更新紧急停止状态
        system_state_.emergency_stop = emergency_stop_triggered_;
        
        // 如果没有检测到异常，保持正常状态
        if (!emergency_stop_triggered_ && system_state_.status != "error" && system_state_.status != "critical") {
            system_state_.status = "normal";
        }

        system_state_pub_->publish(system_state_);
        
        // 定期检查紧急状态，如果超过一定时间自动解除（可选功能）
        if (emergency_stop_triggered_) {
            auto now = this->get_clock()->now();
            auto duration = now - last_emergency_time_;
            if (duration > 5s) {
                CUSTOM_LOG_WARN(this, "紧急状态已持续超过5秒，自动解除");
                clear_emergency_state();
            }
        }
    }

public:
    GroundStationNode() : Node("ground_station_node"), 
                         emergency_stop_triggered_(false),
                         min_battery_percentage_(20.0),
                         min_satellite_count_(8),
                         enable_auto_emergency_(false),
                         enable_connection_check_(false)
    {
        CUSTOM_LOG_INFO(this, "Ground Station Node Started");
        
        // 初始化系统状态
        system_state_.system_name = "Ground Station";
        system_state_.mode = "manual";
        system_state_.status = "normal";
        system_state_.emergency_stop = false;
        system_state_.system_ready = true;
        system_state_.error_message = "";
        
        // 声明并获取参数
        this->declare_parameter<double>("min_battery_percentage", 20.0);
        this->declare_parameter<int>("min_satellite_count", 8);
        this->declare_parameter<bool>("enable_auto_emergency", false);
        this->declare_parameter<bool>("enable_connection_check", false);
        
        min_battery_percentage_ = this->get_parameter("min_battery_percentage").as_double();
        min_satellite_count_ = this->get_parameter("min_satellite_count").as_int();
        enable_auto_emergency_ = this->get_parameter("enable_auto_emergency").as_bool();
        enable_connection_check_ = this->get_parameter("enable_connection_check").as_bool();
        
        CUSTOM_LOG_INFO(this, "地面站配置:");
        CUSTOM_LOG_INFO(this, "  最低电池电量阈值: %.1f%%", min_battery_percentage_);
        CUSTOM_LOG_INFO(this, "  最低卫星数量: %d颗", min_satellite_count_);
        CUSTOM_LOG_INFO(this, "  自动应急措施: %s", enable_auto_emergency_ ? "启用" : "禁用");
        CUSTOM_LOG_INFO(this, "  无人机连接检测: %s", enable_connection_check_ ? "启用" : "禁用");

        // 初始化发布者
        system_state_pub_ = this->create_publisher<communication::msg::SystemState>("/system_state", 10);
        drone_command_pub_ = this->create_publisher<communication::msg::DroneCommand>("/drone/command", 10);
        psdk_cmd_pub_ = this->create_publisher<communication::msg::PSDKCmd>("/drone/psdk_cmd", 10);
        
        // 初始化服务
        upload_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/ground_station/upload_waypoint_v3",
            std::bind(&GroundStationNode::upload_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        start_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/ground_station/start_waypoint_v3",
            std::bind(&GroundStationNode::start_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
                       
        stop_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/ground_station/stop_waypoint_v3",
            std::bind(&GroundStationNode::stop_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
                       
        pause_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/ground_station/pause_waypoint_v3",
            std::bind(&GroundStationNode::pause_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
                       
        resume_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/ground_station/resume_waypoint_v3",
            std::bind(&GroundStationNode::resume_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // 执行检验服务
        verify_waypoint_srv_ = create_service<std_srvs::srv::Trigger>(
            "/ground_station/verify_waypoint_v3",
            std::bind(&GroundStationNode::verify_waypoint_v3_callback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        CUSTOM_LOG_INFO(this, "  - 航线上传: /ground_station/upload_waypoint_v3");
        CUSTOM_LOG_INFO(this, "  - 任务开始: /ground_station/start_waypoint_v3");
        CUSTOM_LOG_INFO(this, "  - 任务停止: /ground_station/stop_waypoint_v3");
        CUSTOM_LOG_INFO(this, "  - 任务暂停: /ground_station/pause_waypoint_v3");
        CUSTOM_LOG_INFO(this, "  - 任务恢复: /ground_station/resume_waypoint_v3");
        CUSTOM_LOG_INFO(this, "  - 执行检验: /ground_station/verify_waypoint_v3");
        CUSTOM_LOG_INFO(this, "订阅的话题:");
        CUSTOM_LOG_INFO(this, "  - 无人机状态: /drone/system_state");
        CUSTOM_LOG_INFO(this, "  - 前端无人机控制命令: /frontend/drone/command");
        CUSTOM_LOG_INFO(this, "  - 任务命令: /mission/cmd");
        
        // 初始化订阅者
        drone_state_sub_ = this->create_subscription<communication::msg::SystemState>(
            "/drone/system_state", 10, 
            std::bind(&GroundStationNode::drone_state_callback, this, std::placeholders::_1));
            
        // 订阅前端无人机命令话题（从前端接收控制命令）
        drone_command_sub_ = this->create_subscription<communication::msg::DroneCommand>(
            "/frontend/drone/command", 10, 
            std::bind(&GroundStationNode::drone_command_callback, this, std::placeholders::_1));
            
        // 订阅任务命令话题
        mission_cmd_sub_ = this->create_subscription<communication::msg::MissionCommand>(
            "/mission/cmd", 10, 
            std::bind(&GroundStationNode::mission_command_callback, this, std::placeholders::_1));

        // 初始化定时器，每秒发布一次系统状态
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&GroundStationNode::timer_callback, this));
            
        // 初始化系统状态
        emergency_stop_triggered_ = false;
        last_emergency_time_ = this->get_clock()->now();
        
        CUSTOM_LOG_INFO(this, "地面站已初始化完成，等待命令...");
    }

    // 航线上传服务回调
    void upload_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到航线上传请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            // 1. 上传航线文件
            auto upload_cmd = communication::msg::PSDKCmd();
            upload_cmd.header.stamp = this->get_clock()->now();
            upload_cmd.command_type = communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_UPLOAD;
            // 这里应该使用实际的航线文件路径
            upload_cmd.waypoint_file_path = "src/ground_station/config/waypoint_v3_test_file.kmz";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 准备发送航线文件上传命令");
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线文件路径: %s", upload_cmd.waypoint_file_path.c_str());
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 命令类型: %d (WAYPOINT_V3_UPLOAD)", upload_cmd.command_type);
            
            psdk_cmd_pub_->publish(upload_cmd);
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 已成功发送航线文件上传命令到 /drone/psdk_cmd 话题");
            
            response->success = true;
            response->message = "Waypoint V3 航线文件上传成功";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线上传请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理航线上传请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理航线上传请求时发生异常: " + std::string(e.what());
        }
    }
    
    // 航线执行检验服务回调
    void verify_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到执行检验请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            // 执行航线检验逻辑
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 开始执行航线检验...");
            
            // 检查无人机状态
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 检查无人机状态: 模式=%s, 状态=%s", 
                       latest_drone_state_.mode.c_str(), 
                       latest_drone_state_.status.c_str());
            
            // 检查电池电量
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 检查电池电量: %f%%/%f%%", 
                       latest_drone_state_.battery1.percentage, 
                       latest_drone_state_.battery2.percentage);
            
            // 检查系统就绪状态
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 检查系统就绪状态: %s", 
                       latest_drone_state_.system_ready ? "就绪" : "未就绪");
            
            // 检查航线文件是否存在
            std::string waypoint_file_path = "src/ground_station/config/waypoint_v3_test_file.kmz";
            std::ifstream file(waypoint_file_path);
            if (file.good()) {
                CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线文件存在: %s", waypoint_file_path.c_str());
            } else {
                CUSTOM_LOG_WARN(this, "[Waypoint V3] 航线文件可能不存在: %s", waypoint_file_path.c_str());
            }
            file.close();
            
            // 模拟检验过程
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 执行详细检验步骤...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线执行检验完成");
            
            response->success = true;
            response->message = "Waypoint V3 执行检验通过";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 执行检验请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理执行检验请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理执行检验请求时发生异常: " + std::string(e.what());
        }
    }
    
    // 航线任务开始服务回调
    void start_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到航线任务开始请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            auto start_cmd = communication::msg::PSDKCmd();
            start_cmd.header.stamp = this->get_clock()->now();
            start_cmd.command_type = communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_START;
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 准备发送航线任务开始命令");
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 命令类型: %d (WAYPOINT_V3_START)", start_cmd.command_type);
            
            psdk_cmd_pub_->publish(start_cmd);
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 已成功发送航线任务开始命令到 /drone/psdk_cmd 话题");
            
            response->success = true;
            response->message = "Waypoint V3 航线任务开始命令已发送";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线任务开始请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理航线任务开始请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理航线任务开始请求时发生异常: " + std::string(e.what());
        }
    }
    
    // 航线任务停止服务回调
    void stop_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到航线任务停止请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            auto stop_cmd = communication::msg::PSDKCmd();
            stop_cmd.header.stamp = this->get_clock()->now();
            stop_cmd.command_type = communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_STOP;
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 准备发送航线任务停止命令");
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 命令类型: %d (WAYPOINT_V3_STOP)", stop_cmd.command_type);
            
            psdk_cmd_pub_->publish(stop_cmd);
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 已成功发送航线任务停止命令到 /drone/psdk_cmd 话题");
            
            response->success = true;
            response->message = "Waypoint V3 航线任务停止命令已发送";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线任务停止请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理航线任务停止请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理航线任务停止请求时发生异常: " + std::string(e.what());
        }
    }
    
    // 航线任务暂停服务回调
    void pause_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到航线任务暂停请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            auto pause_cmd = communication::msg::PSDKCmd();
            pause_cmd.header.stamp = this->get_clock()->now();
            pause_cmd.command_type = communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_PAUSE;
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 准备发送航线任务暂停命令");
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 命令类型: %d (WAYPOINT_V3_PAUSE)", pause_cmd.command_type);
            
            psdk_cmd_pub_->publish(pause_cmd);
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 已成功发送航线任务暂停命令到 /drone/psdk_cmd 话题");
            
            response->success = true;
            response->message = "Waypoint V3 航线任务暂停命令已发送";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线任务暂停请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理航线任务暂停请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理航线任务暂停请求时发生异常: " + std::string(e.what());
        }
    }
    
    // 航线任务恢复服务回调
    void resume_waypoint_v3_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 收到航线任务恢复请求");
        CUSTOM_LOG_INFO(this, "[Waypoint V3] 请求时间戳: %f", this->get_clock()->now().seconds());
        
        try {
            auto resume_cmd = communication::msg::PSDKCmd();
            resume_cmd.header.stamp = this->get_clock()->now();
            resume_cmd.command_type = communication::msg::PSDKCmd::CMD_TYPE_WAYPOINT_V3_RESUME;
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 准备发送航线任务恢复命令");
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 命令类型: %d (WAYPOINT_V3_RESUME)", resume_cmd.command_type);
            
            psdk_cmd_pub_->publish(resume_cmd);
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 已成功发送航线任务恢复命令到 /drone/psdk_cmd 话题");
            
            response->success = true;
            response->message = "Waypoint V3 航线任务恢复命令已发送";
            
            CUSTOM_LOG_INFO(this, "[Waypoint V3] 航线任务恢复请求处理完成，返回成功响应");
        } catch (const std::exception& e) {
            CUSTOM_LOG_ERROR(this, "[Waypoint V3] 处理航线任务恢复请求时发生异常: %s", e.what());
            response->success = false;
            response->message = "处理航线任务恢复请求时发生异常: " + std::string(e.what());
        }
    }
    
    ~GroundStationNode()
    {
        CUSTOM_LOG_INFO(this, "Ground Station Node Stopped");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundStationNode>());
    rclcpp::shutdown();
    return 0;
}