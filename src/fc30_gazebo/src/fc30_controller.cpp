#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <communication/msg/psdk_cmd.hpp>

class FC30Controller : public rclcpp::Node
{
public:
  FC30Controller() : Node("fc30_controller")
  {
    // Subscribers
    psdk_cmd_sub_ = this->create_subscription<communication::msg::PSDKCmd>(
      "/psdk_cmd", 10, 
      std::bind(&FC30Controller::psdk_cmd_callback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/fc30/pose_estimate", 10, 
      std::bind(&FC30Controller::pose_callback, this, std::placeholders::_1));
    
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/fc30/twist_estimate", 10, 
      std::bind(&FC30Controller::twist_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/fc30/imu", 10, 
      std::bind(&FC30Controller::imu_callback, this, std::placeholders::_1));
    
    // Publishers for rotor velocities
    rotor_front_left_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/fc30/rotor_front_left/cmd_vel", 10);
      
    rotor_front_right_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/fc30/rotor_front_right/cmd_vel", 10);
      
    rotor_back_left_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/fc30/rotor_back_left/cmd_vel", 10);
      
    rotor_back_right_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/fc30/rotor_back_right/cmd_vel", 10);
    
    // Timer for control loop
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&FC30Controller::control_loop, this));
    
    /**
   * @brief 初始化目标姿态
   * 设置默认的悬停高度为1.0米
   */
  target_pose_.pose.position.x = 0.0;
  target_pose_.pose.position.y = 0.0;
  target_pose_.pose.position.z = 1.0;  // 默认悬停高度
  target_yaw_ = 0.0;  // 默认偏航角为0弧度
  
  /**
   * @brief 初始化当前姿态
   */
  current_pose_.pose.position.x = 0.0;
  current_pose_.pose.position.y = 0.0;
  current_pose_.pose.position.z = 0.0;
  
  /**
   * @brief 初始化PID增益参数
   * 设置位置控制器和姿态控制器的PID增益值
   */
  kp_pos_ = {0.5, 0.5, 1.0};  // 位置控制器比例增益
  ki_pos_ = {0.0, 0.0, 0.1};  // 位置控制器积分增益
  kd_pos_ = {0.2, 0.2, 0.5};  // 位置控制器微分增益
  
  kp_att_ = {2.0, 2.0, 1.0};  // 姿态控制器比例增益
  ki_att_ = {0.0, 0.0, 0.0};  // 姿态控制器积分增益
  kd_att_ = {0.5, 0.5, 0.2};  // 姿态控制器微分增益
    
    RCLCPP_INFO(this->get_logger(), "FC30 Controller Node initialized");
  }

private:
  /**
   * @brief PSDK命令订阅者
   * 订阅来自地面站的PSDK命令消息
   */
  rclcpp::Subscription<communication::msg::PSDKCmd>::SharedPtr psdk_cmd_sub_;
  
  /**
   * @brief 无人机姿态订阅者
   * 订阅无人机的估计姿态信息
   */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  
  /**
   * @brief 无人机速度订阅者
   * 订阅无人机的估计速度信息
   */
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  
  /**
   * @brief IMU数据订阅者
   * 订阅无人机的IMU数据
   */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  /**
   * @brief 前左旋翼速度发布者
   * 发布前左旋翼的目标速度命令
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_front_left_pub_;
  
  /**
   * @brief 前右旋翼速度发布者
   * 发布前右旋翼的目标速度命令
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_front_right_pub_;
  
  /**
   * @brief 后左旋翼速度发布者
   * 发布后左旋翼的目标速度命令
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_back_left_pub_;
  
  /**
   * @brief 后右旋翼速度发布者
   * 发布后右旋翼的目标速度命令
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_back_right_pub_;
  
  /**
   * @brief 控制循环定时器
   * 定期执行控制循环，默认10ms执行一次
   */
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  /**
   * @brief 当前无人机姿态
   * 存储无人机的当前估计姿态
   */
  geometry_msgs::msg::PoseStamped current_pose_;
  
  /**
   * @brief 当前无人机速度
   * 存储无人机的当前估计速度
   */
  geometry_msgs::msg::TwistStamped current_twist_;
  
  /**
   * @brief 当前IMU数据
   * 存储无人机的当前IMU数据
   */
  sensor_msgs::msg::Imu current_imu_;
  
  /**
   * @brief 目标姿态
   * 存储无人机的目标姿态
   */
  geometry_msgs::msg::PoseStamped target_pose_;
  
  /**
   * @brief 目标偏航角
   * 存储无人机的目标偏航角（单位：弧度）
   */
  double target_yaw_;
  
  /**
   * @brief 飞行状态标志
   * 标记无人机是否处于飞行状态
   */
  bool is_flying_ = false;
  
  /**
   * @brief 位置控制器PID比例增益
   * PID控制器的比例增益参数，分别对应x、y、z轴
   * 默认值：(0.5, 0.5, 1.0)
   * 取值范围：0.0-10.0
   */
  std::tuple<double, double, double> kp_pos_;
  
  /**
   * @brief 位置控制器PID积分增益
   * PID控制器的积分增益参数，分别对应x、y、z轴
   * 默认值：(0.0, 0.0, 0.1)
   * 取值范围：0.0-1.0
   */
  std::tuple<double, double, double> ki_pos_;
  
  /**
   * @brief 位置控制器PID微分增益
   * PID控制器的微分增益参数，分别对应x、y、z轴
   * 默认值：(0.2, 0.2, 0.5)
   * 取值范围：0.0-5.0
   */
  std::tuple<double, double, double> kd_pos_;
  
  /**
   * @brief 姿态控制器PID比例增益
   * PID控制器的比例增益参数，分别对应roll、pitch、yaw轴
   * 默认值：(2.0, 2.0, 1.0)
   * 取值范围：0.0-10.0
   */
  std::tuple<double, double, double> kp_att_;
  
  /**
   * @brief 姿态控制器PID积分增益
   * PID控制器的积分增益参数，分别对应roll、pitch、yaw轴
   * 默认值：(0.0, 0.0, 0.0)
   * 取值范围：0.0-1.0
   */
  std::tuple<double, double, double> ki_att_;
  
  /**
   * @brief 姿态控制器PID微分增益
   * PID控制器的微分增益参数，分别对应roll、pitch、yaw轴
   * 默认值：(0.5, 0.5, 0.2)
   * 取值范围：0.0-5.0
   */
  std::tuple<double, double, double> kd_att_;
  
  /**
   * @brief 位置误差
   * 存储位置控制器的当前误差值，分别对应x、y、z轴
   */
  std::tuple<double, double, double> pos_error_;
  
  /**
   * @brief 位置误差积分
   * 存储位置控制器的误差积分值，分别对应x、y、z轴
   * 使用抗积分饱和技术限制在[-5.0, 5.0]范围内
   */
  std::tuple<double, double, double> pos_error_integral_;
  
  /**
   * @brief 位置误差微分
   * 存储位置控制器的误差微分值，分别对应x、y、z轴
   */
  std::tuple<double, double, double> pos_error_derivative_;
  
  /**
   * @brief 姿态误差
   * 存储姿态控制器的当前误差值，分别对应roll、pitch、yaw轴
   */
  std::tuple<double, double, double> att_error_;
  
  /**
   * @brief 姿态误差积分
   * 存储姿态控制器的误差积分值，分别对应roll、pitch、yaw轴
   */
  std::tuple<double, double, double> att_error_integral_;
  
  /**
   * @brief 姿态误差微分
   * 存储姿态控制器的误差微分值，分别对应roll、pitch、yaw轴
   */
  std::tuple<double, double, double> att_error_derivative_;
  
  /**
   * @brief PSDK命令回调函数
   * 处理来自地面站的PSDK命令消息
   * 
   * @param msg PSDK命令消息指针，包含命令类型和参数
   */
  void psdk_cmd_callback(const communication::msg::PSDKCmd::SharedPtr msg)
  {
    if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_TAKEOFF)
    {
      RCLCPP_INFO(this->get_logger(), "Takeoff command received");
      target_pose_.pose.position.z = 1.5;  // 起飞到1.5米高度
      is_flying_ = true;
    }
    else if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_LAND)
    {
      RCLCPP_INFO(this->get_logger(), "Land command received");
      target_pose_.pose.position.z = 0.1;  // 着陆到0.1米高度
      is_flying_ = false;
    }
    else if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_HOVER)
    {
      RCLCPP_INFO(this->get_logger(), "Hover command received");
      // 保持当前位置
      target_pose_.pose.position.x = current_pose_.pose.position.x;
      target_pose_.pose.position.y = current_pose_.pose.position.y;
      target_pose_.pose.position.z = current_pose_.pose.position.z;
    }
    else if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_MOVE)
    {
      RCLCPP_INFO(this->get_logger(), "Move command received: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
                msg->target_x, msg->target_y, msg->target_z, msg->target_yaw);
      target_pose_.pose.position.x = msg->target_x;
      target_pose_.pose.position.y = msg->target_y;
      target_pose_.pose.position.z = msg->target_z;
      target_yaw_ = msg->target_yaw;
    }
    // 可以在这里添加其他命令类型的处理
  }
  
  /**
   * @brief 姿态消息回调函数
   * 更新无人机当前姿态信息
   * 
   * @param msg 姿态消息指针
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
  }
  
  /**
   * @brief 速度消息回调函数
   * 更新无人机当前速度信息
   * 
   * @param msg 速度消息指针
   */
  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    current_twist_ = *msg;
  }
  
  /**
   * @brief IMU数据回调函数
   * 更新无人机当前IMU数据
   * 
   * @param msg IMU消息指针
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_imu_ = *msg;
  }
  
  /**
   * @brief 控制循环函数
   * 执行位置和姿态控制逻辑，计算并设置四个旋翼的速度
   * 控制周期：100Hz (10ms)
   */
  void control_loop()
  {
    // 检查是否已经着陆，若着陆则停止所有旋翼
    if (!is_flying_ && current_pose_.pose.position.z < 0.2)
    {
      // Landed, stop all rotors
      set_rotor_velocities(0.0, 0.0, 0.0, 0.0);
      return;
    }
    
    // 计算位置误差（目标位置 - 当前位置）
    double pos_error_x = target_pose_.pose.position.x - current_pose_.pose.position.x;
    double pos_error_y = target_pose_.pose.position.y - current_pose_.pose.position.y;
    double pos_error_z = target_pose_.pose.position.z - current_pose_.pose.position.z;
    
    // 计算位置误差微分（使用当前速度的负值作为误差变化率）
    double pos_error_derivative_x = -current_twist_.twist.linear.x;
    double pos_error_derivative_y = -current_twist_.twist.linear.y;
    double pos_error_derivative_z = -current_twist_.twist.linear.z;
    
    // 更新位置误差积分（使用控制周期0.01秒，实现抗积分饱和）
    double pos_error_integral_x = std::get<0>(pos_error_integral_) + pos_error_x * 0.01;
    double pos_error_integral_y = std::get<1>(pos_error_integral_) + pos_error_y * 0.01;
    double pos_error_integral_z = std::get<2>(pos_error_integral_) + pos_error_z * 0.01;
    
    // 抗积分饱和：限制积分项在合理范围内
    pos_error_integral_x = std::clamp(pos_error_integral_x, -5.0, 5.0);
    pos_error_integral_y = std::clamp(pos_error_integral_y, -5.0, 5.0);
    pos_error_integral_z = std::clamp(pos_error_integral_z, -5.0, 5.0);
    
    // 更新误差变量
    pos_error_ = std::make_tuple(pos_error_x, pos_error_y, pos_error_z);
    pos_error_derivative_ = std::make_tuple(pos_error_derivative_x, pos_error_derivative_y, pos_error_derivative_z);
    pos_error_integral_ = std::make_tuple(pos_error_integral_x, pos_error_integral_y, pos_error_integral_z);
    
    // 使用PID算法计算期望加速度
    double desired_accel_x = std::get<0>(kp_pos_) * pos_error_x + std::get<0>(ki_pos_) * pos_error_integral_x + std::get<0>(kd_pos_) * pos_error_derivative_x;
    double desired_accel_y = std::get<1>(kp_pos_) * pos_error_y + std::get<1>(ki_pos_) * pos_error_integral_y + std::get<1>(kd_pos_) * pos_error_derivative_y;
    double desired_accel_z = std::get<2>(kp_pos_) * pos_error_z + std::get<2>(ki_pos_) * pos_error_integral_z + std::get<2>(kd_pos_) * pos_error_derivative_z;
    
    // 基于期望加速度计算期望roll和pitch角（小角度近似）
    double roll = desired_accel_x / 9.81;  // 转换为角度（小角度近似）
    double pitch = desired_accel_y / 9.81; // 转换为角度（小角度近似）
    
    // 计算期望推力（补偿重力加速度）
    double thrust = (desired_accel_z + 9.81) * 2.5;  // 2.5是无人机的质量（单位：kg）
    
    // 计算偏航误差
    double yaw_error = target_yaw_ - get_current_yaw();
    
    // 归一化偏航误差到[-pi, pi]范围内
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
    
    // 计算期望偏航率
    double yaw_rate = std::get<2>(kp_pos_) * yaw_error;
    
    // 使用简化的四旋翼模型计算每个旋翼的速度
    double front_left = thrust + roll + pitch - yaw_rate;
    double front_right = thrust - roll + pitch + yaw_rate;
    double back_left = thrust + roll - pitch + yaw_rate;
    double back_right = thrust - roll - pitch - yaw_rate;
    
    // 设置旋翼速度
    set_rotor_velocities(front_left, front_right, back_left, back_right);
  }
  
  /**
   * @brief 获取当前偏航角
   * 从四元数转换为偏航角
   * 
   * @return 当前偏航角（单位：弧度）
   */
  double get_current_yaw()
  {
    // 从四元数中提取偏航角
    double qx = current_pose_.pose.orientation.x;
    double qy = current_pose_.pose.orientation.y;
    double qz = current_pose_.pose.orientation.z;
    double qw = current_pose_.pose.orientation.w;
    
    // 使用四元数到欧拉角的转换公式计算偏航角
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
  }
  
  /**
   * @brief 设置四个旋翼的速度
   * 限制旋翼速度在合理范围内并发布
   * 
   * @param front_left 前左旋翼速度
   * @param front_right 前右旋翼速度
   * @param back_left 后左旋翼速度
   * @param back_right 后右旋翼速度
   */
  void set_rotor_velocities(double front_left, double front_right, double back_left, double back_right)
  {
    // 限制旋翼速度在合理范围内 [0.0, 1000.0]
    front_left = std::max(0.0, std::min(1000.0, front_left));
    front_right = std::max(0.0, std::min(1000.0, front_right));
    back_left = std::max(0.0, std::min(1000.0, back_left));
    back_right = std::max(0.0, std::min(1000.0, back_right));
    
    // 发布旋翼速度
    auto msg = std_msgs::msg::Float64();
    
    msg.data = front_left;
    rotor_front_left_pub_->publish(msg);
    
    msg.data = front_right;
    rotor_front_right_pub_->publish(msg);
    
    msg.data = back_left;
    rotor_back_left_pub_->publish(msg);
    
    msg.data = back_right;
    rotor_back_right_pub_->publish(msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FC30Controller>());
  rclcpp::shutdown();
  return 0;
}