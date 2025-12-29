#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <communication/msg/psdk_cmd.hpp>
#include <communication/msg/psdk_data.hpp>

class PSDKBridge : public rclcpp::Node
{
public:
  PSDKBridge() : Node("psdk_bridge")
  {
    // Publishers to Gazebo
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fc30/pose", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fc30/twist", 10);
    
    // Subscribers from Gazebo
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/fc30/imu", 10, 
      std::bind(&PSDKBridge::imu_callback, this, std::placeholders::_1));
      
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fc30/gps/fix", 10, 
      std::bind(&PSDKBridge::gps_callback, this, std::placeholders::_1));
    
    // Subscribers from PSDK
    psdk_cmd_sub_ = this->create_subscription<communication::msg::PSDKCmd>(
      "/psdk_cmd", 10, 
      std::bind(&PSDKBridge::psdk_cmd_callback, this, std::placeholders::_1));
    
    // Publishers to PSDK
    psdk_data_pub_ = this->create_publisher<communication::msg::PSDKData>(
      "/psdk_data", 10);
    
    // Timer for publishing drone state
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), 
      std::bind(&PSDKBridge::publish_drone_state, this));
    
    RCLCPP_INFO(this->get_logger(), "PSDK Bridge Node initialized");
  }

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<communication::msg::PSDKData>::SharedPtr psdk_data_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<communication::msg::PSDKCmd>::SharedPtr psdk_cmd_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Drone state variables
  sensor_msgs::msg::Imu current_imu_;
  sensor_msgs::msg::NavSatFix current_gps_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_twist_;
  
  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_imu_ = *msg;
  }
  
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    current_gps_ = *msg;
  }
  
  void psdk_cmd_callback(const communication::msg::PSDKCmd::SharedPtr msg)
  {
    // Handle PSDK commands and convert to Gazebo control commands
    RCLCPP_INFO(this->get_logger(), "Received PSDK command: %d", msg->command_type);
    
    // Example: Convert takeoff command to Gazebo control
    if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_TAKEOFF)
    {
      RCLCPP_INFO(this->get_logger(), "Takeoff command received");
      // Implement takeoff logic here
    }
    else if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_LAND)
    {
      RCLCPP_INFO(this->get_logger(), "Land command received");
      // Implement land logic here
    }
    else if (msg->command_type == communication::msg::PSDKCmd::CMD_TYPE_MOVE)
    {
      RCLCPP_INFO(this->get_logger(), "Move command received: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
                msg->target_x, msg->target_y, msg->target_z, msg->target_yaw);
      // Implement move logic here
    }
    // Add handling for other command types
  }
  
  void publish_drone_state()
  {
    // Create PSDK data message and populate with current state
    auto psdk_data = communication::msg::PSDKData();
    
    // System state
    psdk_data.system_ready = true;  // Example: ready
    psdk_data.is_flying = false;  // Example: not flying
    psdk_data.flight_stage = 1;  // Example: ready to take off
    
    // GPS data
    psdk_data.latitude = current_gps_.latitude;
    psdk_data.longitude = current_gps_.longitude;
    psdk_data.altitude = current_gps_.altitude;
    psdk_data.num_satellites = 10;  // Example value
    
    // IMU data
    psdk_data.imu_angular_velocity_x = current_imu_.angular_velocity.x;
    psdk_data.imu_angular_velocity_y = current_imu_.angular_velocity.y;
    psdk_data.imu_angular_velocity_z = current_imu_.angular_velocity.z;
    psdk_data.imu_linear_acceleration_x = current_imu_.linear_acceleration.x;
    psdk_data.imu_linear_acceleration_y = current_imu_.linear_acceleration.y;
    psdk_data.imu_linear_acceleration_z = current_imu_.linear_acceleration.z;
    
    // Attitude data
    psdk_data.roll = 0.0;  // Example value
    psdk_data.pitch = 0.0;  // Example value
    psdk_data.yaw = 0.0;  // Example value
    
    // Battery data
    psdk_data.battery_voltage = 24.0;  // Example value
    psdk_data.battery_percentage = 80;  // Example value
    
    // Velocity data
    psdk_data.velocity_x = current_twist_.twist.linear.x;
    psdk_data.velocity_y = current_twist_.twist.linear.y;
    psdk_data.velocity_z = current_twist_.twist.linear.z;
    
    // Publish the data
    psdk_data_pub_->publish(psdk_data);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PSDKBridge>());
  rclcpp::shutdown();
  return 0;
}