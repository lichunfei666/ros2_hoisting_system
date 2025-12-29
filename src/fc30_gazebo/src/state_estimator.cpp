#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class StateEstimator : public rclcpp::Node
{
public:
  StateEstimator() : Node("state_estimator")
  {
    // Create subscribers
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/fc30/imu", 10, std::bind(&StateEstimator::ImuCallback, this, std::placeholders::_1));

    gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fc30/gps/fix", 10, std::bind(&StateEstimator::GpsCallback, this, std::placeholders::_1));

    // Create publishers
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fc30/pose_estimate", 10);
    twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fc30/twist_estimate", 10);

    // Initialize state variables
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    // Initialize filter parameters
    gps_filter_gain = 0.1;
    imu_filter_gain = 0.9;

    // Initialize time variables
    last_time = this->now();

    // Create timer for publishing estimated state
    timer = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&StateEstimator::PublishState, this));

    RCLCPP_INFO(this->get_logger(), "State Estimator node initialized");
  }

  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    // Update orientation from IMU
    pose.orientation = msg->orientation;

    // Update angular velocity from IMU
    twist.angular.x = msg->angular_velocity.x;
    twist.angular.y = msg->angular_velocity.y;
    twist.angular.z = msg->angular_velocity.z;

    // Estimate linear acceleration in world frame
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);

    // Convert body frame acceleration to world frame
    tf2::Vector3 accel_body(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    tf2::Vector3 accel_world = m * accel_body;

    // Subtract gravity
    accel_world.setZ(accel_world.z() - 9.81);

    // Update velocity using acceleration
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    // Integrate acceleration to get velocity
    twist.linear.x += accel_world.x() * dt * imu_filter_gain;
    twist.linear.y += accel_world.y() * dt * imu_filter_gain;
    twist.linear.z += accel_world.z() * dt * imu_filter_gain;

    // Integrate velocity to get position (IMU-based estimate)
    double imu_pos_x = pose.position.x + twist.linear.x * dt * imu_filter_gain;
    double imu_pos_y = pose.position.y + twist.linear.y * dt * imu_filter_gain;
    double imu_pos_z = pose.position.z + twist.linear.z * dt * imu_filter_gain;

    // Apply low-pass filter to position (combine with previous estimate)
    pose.position.x = pose.position.x * (1 - imu_filter_gain) + imu_pos_x * imu_filter_gain;
    pose.position.y = pose.position.y * (1 - imu_filter_gain) + imu_pos_y * imu_filter_gain;
    pose.position.z = pose.position.z * (1 - imu_filter_gain) + imu_pos_z * imu_filter_gain;
  }

  void GpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    // Check if GPS data is valid
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    {
      // Convert GPS coordinates to local coordinates (UTM)
      // For simplicity, we'll use a flat Earth approximation here
      // In a real application, you would use a proper coordinate transformation
      double gps_x = msg->longitude * 111319.9; // Approximate meters per degree of longitude at equator
      double gps_y = msg->latitude * 110574.0;  // Approximate meters per degree of latitude
      double gps_z = msg->altitude;

      // Apply low-pass filter to combine GPS and IMU estimates
      pose.position.x = pose.position.x * (1 - gps_filter_gain) + gps_x * gps_filter_gain;
      pose.position.y = pose.position.y * (1 - gps_filter_gain) + gps_y * gps_filter_gain;
      pose.position.z = pose.position.z * (1 - gps_filter_gain) + gps_z * gps_filter_gain;

      // Update velocity from GPS (simple finite difference)
      rclcpp::Time current_time = this->now();
      double dt = (current_time - last_gps_time).seconds();

      if (dt > 0.0 && dt < 1.0) // Valid time difference
      {
        twist.linear.x = (gps_x - last_gps_x) / dt * gps_filter_gain + twist.linear.x * (1 - gps_filter_gain);
        twist.linear.y = (gps_y - last_gps_y) / dt * gps_filter_gain + twist.linear.y * (1 - gps_filter_gain);
        twist.linear.z = (gps_z - last_gps_z) / dt * gps_filter_gain + twist.linear.z * (1 - gps_filter_gain);

        last_gps_x = gps_x;
        last_gps_y = gps_y;
        last_gps_z = gps_z;
        last_gps_time = current_time;
      }
    }
  }

  void PublishState()
  {
    // Create pose message
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = this->now();
    pose_msg->header.frame_id = "world";
    pose_msg->pose = pose;

    // Create twist message
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = "world";
    twist_msg->twist = twist;

    // Publish messages
    pose_pub->publish(*pose_msg);
    twist_pub->publish(*twist_msg);
  }

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;

  // State variables
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;

  // Filter parameters
  double gps_filter_gain;
  double imu_filter_gain;

  // Time variables
  rclcpp::Time last_time;
  rclcpp::Time last_gps_time;

  // GPS history variables
  double last_gps_x;
  double last_gps_y;
  double last_gps_z;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);

  // Create state estimator object
  auto state_estimator = std::make_shared<StateEstimator>();

  // Spin ROS node
  rclcpp::spin(state_estimator);

  // Shutdown
  rclcpp::shutdown();

  return 0;
}