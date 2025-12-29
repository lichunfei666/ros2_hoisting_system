#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <chrono>

using namespace std::chrono_literals;
using namespace Eigen;

class TrajectoryPlanner : public rclcpp::Node
{
private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    
    // State
    geometry_msgs::msg::PoseStamped current_pose_;
    std::vector<Vector3d> waypoints_;
    bool is_trajectory_executing_ = false;
    double trajectory_progress_ = 0.0;
    double trajectory_duration_ = 5.0; // seconds
    rclcpp::Time trajectory_start_time_;
    
    // Trajectory parameters
    double max_velocity_ = 1.0; // m/s
    double max_acceleration_ = 0.5; // m/s²

public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        // Initialize publishers
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fc30/target_pose", 10);
        
        trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/fc30/trajectory_marker", 10);
        
        // Initialize subscribers
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/fc30/waypoint", 10, 
            std::bind(&TrajectoryPlanner::waypoint_callback, this, std::placeholders::_1));
        
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fc30/pose_estimate", 10, 
            std::bind(&TrajectoryPlanner::pose_callback, this, std::placeholders::_1));
        
        // Initialize timer
        trajectory_timer_ = this->create_wall_timer(
            10ms, std::bind(&TrajectoryPlanner::execute_trajectory, this));
        
        RCLCPP_INFO(this->get_logger(), "Trajectory Planner Node initialized");
    }
    
    ~TrajectoryPlanner() {}
    
private:
    void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        Vector3d waypoint(msg->x, msg->y, msg->z);
        waypoints_.push_back(waypoint);
        
        RCLCPP_INFO(this->get_logger(), "Waypoint added: (%.2f, %.2f, %.2f)", msg->x, msg->y, msg->z);
        
        // If this is the first waypoint, add current position as start point
        if (waypoints_.size() == 1)
        {
            Vector3d start_point(
                current_pose_.pose.position.x,
                current_pose_.pose.position.y,
                current_pose_.pose.position.z);
            waypoints_.insert(waypoints_.begin(), start_point);
            
            // Start executing trajectory
            start_trajectory();
        }
        
        // Publish trajectory marker
        publish_trajectory_marker();
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }
    
    void start_trajectory()
    {
        if (waypoints_.size() < 2) return;
        
        is_trajectory_executing_ = true;
        trajectory_progress_ = 0.0;
        trajectory_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution");
    }
    
    void execute_trajectory()
    {
        if (!is_trajectory_executing_) return;
        
        // Calculate elapsed time
        rclcpp::Duration elapsed = this->now() - trajectory_start_time_;
        double t = elapsed.seconds();
        
        // Normalize time to [0, 1]
        trajectory_progress_ = std::min(t / trajectory_duration_, 1.0);
        
        // Generate trajectory point
        Vector3d current_point = generate_trajectory_point(trajectory_progress_);
        
        // Publish target pose
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.stamp = this->now();
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = current_point.x();
        target_pose.pose.position.y = current_point.y();
        target_pose.pose.position.z = current_point.z();
        // Keep current orientation
        target_pose.pose.orientation = current_pose_.pose.orientation;
        
        target_pose_pub_->publish(target_pose);
        
        // Check if trajectory is complete
        if (trajectory_progress_ >= 1.0)
        {
            is_trajectory_executing_ = false;
            waypoints_.clear();
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
        }
    }
    
    Vector3d generate_trajectory_point(double progress)
    {
        if (waypoints_.size() < 2) return Vector3d::Zero();
        
        // Simple cubic spline interpolation between first and last waypoint
        Vector3d start_point = waypoints_[0];
        Vector3d end_point = waypoints_[waypoints_.size() - 1];
        
        // Cubic polynomial: p(t) = a + bt + ct² + dt³
        // With boundary conditions: p(0) = start_point, p(1) = end_point
        // Velocity at start and end is zero
        Vector3d a = start_point;
        Vector3d b = Vector3d::Zero();
        Vector3d c = 3.0 * (end_point - start_point);
        Vector3d d = -2.0 * (end_point - start_point);
        
        double t = progress;
        Vector3d point = a + b * t + c * t * t + d * t * t * t;
        
        return point;
    }
    
    void publish_trajectory_marker()
    {
        if (waypoints_.empty()) return;
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        // Add waypoints to marker
        for (const auto& wp : waypoints_)
        {
            geometry_msgs::msg::Point p;
            p.x = wp.x();
            p.y = wp.y();
            p.z = wp.z();
            marker.points.push_back(p);
        }
        
        // Add interpolated points for smoother visualization
        for (double t = 0.0; t <= 1.0; t += 0.05)
        {
            Vector3d point = generate_trajectory_point(t);
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }
        
        trajectory_marker_pub_->publish(marker);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}