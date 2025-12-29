# PSDK Data Message
std_msgs/Header header

# System state
bool system_ready
bool is_flying
string flight_mode  # manual, assisted_takeoff, gps_atti, gps_vel, gps_pos, altitude_hold, hotpoint, waypoint, follow_me, auto_landing, motor_off
uint8 flight_stage  # 0: Unknown, 1: Ready to take off, 2: Taking off, 3: In flight, 4: Landing, 5: Landed

# GPS data
bool gps_valid
float64 latitude  # degrees
float64 longitude  # degrees
float64 altitude  # meters above sea level
float64 height_above_ground  # meters above ground
uint8 num_satellites

# IMU data (angular velocities)
float64 imu_angular_velocity_x  # rad/s
float64 imu_angular_velocity_y  # rad/s
float64 imu_angular_velocity_z  # rad/s

# IMU data (linear accelerations)
float64 imu_linear_acceleration_x  # m/s^2
float64 imu_linear_acceleration_y  # m/s^2
float64 imu_linear_acceleration_z  # m/s^2

# Attitude data
float64 roll  # radians
float64 pitch  # radians
float64 yaw  # radians

# Velocity data
float64 velocity_x  # m/s (north)
float64 velocity_y  # m/s (east)
float64 velocity_z  # m/s (down)

# Battery information
float64 battery_voltage  # V
float64 battery_percentage  # 0-100%
float64 remaining_flight_time  # minutes

# Connection status
float64 signal_quality  # 0-1
uint8 connection_type  # 0: USB, 1: WiFi, 2: Serial

# Error information
string error_message
uint32 error_code