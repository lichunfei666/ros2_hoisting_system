# Drone Command Message
std_msgs/Header header

# Command type (enum)
uint8 COMMAND_TYPE_TAKEOFF = 0
uint8 COMMAND_TYPE_LAND = 1
uint8 COMMAND_TYPE_MOVE = 2
uint8 COMMAND_TYPE_HOVER = 3
uint8 COMMAND_TYPE_RETURN_HOME = 4
uint8 COMMAND_TYPE_SET_MODE = 5
uint8 COMMAND_TYPE_EMERGENCY_STOP = 6

uint8 command_type

# Position command parameters
float64 target_x  # Target X coordinate in meters (north)
float64 target_y  # Target Y coordinate in meters (east)
float64 target_z  # Target Z coordinate in meters (altitude)
float64 target_yaw  # Target yaw angle in radians
float64 speed  # Movement speed in m/s

# Mode command parameters
string flight_mode  # Flight mode (manual, gps_atti, gps_pos, etc.)

# Emergency parameters
bool emergency_stop  # Emergency stop flag

# Takeoff/Land parameters
float64 takeoff_height  # Takeoff height in meters
float64 landing_speed  # Landing speed in m/s