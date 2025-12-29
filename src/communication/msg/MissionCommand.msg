# Mission Command Message
std_msgs/Header header

# Mission type (enum)
uint8 MISSION_TYPE_TAKE_OFF = 0
uint8 MISSION_TYPE_LAND = 1
uint8 MISSION_TYPE_MOVE_TO = 2
uint8 MISSION_TYPE_FOLLOW_PATH = 3
uint8 MISSION_TYPE_HOVER = 4
uint8 MISSION_TYPE_RETURN_HOME = 5
uint8 MISSION_TYPE_CUSTOM = 6

uint8 mission_type
string mission_name  # Mission name for identification

# Mission parameters
float64 target_x  # Target X coordinate in meters
float64 target_y  # Target Y coordinate in meters
float64 target_z  # Target Z coordinate in meters
float64 target_yaw  # Target yaw angle in radians
float64 speed  # Mission speed in m/s

# Mission flags
bool start_mission  # Start the mission
bool pause_mission  # Pause the mission
bool cancel_mission  # Cancel the mission
bool emergency_stop  # Emergency stop flag

# Mission configuration
float64 mission_timeout  # Mission timeout in seconds
bool mission_repeat  # Repeat mission after completion
bool mission_preemptible  # Mission can be preempted by higher priority missions