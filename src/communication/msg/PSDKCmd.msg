# PSDK Command Message
std_msgs/Header header

# Command type (enum)
uint8 CMD_TYPE_TAKEOFF = 0
uint8 CMD_TYPE_LAND = 1
uint8 CMD_TYPE_MOVE = 2
uint8 CMD_TYPE_HOVER = 3
uint8 CMD_TYPE_RETURN_HOME = 4
uint8 CMD_TYPE_HOOK_CONTROL = 5
uint8 CMD_TYPE_EMERGENCY_STOP = 6
uint8 CMD_TYPE_WAYPOINT_V3_UPLOAD = 7
uint8 CMD_TYPE_WAYPOINT_V3_START = 8
uint8 CMD_TYPE_WAYPOINT_V3_STOP = 9
uint8 CMD_TYPE_WAYPOINT_V3_PAUSE = 10
uint8 CMD_TYPE_WAYPOINT_V3_RESUME = 11

uint8 command_type

# Waypoint V3 parameters (for WAYPOINT_V3 commands)
string waypoint_file_path  # Path to the waypoint V3 kmz file

# Common command parameters
bool emergency  # Emergency command flag
float64 timeout  # Command timeout in seconds

# Position command parameters (for MOVE command)
float64 target_x  # Target X position in meters (north)
float64 target_y  # Target Y position in meters (east)
float64 target_z  # Target Z position in meters (altitude)
float64 target_yaw  # Target yaw angle in radians
float64 movement_speed  # Movement speed in m/s

# Hook control parameters (for HOOK_CONTROL command)
uint8 HOOK_ACTION_OPEN = 0
uint8 HOOK_ACTION_CLOSE = 1
uint8 HOOK_ACTION_RELEASE = 2

uint8 hook_action
float64 hook_force  # Hook force in N (if supported)
float64 hook_duration  # Hook action duration in seconds (if supported)
bool emergency_release  # Emergency release flag

# Takeoff/Land parameters
float64 takeoff_height  # Takeoff height in meters
float64 landing_speed  # Landing speed in m/s