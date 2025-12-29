# Robot Command Message
std_msgs/Header header

# Command type (enum)
uint8 COMMAND_TYPE_MOVE = 0
uint8 COMMAND_TYPE_ROTATE = 1
uint8 COMMAND_TYPE_GRAB = 2
uint8 COMMAND_TYPE_RELEASE = 3
uint8 COMMAND_TYPE_STOP = 4
uint8 COMMAND_TYPE_EMERGENCY_STOP = 5

uint8 command_type

# Movement command parameters
float64 target_x  # Target X position in meters
float64 target_y  # Target Y position in meters
float64 target_z  # Target Z position in meters
float64 movement_speed  # Movement speed in m/s

# Rotation command parameters
float64 target_roll  # Target roll angle in radians
float64 target_pitch  # Target pitch angle in radians
float64 target_yaw  # Target yaw angle in radians
float64 rotation_speed  # Rotation speed in rad/s

# Gripper command parameters
float64 gripper_force  # Gripper force in N
float64 gripper_duration  # Gripper action duration in seconds

# Emergency parameters
bool emergency_stop  # Emergency stop flag