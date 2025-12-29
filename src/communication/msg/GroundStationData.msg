# Ground Station Data Message
std_msgs/Header header

# System status
bool system_ready
bool emergency_stop
uint8 operating_mode  # 0: Manual, 1: Auto, 2: Semi-auto

# Control commands
float64 hoist_target_height  # Target height for hoist in meters
float64 hoist_target_speed   # Target speed for hoist in m/s
uint8 hoist_command          # 0: Stop, 1: Up, 2: Down

# Vehicle commands
float64 vehicle_target_x     # Target x position in meters
float64 vehicle_target_y     # Target y position in meters
float64 vehicle_target_z     # Target z position in meters
float64 vehicle_target_yaw   # Target yaw in radians
uint8 vehicle_command        # 0: Hover, 1: Move, 2: Return

# Safety parameters
float64 max_hoist_height
float64 min_hoist_height
float64 max_hoist_speed