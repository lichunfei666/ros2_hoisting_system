# Hoist Data Message
std_msgs/Header header

# Hoist state
bool hoist_ready
bool hoist_error
uint8 hoist_status  # 0: Idle, 1: Moving Up, 2: Moving Down, 3: Error

# Position and speed
float64 current_height  # Current height in meters
float64 current_speed   # Current speed in m/s
float64 target_height   # Target height in meters
float64 target_speed    # Target speed in m/s

# Load information
float64 load_weight     # Load weight in kg
float64 tension         # Cable tension in N

# Motor information
float64 motor_temperature  # Motor temperature in °C
float64 motor_current      # Motor current in A
float64 battery_voltage    # Battery voltage in V

# Limit switches
bool upper_limit_hit
bool lower_limit_hit
bool emergency_stop_hit