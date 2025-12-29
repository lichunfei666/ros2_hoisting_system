# System State Message
std_msgs/Header header

# System identification
string system_name

# Operational mode
string mode  # manual, auto, semi_auto, emergency
string status  # normal, warning, error, critical

# Power information
float64 battery_percentage  # Battery level in % (deprecated - use battery1 or battery2 fields)
float64 battery_voltage  # Battery voltage in V (deprecated - use battery1 or battery2 fields)
float64 battery_current  # Battery current in A (deprecated - use battery1 or battery2 fields)

# Individual battery information
communication/BatteryInfo battery1  # Battery 1 information
communication/BatteryInfo battery2  # Battery 2 information

# Safety information
bool emergency_stop  # Emergency stop status
bool system_ready  # System ready status

# Error information
string error_message  # Error message if any
uint32 error_code  # Error code if any

# Performance metrics
float64 cpu_usage  # CPU usage in %
float64 memory_usage  # Memory usage in %
float64 system_temperature  # System temperature in °C