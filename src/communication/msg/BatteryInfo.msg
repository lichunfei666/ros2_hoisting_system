# Battery Information Message
std_msgs/Header header

# Battery identification
uint8 battery_id  # Battery ID
bool available  # Whether the battery data is available

# Battery status
float64 voltage  # Battery voltage in V
float64 current  # Battery current in A
float64 percentage  # Battery level in %
float64 temperature  # Battery temperature in °C