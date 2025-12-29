# Hook Command Message
std_msgs/Header header

# Action type
bool emergency_release  # Emergency release flag
float64 hook_force  # Hook force in N (if supported)
float64 hook_duration  # Hook action duration in seconds (if supported)

# Hook action commands
uint8 ACTION_OPEN = 0
uint8 ACTION_CLOSE = 1
uint8 ACTION_RELEASE = 2
uint8 ACTION_LOCK = 3

uint8 action_type  # The action to perform