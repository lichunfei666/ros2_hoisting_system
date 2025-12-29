# Common data frame structure for all messages
std_msgs/Header header

# Frame identification
uint8 frame_id
uint8 message_type
uint16 sequence_number

# Data validity flag
bool is_valid

# Timestamp from source
float64 source_timestamp