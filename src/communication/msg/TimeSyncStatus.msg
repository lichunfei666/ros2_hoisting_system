# Time Synchronization Status Message
std_msgs/Header header

# Time synchronization status
bool pps_trigger_time_updated  # PPS trigger time updated flag
uint32 pps_trigger_time_sec     # PPS trigger time (seconds)
uint32 pps_trigger_time_nsec    # PPS trigger time (nanoseconds)
uint64 local_time_sec           # Local time (seconds)
uint64 local_time_nsec          # Local time (nanoseconds)