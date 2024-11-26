uint8 TIME_SYS_UNKNOWN = 0
uint8 TIME_SYS_BDS = 1
uint8 TIME_SYS_GPS = 2
uint8 TIME_SYS_GAL = 3
uint8 TIME_SYS_GLO = 4  

std_msgs/Header header
uint8 time_system
uint32 week_num
float64 secs_in_week
int8 leap_sec