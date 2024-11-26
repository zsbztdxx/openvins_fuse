uint8 POS_STATUS_NONE = 0 
uint8 POS_STATUS_SINGLE = 1
uint8 POS_STATUS_DIFF = 2
uint8 POS_STATUS_PPS = 3
uint8 POS_STATUS_FIX = 4
uint8 POS_STATUS_FLOAT = 5
uint8 POS_STATUS_RECKON = 6
uint8 POS_STATUS_INPUT = 7
uint8 POS_STATUS_PPP = 8

CsjwGnssHeader header
uint8 pos_status
uint8 header_status
float64 latitude
float64 longitude
float32 altitude
float32 vel_east
float32 vel_north
float32 vel_up
float32 pitch
float32 roll
float32 yaw
float32 std_latitude
float32 std_longitude
float32 std_altitude
float32 std_vel_east
float32 std_vel_north
float32 std_vel_up
float32 std_pitch
float32 std_roll
float32 std_yaw
float32 sat_num_master
float32 sat_num_slaver
float32 diff_age
