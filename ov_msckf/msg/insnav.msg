uint8 NAV_STATUS_INS_INACTIVE = 0 
uint8 NAV_STATUS_INS_ALIGNING = 1
uint8 NAV_STATUS_INS_HIGH_VARIANCE = 2
uint8 NAV_STATUS_INS_SOLUTION_GOOD = 3
uint8 NAV_STATUS_INS_SOLUTION_FREE = 6
uint8 NAV_STATUS_INS_ALIGNMENT_COMPLETE = 7
uint8 NAV_STATUS_DETERMINING_ORIENTATION = 8
uint8 NAV_STATUS_WAITING_INITIALPOS = 9
uint8 NAV_STATUS_WAITING_AZIMUTH = 10
uint8 NAV_STATUS_INITIALIZING_BIASES = 11
uint8 NAV_STATUS_MOTION_DETECT = 12

uint8 POS_STATUS_NONE = 0 
uint8 POS_STATUS_FIXEDPOS = 1
uint8 POS_STATUS_SINGLE = 16
uint8 POS_STATUS_PSRDIFF = 17
uint8 POS_STATUS_NARROW_FLOAT = 34
uint8 POS_STATUS_NARROW_INT = 50
uint8 POS_STATUS_INS_PSRSP = 53
uint8 POS_STATUS_INS_PSRDIFF = 54
uint8 POS_STATUS_INS_RTKFLOAT = 55
uint8 POS_STATUS_INS_RTKFIXED = 56
uint8 POS_STATUS_INS_PPP = 69

CsjwGnssHeader header
uint8 nav_status
uint8 pos_status
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