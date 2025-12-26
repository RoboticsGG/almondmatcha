# Aggregated rover monitoring status

# Mission info
bool mission_active
float64 distance_remaining

# Position info
float32 current_latitude
float32 current_longitude
float32 destination_latitude
float32 destination_longitude

# RTK GNSS info
float64 rtk_latitude
float64 rtk_longitude
float64 rtk_altitude
string rtk_fix_quality
float32 rtk_centimeter_error
int32 rtk_satellites

# Control info
uint8 fdr_msg
float32 ro_ctrl_msg
uint8 spd_msg
uint8 bdr_msg
