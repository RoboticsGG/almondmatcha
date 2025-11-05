# Chassis Sensors Message
# Contains motor encoder readings and battery status

int32 mt_lf_encode_msg  # Left motor encoder count
int32 mt_rt_encode_msg  # Right motor encoder count
float32 sys_current_msg # System battery current (Amperes)
float32 sys_volt_msg    # System battery voltage (Volts)