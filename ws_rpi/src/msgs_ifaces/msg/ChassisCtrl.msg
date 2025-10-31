# Chassis Control Command Message
# Low-level control commands for the rover chassis

uint8 fdr_msg       # Front direction: 1=right, 2=straight, 3=left
float32 ro_ctrl_msg # Steering control value (0.0 to 1.0)
uint8 spd_msg       # Speed command (0-255)
uint8 bdr_msg       # Back direction: 0=stop, 1=forward, 2=backward