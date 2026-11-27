# Per-motor MIT setpoint / hold / zero-torque command.
uint8 MIT=0
uint8 HOLD=1
uint8 ZERO_TORQUE=2

uint8 command
int32 can_id
float64 kp
float64 kd
float64 q
float64 dq
float64 tau
float64 timeout
