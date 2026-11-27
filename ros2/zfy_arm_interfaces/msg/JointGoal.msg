uint8 MOVE=0
uint8 APPEND=1
uint8 DIRECT=2
uint8 STOP=3
uint8 GAINS=4
uint8 DRAG_ON=5
uint8 DRAG_OFF=6

uint8 command
float64 duration
float64[] position
#: DIRECT 指令的可选速度前馈（rad/s，URDF 方向）。空 = 无速度。
#: POS_VEL 回放会带上它；MIT 仅在驱动 velocity_feedforward=true 时使用。
float64[] velocity
float64 kp
float64 kd
bool gravity
float64 gravity_scale
bool friction
float64 coulomb
float64 viscous
float64 friction_slope
float64 dwell
