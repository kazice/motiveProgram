Header header

uint8 msgid
uint8 opt_health_flag
uint8 network_health_flag
uint8 pendulum_health_flag
uint8 mavstate
uint8 takeoffstate
uint8 lost_data
float32 net_delay
float64 timer_10ms
float64 timer_20ms

float32[3] opt_pos
float32[3] des_pos
float32[3] err_pos
float32[3] u_pos
float32[3] esum_pos

float32[3] opt_velt
float32[3] filter_velt
float32[3] des_velt
float32[3] err_velt
float32[3] u_velt
float32[3] esum_velt

float32[3] opt_attitude
float32[3] des_attitude
float32[3] err_attitude
float32[3] u_attitude
float32[3] esum_attitude

float32[2] opt_attitude_pendulum
float32[2] des_attitude_pendulum
float32[2] err_attitude_pendulum
float32[2] u_attitude_pendulum
float32[2] esum_attitude_pendulum

float32[5] pendulum

float32[3] test_1
float32[3] test_2
float32[3] test_3

uint16[8] rc_controller
uint16[8] rc2mav


