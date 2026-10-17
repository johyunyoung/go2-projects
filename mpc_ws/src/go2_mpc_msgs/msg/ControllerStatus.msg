# Controller-level status of go2_mpc, republished from its UDP debug stream.

std_msgs/Header header

uint8 PASSIVE=0
uint8 STAND=1
uint8 MPC=2
uint8 mode
string mode_name

uint64 seq                    # debug packet sequence number (gaps = dropped packets)
float64 controller_time       # [s] since go2_mpc started

float32 loop_period_ms        # measured control period (target 2.0)
float32 loop_compute_ms       # time spent in one control tick
float32 lowstate_age_ms       # age of the latest rt/lowstate

geometry_msgs/Twist cmd_vel   # remote velocity command (body frame: linear.x/y, angular.z)

bool ground_truth_valid       # base_pos/base_vel come from rt/sportmodestate
geometry_msgs/Vector3 base_lin_vel_world
geometry_msgs/Vector3 base_ang_vel_body
geometry_msgs/Vector3 base_lin_acc_body
geometry_msgs/Vector3 rpy

float32[12] tau_est           # SDK joint order FR, FL, RR, RL x hip, thigh, calf
float32[12] q_des
float32[12] dq_des
float32[12] kp
float32[12] kd
float32[12] tau_ff

# State estimation. base_* fields above are what the controller uses.
bool kalman_in_use                 # true: the controller runs on the Kalman filter
bool ground_truth_available        # simulator ground truth below is valid
geometry_msgs/Vector3 kf_pos       # Kalman filter, world
geometry_msgs/Vector3 kf_vel
geometry_msgs/Vector3 gt_pos       # simulator ground truth, world
geometry_msgs/Vector3 gt_vel
float32[4] contact_trust           # estimator foot trust, FR FL RR RL
