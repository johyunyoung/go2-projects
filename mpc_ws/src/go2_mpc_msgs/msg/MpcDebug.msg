# SRB-MPC internals, republished from go2_mpc's UDP debug stream (Mpc mode only).

std_msgs/Header header

bool has_solution
int8 status                   # HPIPM status of the last solve (0 = success)
uint16 iterations
float32 solve_ms              # problem build + solve
float32 age_ms                # age of the solution being applied
uint32 solves
uint32 failures

# x = [roll, pitch, yaw, p_com (world), omega (world), v_com (world)]
float64[12] state
float64[12] reference

float64[12] grf               # ground reaction forces applied, world frame, FR FL RR RL
float64[12] foot_world        # foot positions, world frame, FR FL RR RL

float64 cmd_dz                # body pose command (offsets)
float64 cmd_roll
float64 cmd_pitch
float64 cmd_yaw
