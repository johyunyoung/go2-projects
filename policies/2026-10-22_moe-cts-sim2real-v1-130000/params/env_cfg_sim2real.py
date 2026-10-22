"""Go2 MoE-CTS config adjusted from real-robot logs (2026-10-21, go2-projects sim2real/).

Changes relative to Go2EnvCfg, each backed by the real-log analysis:
1. Calf actuator split. Real calf torque-speed envelope ~1.5x hip/thigh (p99.9 33-36 Nm at low speed,
   max 62 Nm); training used the 23.4 Nm hip/thigh curve for every joint.
2. Base mass / CoM shifted for the Jetson + router payload (real static calf load +7 %, front-heavy).
3. Leg disturbance forces and more rough terrain to approximate grass (69 % of real walking time,
   3x pitch oscillation and 2x power vs flat).
4. Fine-tuning safety: curricula keyed on env.common_step_counter restart at 0 on --resume, so command
   ranges and reward weights are pinned to their final values.
Unchanged on purpose: actuator delay (measured ~7 ms, inside 0-20 ms), kp/kd (within +-10 %), obs noise.
"""
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import robot_lab.tasks.go2.mdp as mdp
from robot_lab.assets.unitree_actuator import UnitreeActuatorCfg

from .env_cfg import Go2EnvCfg


@configclass
class Go2Sim2RealEnvCfg(Go2EnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # 1. actuators: hip/thigh keep the Go2HV curve, calf gets the measured stronger curve
        common = dict(stiffness=25.0, damping=0.5, friction=0.01, min_delay=0, max_delay=4)
        self.scene.robot.actuators = {
            "GO2HV_hip_thigh": UnitreeActuatorCfg(
                joint_names_expr=[".*_hip_joint", ".*_thigh_joint"],
                X1=13.5, X2=30.0, Y1=20.2, Y2=23.4, **common),
            "GO2HV_calf": UnitreeActuatorCfg(
                joint_names_expr=[".*_calf_joint"],
                X1=9.0, X2=30.0, Y1=30.3, Y2=35.1, **common),
        }

        # 2. payload: heavier and front-heavy
        self.events.randomize_rigid_body_mass_base.params["mass_distribution_params"] = (-0.5, 2.0)
        self.events.randomize_com_positions.params["com_range"] = {
            "x": (-0.02, 0.05), "y": (-0.03, 0.03), "z": (-0.03, 0.03)}

        # 3. grass-like disturbances: random forces on the lower legs, resampled every 0.5-2 s
        self.events.leg_disturbance = EventTerm(
            func=mdp.apply_external_force_torque,
            mode="interval",
            interval_range_s=(0.5, 2.0),
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_calf"),
                "force_range": (-6.0, 6.0),
                "torque_range": (-0.5, 0.5),
            },
        )
        subs = self.scene.terrain.terrain_generator.sub_terrains
        subs["flat"].proportion = 0.08
        subs["rough_slope"].proportion = 0.12

        # 4. pin curricula to their final values (they restart at iteration 0 on resume)
        cmd = self.commands.base_velocity
        cmd.ranges.lin_vel_x = [-2.0, 2.0]
        cmd.ranges.lin_vel_y = [-1.0, 1.0]
        cmd.ranges.ang_vel_yaw = [-2.0, 2.0]
        cmd.command_range_curriculum = []
        cmd.zero_command_curriculum = {"start_iter": 0, "end_iter": 1, "start_value": 0.1, "end_value": 0.1}
        self.curriculum.base_linear_velocity = None
        self.curriculum.base_height_l2 = None
        self.rewards.lin_vel_z_l2.weight = 0.0
        self.rewards.base_height_l2.weight = -10.0
