"""Full FSM sequence in MuJoCo: lying (Passive) -> stand-up -> policy (blend 0.5 s; stand 3 s, walk 0.5 m/s 3 s, stand 2 s)
-> sit-down -> Passive.  --mode current : author's FixStand / FixSquat (instant kp 60/80, q0 = lowcmd q, 1.36/-2.65 crouch)
                         --mode replay  : recorded sport-mode StandUp / StandDown keyframes (sport_motion.npz) with
                                          gain ramp, q0 = measured q, and a lead-in blend onto the recorded start pose.
Reports per phase: torque jump at entry, peak |tau| hip/thigh/calf, peak |dq|, max |pitch|, min head z, and the tracking
error against the recording; for the final drop into Passive the peak base vertical speed."""
import sys, math
from argparse import ArgumentParser
from pathlib import Path
sys.path.append(str(Path(__file__).parent))
import numpy as np, mujoco, torch
torch.set_num_threads(1)
from utils import load_config, set_initial_state, ROOT_DIR
from deploy_go2 import build_features, build_single_obs, action_to_target

p = ArgumentParser()
p.add_argument("--policy", required=True); p.add_argument("--mode", default="replay"); p.add_argument("--xml", default="flat.xml")
p.add_argument("--npz", required=True); p.add_argument("--kp", type=float, nargs=3, default=[60, 80, 80])
p.add_argument("--kd", type=float, nargs=3, default=[5, 4, 4]); p.add_argument("--ramp", type=float, default=0.3)
p.add_argument("--lead", type=float, default=0.5, help="replay sit-down: blend from measured q onto the recorded start over this time")
p.add_argument("--q0", default="measured", help="sit-down start: measured q, or cmd (last commanded target)")
p.add_argument("--ff", action="store_true", help="replay: add recorded tau_est as feed-forward torque and finite-difference dq as velocity target")
p.add_argument("--config", default="", help="--mode config: read FixStand/FixSquat from this unitree_cpp_deploy config (patched State_Fix semantics)")
p.add_argument("--kf_dt", type=float, default=0.04, help="keyframe spacing (State_Fix interpolates linearly between them)")
args = p.parse_args()
cfg = load_config("go2.yaml"); cfg.kps = np.full(12, 25.0, np.float32); cfg.kds = np.full(12, 0.5, np.float32)
layout = [("ang_vel", 3), ("gravity", 3), ("cmd", 3), ("joint_pos", 12), ("joint_vel", 12), ("last_action", 12)]
m = mujoco.MjModel.from_xml_path(str(ROOT_DIR / "resources/go2" / args.xml)); m.opt.timestep = cfg.dt; d = mujoco.MjData(m)
base = m.body("base_link").id if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link") >= 0 else m.body("base").id
LIM = np.array([23.7, 23.7, 45.4] * 4); DT = cfg.dt
SDK2MJ = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]              # MuJoCo FL FR RL RR  <->  SDK FR FL RR RL (self-inverse)
z = np.load(args.npz); UP = z["up"][:, SDK2MJ]; DOWN = z["down"][:, SDK2MJ]; UPT = z["up_tau"][:, SDK2MJ]; DOWNT = z["down_tau"][:, SDK2MJ]
KP = np.array(args.kp * 4, np.float32); KD = np.array(args.kd * 4, np.float32)
CROUCH = np.array([0.0, 1.36, -2.65] * 4, np.float32); STAND = np.array([0.0, 0.8, -1.5] * 4, np.float32)


def keyframes(traj, t0=0.0):
    idx = np.unique(np.r_[np.arange(0, len(traj), round(args.kf_dt * 500)), len(traj) - 1])
    return list(t0 + idx / 500.0), [traj[i] for i in idx]


def ff_at(t, ts, traj, taus, t0=0.0, r=1.0):
    """feed-forward torque / velocity at time t from the keyframes of traj (State_Fix-style linear interpolation)"""
    _, tk = keyframes(taus, t0); kt, qk = keyframes(traj, t0)
    dq = (interp(t + 1e-3, kt, qk) - interp(t - 1e-3, kt, qk)) / 2e-3 if kt[0] < t < kt[-1] else np.zeros(12)
    return r * interp(t, kt, tk), r * dq


def interp(t, ts, qs):
    if t <= ts[0]: return qs[0]
    if t >= ts[-1]: return qs[-1]
    k = np.searchsorted(ts, t) - 1; a = (t - ts[k]) / (ts[k + 1] - ts[k]); return qs[k] + a * (qs[k + 1] - qs[k])


class StateFix:
    """unitree_cpp_deploy State_Fix + sport-replay patch: q0 = lowcmd q at entry, linear q / tau keyframes,
    dq = slope of the current segment, kp/kd ramped from the gains at entry over gain_ramp_s"""
    def __init__(s, c):
        s.ts = np.array(c["ts"], float); s.kp = np.array(c["kp"], float)[SDK2MJ]; s.kd = np.array(c["kd"], float)[SDK2MJ]
        s.qs = [np.array(q, float)[SDK2MJ] if q else None for q in c["qs"]]
        s.taus = [np.array(t, float)[SDK2MJ] if t else np.zeros(12) for t in c.get("taus", [[]] * len(s.ts))]
        s.ramp = c.get("gain_ramp_s", 0.0); s.vff = c.get("velocity_ff", False)
    def enter(s):
        s.qs[0] = cmd_q.copy(); s.taus[0] = cmd_tau.copy(); s.kp0, s.kd0 = cmd_kp.copy(), cmd_kd.copy(); cmd_dq[:] = 0
        if s.ramp <= 0: cmd_kp[:], cmd_kd[:] = s.kp, s.kd
    def run(s, t):
        cmd_q[:] = interp(t, s.ts, s.qs)
        if s.ramp > 0: r = min(1.0, t / s.ramp); cmd_kp[:] = s.kp0 + r * (s.kp - s.kp0); cmd_kd[:] = s.kd0 + r * (s.kd - s.kd0)
        cmd_tau[:] = interp(t, s.ts, s.taus)
        if s.vff:
            k = int(np.searchsorted(s.ts, t, side="right") - 1)
            cmd_dq[:] = (s.qs[k + 1] - s.qs[k]) / (s.ts[k + 1] - s.ts[k]) if 0 <= k < len(s.ts) - 1 else 0.0
    def exit(s): cmd_dq[:] = 0   # torque left in lowcmd: the next state fades it (RLBase blend) or zeroes it (Passive)


def head_z():
    R = d.xmat[base].reshape(3, 3); return (d.xpos[base] + R @ np.array([0.28, 0, 0.0]))[2]
def pitch():
    w, x, y, zz = d.qpos[3:7]; return math.degrees(math.asin(max(-1, min(1, 2 * (w * y - zz * x)))))


class Phase:
    def __init__(s, name): s.name = name; s.tau = np.zeros(3); s.dq = 0; s.pitch = 0; s.head = 9; s.jump = None; s.err = []; s.vz = 0
    def log(s, tau, tau_prev, ref=None):
        if s.jump is None: s.jump = np.abs(tau - tau_prev).max()
        s.tau = np.maximum(s.tau, np.abs(tau).reshape(4, 3).max(0)); s.dq = max(s.dq, np.abs(d.qvel[6:]).max())
        s.pitch = max(s.pitch, abs(pitch())); s.head = min(s.head, head_z()); s.vz = max(s.vz, abs(d.qvel[2]))
        if ref is not None: s.err.append(np.abs(ref - d.qpos[7:]).max()); s.where = getattr(s, 'where', []) + [(np.abs(ref - d.qpos[7:]).max(), np.abs(ref - d.qpos[7:]).argmax(), len(s.err))]
    def show(s):
        e = f" | track err vs recording max {max(s.err):.3f} rad (joint {max(s.where)[1]}, t {max(s.where)[2]*DT:.2f}s), mean {np.mean(s.err):.3f}" if s.err else ""
        print(f"  {s.name:10} jump {s.jump:5.1f} Nm | peak tau hip/thigh/calf {s.tau[0]:4.1f}/{s.tau[1]:4.1f}/{s.tau[2]:4.1f} | "
              f"peak dq {s.dq:5.2f} | max pitch {s.pitch:4.1f} deg | min head z {s.head:.3f} | peak |vz| {s.vz:.2f} m/s{e}")


cmd_kp = np.zeros(12, np.float32); cmd_kd = np.full(12, 3.0, np.float32); cmd_q = np.zeros(12, np.float32)
cmd_dq = np.zeros(12, np.float32); cmd_tau = np.zeros(12, np.float32)
tau_prev = np.zeros(12)


def step(phase=None, ref=None):
    global tau_prev
    tau = np.clip(cmd_tau + cmd_kp * (cmd_q - d.qpos[7:]) + cmd_kd * (cmd_dq - d.qvel[6:]), -LIM, LIM)
    if phase: phase.log(tau, tau_prev, ref)
    tau_prev = tau; d.ctrl[:] = tau; mujoco.mj_step(m, d)


# 1. lying, Passive (kp 0, kd 3)
lie = UP[0].copy(); lie[2::3] = np.maximum(lie[2::3], -2.72)
mujoco.mj_resetData(m, d); set_initial_state(d, np.array([0, 0, 0.15]), np.array([1, 0, 0, 0.]), lie); mujoco.mj_forward(m, d)
for _ in range(int(2.0 / DT)): cmd_q[:] = d.qpos[7:]; step()
print(f"[{args.mode}] lying: base z {d.qpos[2]:.3f}, head z {head_z():.3f}")
phases = []

# 2. stand up
ph = Phase("stand-up"); phases.append(ph); q_meas = d.qpos[7:].copy(); kp0, kd0 = cmd_kp.copy(), cmd_kd.copy()
if args.mode == "config":
    import yaml; FSMC = yaml.safe_load(open(args.config))["FSM"]; fs = StateFix(FSMC["FixStand"]); fs.enter(); t = 0.0
    lead = FSMC["FixStand"]["ts"][1]
    while t < fs.ts[-1] + 0.7: fs.run(t); step(ph, interp(t - lead, *keyframes(UP)) if t > lead + 0.1 else None); t += DT
    fs.exit(); T = -1
elif args.mode == "current":
    ts, qs = [0, 1, 2], [q_meas, CROUCH, STAND]; cmd_kp[:], cmd_kd[:] = KP, KD; T = 3.0
else:
    ts, qs = keyframes(UP); qs[0] = q_meas; T = ts[-1] + 1.0
t = 0.0
while t < T:
    if args.mode != "current":
        r = min(1.0, t / args.ramp) if args.ramp > 0 else 1.0; cmd_kp[:] = kp0 + r * (KP - kp0); cmd_kd[:] = kd0 + r * (KD - kd0)
    cmd_q[:] = interp(t, ts, qs)
    if args.ff and args.mode != "current": cmd_tau[:], cmd_dq[:] = ff_at(t, ts, UP, UPT, 0.0, r)
    step(ph, interp(t, *keyframes(UP)) if args.mode != "current" and t > 0.1 else None); t += DT
if args.mode != "config": cmd_tau[:] = 0; cmd_dq[:] = 0

# 3. policy (startup blend 0.5 s from the previous command, warm-start last action)
ph = Phase("policy"); phases.append(ph)
policy = torch.jit.load(args.policy); q_en, kp_en, kd_en, tau_en = cmd_q.copy(), cmd_kp.copy(), cmd_kd.copy(), cmd_tau.copy()
action = ((d.qpos[7:] - cfg.default_angles) / cfg.action_scale).astype(np.float32) if hasattr(cfg, "action_scale") else \
         ((d.qpos[7:] - cfg.default_angles) / 0.25).astype(np.float32)
cmd = np.zeros(3, np.float32); t = 0.0; target = cmd_q.copy()
while t < 8.0:
    cmd[0] = 0.5 if 3.0 <= t < 6.0 else 0.0
    obs = build_single_obs(build_features(d, action, cmd, cfg), layout)
    with torch.no_grad(): action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()[cfg.idx_model2mj]
    target = action_to_target(action, cfg)
    for _ in range(cfg.decimation):
        a = min(1.0, t / 0.5)
        cmd_q[:] = (1 - a) * q_en + a * target; cmd_kp[:] = (1 - a) * kp_en + a * cfg.kps; cmd_kd[:] = (1 - a) * kd_en + a * cfg.kds; cmd_tau[:] = (1 - a) * tau_en
        step(ph); t += DT
    if d.qpos[2] < 0.15: print("  FELL during policy"); break
print(f"  policy stand: max |target - q| {np.abs(target - d.qpos[7:]).max():.3f} rad, base z {d.qpos[2]:.3f}")

# 4. sit down, then auto Passive
ph = Phase("sit-down"); phases.append(ph); q_meas = d.qpos[7:].copy(); kp0, kd0 = cmd_kp.copy(), cmd_kd.copy()
if args.mode == "config":
    fq = StateFix(FSMC["FixSquat"]); fq.enter(); t = 0.0; lead = FSMC["FixSquat"]["ts"][1]
    while t < fq.ts[-1]: fq.run(t); step(ph, interp(t - lead, *keyframes(DOWN)) if t > lead + 0.1 else None); t += DT
    fq.exit(); ts = [0.0]; rts = None; T = -1
elif args.mode == "current":
    ts, qs = [0, 1, 2], [cmd_q.copy(), STAND, CROUCH]; cmd_kp[:], cmd_kd[:] = KP, KD
    rts, rqs = None, None
else:
    rts, rqs = keyframes(DOWN, args.lead); ts, qs = [0.0] + rts, [q_meas if args.q0 == "measured" else cmd_q.copy()] + rqs
if args.mode != "config": T = ts[-1] + (0.0 if args.mode == "current" else 0.3)
t = 0.0
while t < T:
    if args.mode != "current":
        r = min(1.0, t / args.ramp) if args.ramp > 0 else 1.0; cmd_kp[:] = kp0 + r * (KP - kp0); cmd_kd[:] = kd0 + r * (KD - kd0)
    cmd_q[:] = interp(t, ts, qs)
    if args.ff and args.mode != "current":
        if t >= args.lead: cmd_tau[:], cmd_dq[:] = ff_at(t, ts, DOWN, DOWNT, args.lead)
        else: cmd_tau[:] = (t / args.lead) * DOWNT[0]; cmd_dq[:] = 0   # fade the hold torque in during the lead-in
    step(ph, interp(t, rts, rqs) if rts and t > args.lead + 0.1 else None); t += DT
if args.mode != "config": cmd_tau[:] = 0; cmd_dq[:] = 0

ph = Phase("->passive"); phases.append(ph); cmd_kp[:] = 0; cmd_kd[:] = 3.0; cmd_tau[:] = 0; cmd_dq[:] = 0; t = 0.0
while t < 1.5: cmd_q[:] = d.qpos[7:]; step(ph); t += DT
for ph in phases: ph.show()
print(f"  final: base z {d.qpos[2]:.3f}, head z {head_z():.3f}")
