"""Measure heading/lateral drift under a pure forward command (no heading hold), optionally with a command bias.
usage: drift_measure.py --policy <pt> [--vx 0.5 1.0 1.5 2.0] [--wz_bias 0] [--vy_bias 0] [--hold 12] [--xml flat.xml]"""
import sys, math
from argparse import ArgumentParser
from pathlib import Path
sys.path.append(str(Path(__file__).parent))
import numpy as np, mujoco, torch
torch.set_num_threads(1)
from utils import load_config, pd_control, set_initial_state, ROOT_DIR
from deploy_go2 import build_features, build_single_obs, action_to_target

p = ArgumentParser()
p.add_argument("--policy", required=True); p.add_argument("--xml", default="flat.xml")
p.add_argument("--vx", type=float, nargs="+", default=[0.5, 1.0, 1.5, 2.0])
p.add_argument("--wz_bias", type=float, default=0.0); p.add_argument("--vy_bias", type=float, default=0.0)
p.add_argument("--hold", type=float, default=12.0); p.add_argument("--kp", type=float, default=25.0)
p.add_argument("--mirror", action="store_true", help="also run with the robot initially rotated 180 deg (checks world-frame asymmetry)")
args = p.parse_args()
cfg = load_config("go2.yaml"); cfg.kps = np.full(cfg.num_actions, args.kp, dtype=np.float32)
layout = [("ang_vel", 3), ("gravity", 3), ("cmd", 3), ("joint_pos", cfg.num_actions), ("joint_vel", cfg.num_actions), ("last_action", cfg.num_actions)]
m = mujoco.MjModel.from_xml_path(str(ROOT_DIR / "resources/go2" / args.xml)); m.opt.timestep = cfg.dt; d = mujoco.MjData(m)
ctrl_dt = cfg.dt * cfg.decimation


def yaw_of(q):
    w, x, y, z = q; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(vx_cmd, yaw0=0.0):
    mujoco.mj_resetData(m, d)
    quat = np.array([math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)], dtype=np.float32)
    set_initial_state(d, cfg.base_init_pos, quat, cfg.default_angles); mujoco.mj_forward(m, d)
    policy = torch.jit.load(args.policy); action = np.zeros(cfg.num_actions, np.float32); target = cfg.default_angles.copy()
    cmd = np.array([0.0, args.vy_bias, args.wz_bias], np.float32)
    n_settle, n_ramp, n_hold = int(2.0 / ctrl_dt), int(1.0 / ctrl_dt), int(args.hold / ctrl_dt)
    rec = []; yaw_prev = None; yaw_unwrapped = 0.0
    for k in range(n_settle + n_ramp + n_hold):
        cmd[0] = 0.0 if k < n_settle else (vx_cmd * min(1.0, (k - n_settle + 1) / n_ramp))
        for _ in range(cfg.decimation):
            d.ctrl[:] = pd_control(target, d.qpos[7:], cfg.kps, np.zeros_like(target), d.qvel[6:], cfg.kds); mujoco.mj_step(m, d)
        obs = build_single_obs(build_features(d, action, cmd, cfg), layout)
        with torch.no_grad(): action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()[cfg.idx_model2mj]
        target = action_to_target(action, cfg)
        yaw = yaw_of(d.qpos[3:7])
        if yaw_prev is not None: yaw_unwrapped += (yaw - yaw_prev + math.pi) % (2 * math.pi) - math.pi
        yaw_prev = yaw
        if k >= n_settle + n_ramp + int(2.0 / ctrl_dt):
            R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); v_b = R.reshape(3, 3).T @ d.qvel[:3]
            rec.append((d.qvel[5], v_b[0], v_b[1], yaw_unwrapped))
        if (R := np.zeros(9)) is not None: mujoco.mju_quat2Mat(R, d.qpos[3:7])
        if (R.reshape(3, 3).T @ np.array([0, 0, -1.0]))[2] > -0.5: return None
    a = np.array(rec); dt_hold = len(rec) * ctrl_dt
    return dict(wz=a[:, 0].mean(), vx=a[:, 1].mean(), vy=a[:, 2].mean(), dyaw_deg=math.degrees(a[-1, 3] - a[0, 3]), T=dt_hold)


print(f"policy {Path(args.policy).parent.parent.name}/{Path(args.policy).name}  bias vy={args.vy_bias:+.2f} wz={args.wz_bias:+.2f}  (+ = left)")
print(f"{'vx_cmd':>6} {'vx':>5} {'vy(m/s)':>8} {'wz(deg/s)':>10} {'yaw change':>11}")
for vx in args.vx:
    for yaw0 in ([0.0, math.pi] if args.mirror else [0.0]):
        r = run(vx, yaw0)
        tag = "" if yaw0 == 0 else "  (start facing -x)"
        if r is None: print(f"{vx:6.2f}  FELL{tag}"); continue
        print(f"{vx:6.2f} {r['vx']:5.2f} {r['vy']:+8.3f} {math.degrees(r['wz']):+10.2f} {r['dyaw_deg']:+9.1f}° /{r['T']:.0f}s{tag}", flush=True)
