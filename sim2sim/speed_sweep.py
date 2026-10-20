"""Flat-ground speed sweep (headless MuJoCo): same obs/action path as deploy_go2.py, heading hold, kp 25.
For each commanded vx: ramp 0 -> vx over 1 s, hold for --hold s, report achieved speed, tilt, height, fall.
usage: speed_sweep.py --policy <pt> [--vx 0.5 1.0 ...] [--hold 15] [--xml flat.xml]"""
import sys, math
from argparse import ArgumentParser
from pathlib import Path
sys.path.append(str(Path(__file__).parent))
import numpy as np, mujoco, torch
torch.set_num_threads(1)
from utils import load_config, pd_control, set_initial_state, ROOT_DIR
from deploy_go2 import build_features, build_single_obs, action_to_target

p = ArgumentParser()
p.add_argument("--policy", required=True)
p.add_argument("--xml", default="flat.xml")
p.add_argument("--vx", type=float, nargs="+", default=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
p.add_argument("--hold", type=float, default=15.0)
p.add_argument("--kp", type=float, default=25.0)
p.add_argument("--seeds", type=int, default=1)
p.add_argument("--friction", type=float, default=None, help="override floor friction (default scene value)")
args = p.parse_args()

cfg = load_config("go2.yaml"); cfg.kps = np.full(cfg.num_actions, args.kp, dtype=np.float32)
layout = [("ang_vel", 3), ("gravity", 3), ("cmd", 3), ("joint_pos", cfg.num_actions), ("joint_vel", cfg.num_actions), ("last_action", cfg.num_actions)]
m = mujoco.MjModel.from_xml_path(str(ROOT_DIR / "resources/go2" / args.xml)); m.opt.timestep = cfg.dt
if args.friction is not None:
    m.geom_friction[:, 0] = args.friction  # all geoms (MuJoCo combines pair friction with max)
d = mujoco.MjData(m)
ctrl_dt = cfg.dt * cfg.decimation


def yaw_of(q):
    w, x, y, z = q; return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(vx_cmd, seed):
    np.random.seed(seed)
    mujoco.mj_resetData(m, d); set_initial_state(d, cfg.base_init_pos, cfg.base_init_quat, cfg.default_angles); mujoco.mj_forward(m, d)
    policy = torch.jit.load(args.policy); action = np.zeros(cfg.num_actions, np.float32); target = cfg.default_angles.copy()
    cmd = np.zeros(3, np.float32); heading = 0.0
    n_settle, n_ramp, n_hold = int(2.0 / ctrl_dt), int(1.0 / ctrl_dt), int(args.hold / ctrl_dt)
    rec = []; fell = False; t_fall = None
    for k in range(n_settle + n_ramp + n_hold):
        if k < n_settle: cmd[0] = 0.0
        elif k < n_settle + n_ramp: cmd[0] = vx_cmd * (k - n_settle + 1) / n_ramp
        else: cmd[0] = vx_cmd
        for _ in range(cfg.decimation):
            d.ctrl[:] = pd_control(target, d.qpos[7:], cfg.kps, np.zeros_like(target), d.qvel[6:], cfg.kds); mujoco.mj_step(m, d)
        err = (heading - yaw_of(d.qpos[3:7]) + math.pi) % (2 * math.pi) - math.pi
        cmd[2] = float(np.clip(1.0 * err, -1.0, 1.0))
        obs = build_single_obs(build_features(d, action, cmd, cfg), layout)
        with torch.no_grad(): action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()[cfg.idx_model2mj]
        target = action_to_target(action, cfg)
        # metrics
        R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); R = R.reshape(3, 3)
        g_b = R.T @ np.array([0, 0, -1.0]); v_b = R.T @ d.qvel[:3]
        w, x, y, z = d.qpos[3:7]; roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))); pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
        if g_b[2] > -0.5 and not fell: fell = True; t_fall = k * ctrl_dt
        if k >= n_settle + n_ramp + int(2.0 / ctrl_dt): rec.append((v_b[0], d.qpos[2], roll, pitch, d.qpos[1]))
        if fell: break
    a = np.array(rec) if rec else np.zeros((1, 5))
    return dict(vx=vx_cmd, fell=fell, t_fall=t_fall, v_mean=a[:, 0].mean(), v_std=a[:, 0].std(), z=a[:, 1].mean(),
                roll_max=np.abs(a[:, 2]).max(), pitch_max=np.abs(a[:, 3]).max(), pitch_mean=a[:, 3].mean(), y_end=a[-1, 4], x_end=d.qpos[0])


print(f"policy {args.policy}  scene {args.xml}  friction {m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor'),0]:.2f}  hold {args.hold}s  (stats over hold minus first 2 s)")
print(f"{'vx_cmd':>6} {'v_mean':>7} {'v_std':>6} {'ratio':>6} {'z':>5} {'|roll|max':>9} {'|pitch|max':>10} {'pitch_mean':>10} {'y_end':>6}  result")
for vx in args.vx:
    for s in range(args.seeds):
        r = run(vx, s)
        res = f"FELL at {r['t_fall']:.1f}s" if r["fell"] else "ok"
        print(f"{vx:6.2f} {r['v_mean']:7.2f} {r['v_std']:6.2f} {r['v_mean']/vx:6.2f} {r['z']:5.2f} {r['roll_max']:9.1f} {r['pitch_max']:10.1f} {r['pitch_mean']:10.1f} {r['y_end']:6.2f}  {res}", flush=True)
