"""Real-log sim2real report for one walking run.

usage:
  python analyze.py --logs <export dir with velocity/ stand/> --run 2026-10-21_14-44-52 \
      --onnx <policy.onnx> [--xml <go2_rl_robotlab>/resources/go2/flat.xml] [--out result.json]

Steps: 1) policy replay check  2) actuator delay/gain fit  3) real (calm/rough) vs MuJoCo flat with the same commands.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from go2log import (G, arrays, command, fit_actuator, load, policy_steps, replay, segments, sim_replay, stats,
                    unitree_curve_exceeded, window_class)

ap = argparse.ArgumentParser()
ap.add_argument("--logs", required=True)
ap.add_argument("--run", required=True)
ap.add_argument("--onnx", required=True)
ap.add_argument("--xml", default=None, help="MuJoCo scene (flat.xml from go2_rl_robotlab). Sim step skipped if omitted.")
ap.add_argument("--out", default=None)
args = ap.parse_args()

res = {"run": args.run}
a = arrays(load(args.logs, args.run))
t = a["t"]
segs = segments(t)
cmd = command(a["joy"], a["fixed"])
print(f"run {args.run}: {len(t)} rows, policy segments", [(a["wall"][s], a["wall"][e - 1]) for s, e in segs])

# 1) policy replay
s, e = max(segs, key=lambda x: x[1] - x[0])
e = min(e, s + 40000)
act, pred = replay(a, s, e, args.onnx)
err = np.abs(pred - act)
res["replay"] = dict(corr=float(np.corrcoef(pred.ravel(), act.ravel())[0, 1]),
                     err_median=float(np.median(err)), err_p95=float(np.percentile(err, 95)))
print("1) policy replay:", res["replay"], "(corr > 0.99: obs/history/command pipeline matches training)")

# 2) actuator
mask = np.zeros(len(t), bool)
for s, e in segs:
    mask[s:e] = t[s:e] > t[s] + 1.0           # drop the 0.5 s start-up blend
mask &= np.abs(a["dq"]).max(1) > 0.5
idx = np.where(mask)[0]
sub = np.zeros_like(mask)
sub[idx[::3]] = True
delays = np.arange(0.0, 0.0205, 0.0005)
R2, C = fit_actuator(a, sub, delays)
kb = R2.mean(1).argmax()
res["actuator"] = dict(delay_ms=float(1000 * delays[kb]), r2=float(R2[kb].mean()),
                       kp_scale={g: float(C[kb, js, 0].mean()) for g, js in G.items()},
                       kd_scale={g: float(C[kb, js, 1].mean()) for g, js in G.items()})
print("2) actuator:", res["actuator"])
for gname, js in G.items():
    over = unitree_curve_exceeded(a["dq"][:, js], a["tau"][:, js]).mean() * 100
    print(f"   {gname:5s} |tau| max {np.abs(a['tau'][:, js]).max():5.1f} Nm, time above training curve {over:5.2f} %")

# 3) real vs sim
i = np.clip(np.searchsorted(t, t - res["actuator"]["delay_ms"] / 1000, side="right") - 1, 0, None)
cls = window_class(t, a["rpy"])
mov = np.abs(cmd[:, 0]) > 0.3
res["terrain_share"] = {k: float(np.mean(cls[mov] == c)) for k, c in [("calm", 0), ("rough", 1), ("tilted", 2)]}
R = {"real_calm": stats(t, a["q_des"][i], a["q"], a["dq"], a["tau"], a["rpy"], a["gyro"], cmd[:, 0], cls == 0),
     "real_rough": stats(t, a["q_des"][i], a["q"], a["dq"], a["tau"], a["rpy"], a["gyro"], cmd[:, 0], cls == 1)}
if args.xml:
    st = policy_steps(a["q_des"])
    cmds = command(a["joy"][st - 1], a["fixed"][st - 1])
    for act_name in ["isaac", "sim2real"]:
        o = sim_replay(args.xml, args.onnx, cmds, act_name, 1)
        R[f"sim_flat_{act_name}"] = stats(o[:, 0], o[:, 1:13], o[:, 13:25], o[:, 25:37], o[:, 37:49], o[:, 49:52],
                                          o[:, 52:55], o[:, 55], np.ones(len(o), bool))
res["compare"] = R
print("3) moving-time share by window class:", {k: round(v, 2) for k, v in res["terrain_share"].items()})
keys = ["seconds", "gait_hz", "gyro_rms", "pitch_std_deg", "calf_dq_p99", "calf_tau_p99", "power_p99_w",
        "calf_track_rms", "calf_over_train_curve_pct"]
for b in ["slow", "mid", "fast"]:
    print(f"\n[{b}]" + "".join(f"{k:>22s}" for k in R))
    for key in keys:
        print(f"  {key:26s}" + "".join(f"{R[k].get(b, {}).get(key, float('nan')):22.2f}" for k in R))
if args.out:
    json.dump(res, open(args.out, "w"), indent=1)
