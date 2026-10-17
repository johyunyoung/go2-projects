#!/usr/bin/env python3
"""Kalman filter vs simulator ground truth, from a rosbag2 recording of /go2_mpc/status.

    python3 tools/estimator_report.py <bag dir> [script.yaml]

Prints velocity and height errors (overall and per script segment) and the
horizontal position drift over time. Run inside the go2-mpc container after
`source env.sh`.
"""
import argparse
import math

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message

from go2_mpc_msgs.msg import ControllerStatus

MODE_MPC = 2


def load(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/go2_mpc/status":
            s = deserialize_message(data, ControllerStatus)
            if s.ground_truth_available:
                out.append(s)
    return out


def errors(samples):
    ev = [(s.kf_vel.x - s.gt_vel.x, s.kf_vel.y - s.gt_vel.y, s.kf_vel.z - s.gt_vel.z) for s in samples]
    eh = [math.hypot(e[0], e[1]) for e in ev]
    ez = [abs(e[2]) for e in ev]
    dz = [abs(s.kf_pos.z - s.gt_pos.z) for s in samples]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
    return rms(eh), max(eh), rms(ez), max(ez), rms(dz), max(dz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("script", nargs="?")
    a = ap.parse_args()
    S = [s for s in load(a.bag) if s.mode == MODE_MPC]
    if not S:
        print("no Mpc-mode samples with ground truth")
        return
    print("Mpc mode, %d samples over %.1f s" % (len(S), S[-1].controller_time - S[0].controller_time))
    print("%-24s | %-23s | %-23s | %s" % ("", "|v_xy err| rms / max", "|v_z err| rms / max", "height err rms / max"))
    def row(name, W):
        r = errors(W)
        print("%-24s | %7.4f / %7.4f m/s   | %7.4f / %7.4f m/s   | %6.4f / %6.4f m" % ((name,) + r))
    row("all", S)
    if a.script:
        entries = sorted(yaml.safe_load(open(a.script)), key=lambda e: e["t"])
        cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        for i, e in enumerate(entries):
            if not any(k in e for k in cmd):
                continue
            for k in cmd:
                cmd[k] = e.get(k, cmd[k])
            end = entries[i + 1]["t"] if i + 1 < len(entries) else e["t"]
            W = [s for s in S if e["t"] <= s.controller_time < end]
            if W:
                row("vx %+.1f vy %+.1f wz %+.1f" % (cmd["vx"], cmd["vy"], cmd["wz"]), W)
    # Drift since the first Mpc sample (any offset present at that point is removed).
    ox, oy = S[0].kf_pos.x - S[0].gt_pos.x, S[0].kf_pos.y - S[0].gt_pos.y
    print("horizontal position drift (KF - truth) since t=%.1f s (initial offset %.4f m removed):"
          % (S[0].controller_time, math.hypot(ox, oy)))
    path, prev = 0.0, S[0]
    next_t = S[0].controller_time + 10.0
    for s in S[1:]:
        path += math.hypot(s.gt_pos.x - prev.gt_pos.x, s.gt_pos.y - prev.gt_pos.y)
        prev = s
        if s.controller_time >= next_t or s is S[-1]:
            d = math.hypot(s.kf_pos.x - s.gt_pos.x - ox, s.kf_pos.y - s.gt_pos.y - oy)
            print("  t=%5.1f s: %.4f m after %.2f m of path (%.2f %%)" % (s.controller_time, d, path,
                                                                     100 * d / max(path, 1e-9)))
            next_t += 10.0


if __name__ == "__main__":
    main()
