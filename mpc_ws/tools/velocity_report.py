#!/usr/bin/env python3
"""Velocity tracking report from a rosbag2 recording of /go2_mpc/mpc and /go2_mpc/status.

    python3 tools/velocity_report.py <bag dir> <script.yaml>

For every script segment with a velocity command it averages the body-frame velocity
over the segment's last `--window` seconds (default 2 s) and compares it with the
command. The linear velocity is the simulator's ground truth when recorded (so the
result does not depend on the state estimate the controller used), otherwise the
controller's own state. Run inside the go2-mpc container after `source env.sh`.
"""
import argparse
import math

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message

from go2_mpc_msgs.msg import ControllerStatus, MpcDebug


def load(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    mpc, status = [], {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/go2_mpc/mpc":
            m = deserialize_message(data, MpcDebug)
            if m.has_solution:
                mpc.append(m)
        elif topic == "/go2_mpc/status":
            s = deserialize_message(data, ControllerStatus)
            status[(s.header.stamp.sec, s.header.stamp.nanosec)] = s
    samples = []
    for m in mpc:
        s = status.get((m.header.stamp.sec, m.header.stamp.nanosec))
        if s is None:
            continue
        if s.ground_truth_available:  # judge tracking against the truth
            m.state[9], m.state[10] = s.gt_vel.x, s.gt_vel.y
        samples.append((s.controller_time, m))
    return samples


def segments(script):
    entries = sorted(yaml.safe_load(open(script)), key=lambda e: e["t"])
    cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    out = []
    for i, e in enumerate(entries):
        if not any(k in e for k in cmd):
            continue
        for k in cmd:
            cmd[k] = e.get(k, cmd[k])
        end = entries[i + 1]["t"] if i + 1 < len(entries) else e["t"]
        out.append((e["t"], end, dict(cmd)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("script")
    ap.add_argument("--window", type=float, default=2.0)
    a = ap.parse_args()
    samples = load(a.bag)
    print("%-26s | %-20s | %-20s | %-17s | %s" % ("segment [s]", "command vx vy wz", "measured vx vy wz",
                                                    "error", "max |roll| |pitch|"))
    for start, end, c in segments(a.script):
        w = [m for t, m in samples if end - a.window <= t < end]
        seg = [m for t, m in samples if start <= t < end]
        if not w:
            continue
        def body_v(m):
            yaw, vx, vy = m.state[2], m.state[9], m.state[10]
            return (math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy)
        bv = [body_v(m) for m in w]
        vx = sum(v[0] for v in bv) / len(bv)
        vy = sum(v[1] for v in bv) / len(bv)
        wz = sum(m.state[8] for m in w) / len(w)
        def rel(meas, cmd):
            return "%+5.1f%%" % (100 * (meas - cmd) / cmd) if abs(cmd) > 1e-6 else "%+.3f" % meas
        err = " ".join(rel(m_, c_) for m_, c_ in ((vx, c["vx"]), (vy, c["vy"]), (wz, c["wz"])) if abs(c_) > 1e-6)
        if not err:
            err = "drift %.3f m/s" % math.hypot(vx, vy)
        print("%5.1f-%5.1f %-14s | %+5.2f %+5.2f %+5.2f   | %+5.2f %+5.2f %+5.2f   | %-17s | %.3f %.3f" % (
            start, end, "", c["vx"], c["vy"], c["wz"], vx, vy, wz, err,
            max(abs(m.state[0]) for m in seg), max(abs(m.state[1]) for m in seg)))
    fz = max(max(m.grf[2], m.grf[5], m.grf[8], m.grf[11]) for _, m in samples)
    print("max single-foot fz %.1f N" % fz)


if __name__ == "__main__":
    main()
